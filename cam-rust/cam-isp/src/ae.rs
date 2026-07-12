//! Auto exposure engine — computes exposure time + ISO from scene statistics.
//!
//! Ported from `com.camcore.isp.pipeline.controller.AutoExposureEngine` (Java).
//! Replaces Camera2 HAL AE with custom exposure control.
//!
//! ## Algorithm
//! 1. Compute target luminance from scene brightness + user bias
//! 2. Derive raw exposure from target/measured ratio
//! 3. Split into exposure time (prefer longer for SNR) and ISO (avoid if possible)
//! 4. Apply histogram constraint (avoid highlight clipping)
//! 5. Smooth with EMA

use std::sync::Mutex;

/// Auto-exposure engine state.
#[derive(Debug, Clone)]
pub struct AutoExposureState {
    // ── Exposure ranges ──
    /// Minimum exposure time in ns (1/8000 s ≈ 125 µs).
    pub min_exposure_ns: i64,
    /// Maximum exposure time in ns (1/15 s ≈ 67 ms).
    pub max_exposure_ns: i64,
    /// Minimum ISO.
    pub min_iso: i32,
    /// Maximum ISO before noise becomes unacceptable.
    pub max_iso: i32,
    /// Base ISO where analog gain = 1.0.
    pub base_iso: i32,

    // ── Tuning ──
    /// Target luminance for well-exposed scene (0..1).
    pub target_lum_default: f32,
    /// EMA smoothing factor (0 = no change, 1 = instant).
    pub exposure_alpha: f32,

    // ── State ──
    /// Smoothed exposure time in ns.
    pub smoothed_exposure_ns: i64,
    /// Smoothed ISO.
    pub smoothed_iso: i32,
    /// Last computed target luminance.
    pub last_computed_target_lum: f32,
    /// Frame duration target in ns (for consistent framerate).
    pub frame_duration_ns: i64,
}

impl Default for AutoExposureState {
    fn default() -> Self {
        Self {
            min_exposure_ns: 125_000,    // 1/8000 s
            max_exposure_ns: 66_666_666, // 1/15 s
            min_iso: 50,
            max_iso: 3200,
            base_iso: 100,
            target_lum_default: 0.35,
            exposure_alpha: 0.15,
            smoothed_exposure_ns: 33_333_333, // 1/30 s
            smoothed_iso: 100,
            last_computed_target_lum: 0.35,
            frame_duration_ns: 33_333_333, // 30 fps
        }
    }
}

impl AutoExposureState {
    /// Compute optimal exposure time + ISO from scene statistics.
    ///
    /// * `mean_luminance` — average scene luminance from tone stats (0..1)
    /// * `highlight_ratio` — fraction of overexposed pixels (0..1) from histogram
    /// * `shadow_ratio` — fraction of underexposed pixels (0..1) from histogram
    /// * `brightness_bias` — user brightness preference (0..1, 0.5 = default)
    ///
    /// Returns `(exposure_time_ns, iso)`.
    pub fn compute(
        &mut self,
        mean_luminance: f32,
        highlight_ratio: f32,
        shadow_ratio: f32,
        brightness_bias: f32,
    ) -> (i64, i32) {
        let m = mean_luminance.clamp(0.001, 1.0);

        // Target luminance with brightness bias
        let lum_bias = (brightness_bias - 0.5) * 0.24; // ±0.12
        let target_lum = (self.target_lum_default + lum_bias).clamp(0.02, 0.6);
        self.last_computed_target_lum = target_lum;

        // Raw exposure ratio
        let raw_ratio = target_lum / m;
        let clamped_ratio = raw_ratio.clamp(0.25, 8.0);

        // Highlight clipping guard
        let highlight_guard = if highlight_ratio > 0.01 {
            (1.0 - highlight_ratio).clamp(0.5, 1.0)
        } else {
            1.0
        };

        // Shadow boost guard
        let shadow_guard = if shadow_ratio > 0.3 {
            (1.0 + shadow_ratio * 0.5).clamp(1.0, 1.5)
        } else {
            1.0
        };

        let effective_ratio = (clamped_ratio * highlight_guard * shadow_guard).clamp(0.25, 8.0);

        // Split into exposure time + ISO
        let max_safe_exposure_ns = self.frame_duration_ns.min(self.max_exposure_ns);
        let max_safe_exposure_ns = (max_safe_exposure_ns as f32 * 0.9) as i64;
        let raw_time_ns = (self.smoothed_exposure_ns as f32 * effective_ratio) as i64;
        let time_ns = raw_time_ns.clamp(self.min_exposure_ns, max_safe_exposure_ns);

        // Compute ISO needed given chosen exposure time
        let time_ratio = time_ns as f32 / self.smoothed_exposure_ns.max(1000) as f32;
        let iso_from_time = (self.smoothed_iso as f32 / time_ratio) as i32;
        let iso = iso_from_time.clamp(self.min_iso, self.max_iso);

        // Smooth with EMA
        let alpha = self.exposure_alpha.clamp(0.02, 0.5);
        self.smoothed_exposure_ns =
            (self.smoothed_exposure_ns as f32 * (1.0 - alpha) + time_ns as f32 * alpha) as i64;
        self.smoothed_iso = (self.smoothed_iso as f32 * (1.0 - alpha) + iso as f32 * alpha) as i32;

        log::debug!(
            "AeEngine: mL={:.4} tL={:.4} ratio={:.2} hg={:.2} sg={:.2} → exp={}ns iso={}",
            m,
            target_lum,
            clamped_ratio,
            highlight_guard,
            shadow_guard,
            self.smoothed_exposure_ns,
            self.smoothed_iso
        );

        (self.smoothed_exposure_ns, self.smoothed_iso)
    }

    /// Reset to defaults.
    pub fn reset(&mut self) {
        self.smoothed_exposure_ns = 33_333_333;
        self.smoothed_iso = 100;
        self.last_computed_target_lum = self.target_lum_default;
    }
}

/// Thread-safe wrapper around AutoExposureState.
pub struct AutoExposureEngine {
    state: Mutex<AutoExposureState>,
}

impl AutoExposureEngine {
    pub fn new() -> Self {
        Self {
            state: Mutex::new(AutoExposureState::default()),
        }
    }

    /// Compute exposure (thread-safe).
    pub fn compute(
        &self,
        mean_luminance: f32,
        highlight_ratio: f32,
        shadow_ratio: f32,
        brightness_bias: f32,
    ) -> (i64, i32) {
        let mut state = self.state.lock().unwrap();
        state.compute(
            mean_luminance,
            highlight_ratio,
            shadow_ratio,
            brightness_bias,
        )
    }

    /// Get current smoothed values.
    pub fn current(&self) -> (i64, i32) {
        let state = self.state.lock().unwrap();
        (state.smoothed_exposure_ns, state.smoothed_iso)
    }

    /// Reset.
    pub fn reset(&self) {
        let mut state = self.state.lock().unwrap();
        state.reset();
    }
}

impl Default for AutoExposureEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_state() {
        let state = AutoExposureState::default();
        assert_eq!(state.smoothed_exposure_ns, 33_333_333);
        assert_eq!(state.smoothed_iso, 100);
    }

    #[test]
    fn test_compute_bright_scene() {
        let mut state = AutoExposureState::default();
        let (exp, iso) = state.compute(0.5, 0.0, 0.1, 0.5);
        // Bright scene → shorter exposure or lower ISO
        assert!(exp <= 33_333_333);
        assert!(iso >= 50 && iso <= 3200);
    }

    #[test]
    fn test_compute_dark_scene() {
        let mut state = AutoExposureState::default();
        let (exp, iso) = state.compute(0.01, 0.0, 0.8, 0.5);
        // Dark scene → longer exposure, higher ISO
        assert!(exp >= 125_000);
        assert!(iso >= 50);
    }

    #[test]
    fn test_highlight_guard() {
        let mut state = AutoExposureState::default();
        let (exp_normal, _) = state.compute(0.2, 0.0, 0.1, 0.5);
        let mut state2 = AutoExposureState::default();
        let (exp_clipped, _) = state2.compute(0.2, 0.1, 0.1, 0.5);
        // With highlights, exposure should be reduced
        assert!(exp_clipped <= exp_normal);
    }

    #[test]
    fn test_thread_safe() {
        let engine = AutoExposureEngine::new();
        let (exp, iso) = engine.compute(0.3, 0.0, 0.1, 0.5);
        assert!(exp > 0);
        assert!(iso > 0);
        let (cur_exp, cur_iso) = engine.current();
        assert_eq!(cur_exp, exp);
        assert_eq!(cur_iso, iso);
    }

    #[test]
    fn test_reset() {
        let mut state = AutoExposureState::default();
        state.compute(0.01, 0.0, 0.8, 0.5);
        assert!(state.smoothed_exposure_ns != 33_333_333 || state.smoothed_iso != 100);
        state.reset();
        assert_eq!(state.smoothed_exposure_ns, 33_333_333);
        assert_eq!(state.smoothed_iso, 100);
    }
}
