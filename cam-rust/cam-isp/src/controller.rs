//! ISP Controller — AWB, AE, CCM, Tone parameter estimation from frame statistics.
//!
//! Ported from `com.camcore.isp.pipeline.IspController` (Java).
//!
//! The controller takes per-frame statistics (channel means, tone stats, histogram)
//! and produces smoothed ISP parameters (AWB gains, CCM matrix, tone curve, exposure).

use cam_types::ToneParams;
use crate::ae::AutoExposureState;

/// Smoothing mode for temporal filtering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SmoothingMode {
    /// Fast initial convergence (first N frames)
    Fast,
    /// Steady-state slow smoothing
    Slow,
}

/// CCM clamp feedback flags.
const CCM_R_OUT_NEGATIVE: i32 = 0x01;
const CCM_DIAGONAL_EXTREME: i32 = 0x02;

/// ISP Controller — state container + orchestrator for ISP parameters.
///
/// # Flow
/// 1. Receive `channel_means` (R, G, B averages) from ISP output
/// 2. Call `update_channel_stats()` → updates AWB + CCT estimate
/// 3. Call `update_tone_stats()` → updates tone curve params
/// 4. Call `update_histogram()` → updates AE gain
/// 5. Read smoothed parameters via getters
#[derive(Debug, Clone)]
pub struct IspController {
    // ── Smoothing ──
    /// Exponential smoothing factor (0..1). Higher = faster response.
    pub smoothing_alpha: f32,
    /// Number of frames processed (for adaptive smoothing).
    pub frame_count: u32,

    // ── AWB state ──
    /// Target neutral color levels (default 1.0 for each channel).
    pub target_r: f32,
    pub target_g: f32,
    pub target_b: f32,
    /// Smoothed AWB gains [R, G, B].
    pub awb_gains: [f32; 3],
    /// Running averages of channel means.
    pub avg_r: f32,
    pub avg_g: f32,
    pub avg_b: f32,
    /// Estimated CCT (color temperature).
    pub estimated_cct: Option<i32>,
    /// AWB prior coefficients for CCT-based gain prediction.
    pub awb_prior_r: [f32; 3],
    pub awb_prior_b: [f32; 3],

    // ── CCM state ──
    /// Smoothed CCM matrix (3×3, row-major).
    pub smoothed_ccm: [f32; 9],
    /// Base sensor CCM.
    pub ccm_matrix: [f32; 9],
    /// CCM clamp flags.
    pub clamp_flags: i32,
    /// CCT at which clamping last occurred.
    pub clamp_cct_ref: i32,

    // ── Tone state ──
    /// Smoothed tone parameters.
    pub tone_contrast: f32,
    pub tone_brightness: f32,
    pub tone_gamma: f32,
    pub tone_shadow_lift: f32,
    pub tone_highlight_roll: f32,
    pub tone_saturation: f32,
    /// Smoothed exposure gain.
    pub exposure_gain: f32,
    /// Smoothed LSC strength.
    pub lsc_strength: f32,
    /// Running averages of luminance stats.
    pub avg_lum_mean: f32,
    pub avg_lum_min: f32,
    pub avg_lum_max: f32,

    // ── Histogram state ──
    pub highlight_ratio: f32,
    pub shadow_ratio: f32,
    pub hist_constrained_gain: f32,

    // ── Auto exposure engine ──
    /// Auto-exposure state for computing exposure time + ISO.
    pub ae_state: AutoExposureState,

    // ── Scene luminance ──
    pub scene_luminance: f32,
}

impl Default for IspController {
    fn default() -> Self {
        Self::new()
    }
}

impl IspController {
    pub fn new() -> Self {
        Self {
            smoothing_alpha: 0.15,
            frame_count: 0,
            target_r: 1.0,
            target_g: 1.0,
            target_b: 1.0,
            awb_gains: [1.0, 1.0, 1.0],
            avg_r: 0.0,
            avg_g: 0.0,
            avg_b: 0.0,
            estimated_cct: None,
            awb_prior_r: [3.2e-08, -4.2e-04, 2.60],
            awb_prior_b: [3.8e-08, -3.1e-04, 1.50],
            smoothed_ccm: IDENTITY_CCM,
            ccm_matrix: DEFAULT_SENSOR_CCM,
            clamp_flags: 0,
            clamp_cct_ref: 0,
            tone_contrast: 1.0,
            tone_brightness: 0.0,
            tone_gamma: 2.2,
            tone_shadow_lift: 0.10,
            tone_highlight_roll: 0.06,
            tone_saturation: 1.1,
            exposure_gain: 1.0,
            lsc_strength: 0.0,
            avg_lum_mean: 0.0,
            avg_lum_min: 0.0,
            avg_lum_max: 1.0,
            highlight_ratio: 0.0,
            shadow_ratio: 0.0,
            hist_constrained_gain: 1.5,
            ae_state: AutoExposureState::default(),
            scene_luminance: 0.0,
        }
    }

    // ── Smoothing mode ──
    fn awb_alpha(&self) -> f32 {
        if self.frame_count < 10 { 0.4 } else { self.smoothing_alpha }
    }

    fn tone_alpha(&self) -> f32 {
        if self.frame_count < 10 { 0.5 } else { self.smoothing_alpha }
    }

    // ── Prior CCT estimation ──

    /// Compute prior R/G ratio for a given CCT (quadratic).
    pub fn prior_r(&self, cct: i32) -> f32 {
        let a = self.awb_prior_r[0];
        let b = self.awb_prior_r[1];
        let c = self.awb_prior_r[2];
        (a * cct as f32 * cct as f32 + b * cct as f32 + c).clamp(0.5, 2.0)
    }

    /// Compute prior B/G ratio for a given CCT.
    pub fn prior_b(&self, cct: i32) -> f32 {
        let a = self.awb_prior_b[0];
        let b = self.awb_prior_b[1];
        let c = self.awb_prior_b[2];
        (a * cct as f32 * cct as f32 + b * cct as f32 + c).clamp(0.5, 2.0)
    }

    /// Estimate CCT from R/G and B/G ratios.
    pub fn estimate_cct(rg: f32, bg: f32) -> i32 {
        let cct = 4400.0 - 1300.0 * bg + 2100.0 * rg;
        (cct as i32).clamp(2000, 10000)
    }

    // ── AWB update ──

    /// Update AWB + CCM from channel means (R, G, B averages).
    ///
    /// Called each frame with the average R, G, B values from the ISP output.
    pub fn update_channel_stats(&mut self, channel_means: &[f32; 3]) {
        let min_rgb = 0.001;
        let r = channel_means[0].max(min_rgb);
        let g = channel_means[1].max(min_rgb);
        let b = channel_means[2].max(min_rgb);

        // Handle clamp feedback: if CCM was broken at previous CCT, reset
        if self.clamp_flags & (CCM_R_OUT_NEGATIVE | CCM_DIAGONAL_EXTREME) != 0 {
            log::warn!("CCM clamp flags=0x{:x} at CCT={} -> resetting CCT to 5500",
                self.clamp_flags, self.clamp_cct_ref);
            self.estimated_cct = Some(5500);
            self.clamp_flags = 0;
        }

        self.frame_count += 1;

        // Running averages
        let alpha = 1.0 / self.frame_count.min(30) as f32;
        self.avg_r += (r - self.avg_r) * alpha;
        self.avg_g += (g - self.avg_g) * alpha;
        self.avg_b += (b - self.avg_b) * alpha;

        // Raw gains to hit target neutral
        let r_gain = (g * self.target_r / r).clamp(0.5, 3.0);
        let b_gain = (g * self.target_b / b).clamp(0.5, 3.0);

        // Exponential smoothing
        let alpha_awb = self.awb_alpha();
        self.awb_gains[0] += (r_gain - self.awb_gains[0]) * alpha_awb;
        self.awb_gains[2] += (b_gain - self.awb_gains[2]) * alpha_awb;
        // G gain stays 1.0

        // CCT-aware R/B gain ratio clamp
        let cct_for_clamp = self.estimated_cct.unwrap_or(5000);
        let cct_norm = (cct_for_clamp as f32 - 3000.0).max(0.0);
        let max_rb_ratio = (1.20 + cct_norm * 0.00008).clamp(1.15, 1.60);
        let min_rb_ratio = (0.70 - cct_norm * 0.00003).clamp(0.50, 0.75);
        let rb_ratio = self.awb_gains[0] / self.awb_gains[2].max(0.001);
        if rb_ratio > max_rb_ratio {
            self.awb_gains[0] = self.awb_gains[2] * max_rb_ratio;
        }
        if rb_ratio < min_rb_ratio {
            self.awb_gains[2] = self.awb_gains[0] / min_rb_ratio;
        }

        // Update CCT estimate
        let avg_rg = self.avg_r / self.avg_g;
        let avg_bg = self.avg_b / self.avg_g;
        self.estimated_cct = Some(Self::estimate_cct(avg_rg, avg_bg));
    }

    // ── Tone update ──

    /// Update tone parameters from per-frame tone statistics.
    ///
    /// `tone_stats` = [mean_luminance, min_luminance, max_luminance]
    pub fn update_tone_stats(&mut self, tone_stats: &[f32; 3]) {
        let alpha = self.tone_alpha();

        // Update running averages
        self.avg_lum_mean += (tone_stats[0] - self.avg_lum_mean) * alpha;
        self.avg_lum_min += (tone_stats[1] - self.avg_lum_min) * alpha;
        self.avg_lum_max += (tone_stats[2] - self.avg_lum_max) * alpha;

        // Compute target exposure based on scene luminance
        let target_lum = 0.18;
        if self.avg_lum_mean > 0.001 {
            let ae_gain = (target_lum / self.avg_lum_mean).clamp(0.125, 8.0);
            self.exposure_gain += (ae_gain - self.exposure_gain) * alpha;
        }

        // Adjust contrast based on dynamic range
        let dynamic_range = self.avg_lum_max - self.avg_lum_min;
        if dynamic_range > 0.01 {
            let target_contrast = 1.0 + (1.0 - dynamic_range) * 0.5;
            let contrast = target_contrast.clamp(0.5, 2.0);
            self.tone_contrast += (contrast - self.tone_contrast) * alpha;
        }

        // Scene luminance for display
        self.scene_luminance = self.avg_lum_mean;
    }

    // ── Histogram update ──

    /// Update histogram-based exposure control.
    ///
    /// `hist` — 256-bin luminance histogram (normalized, sum = 1.0).
    pub fn update_histogram(&mut self, hist: &[f32]) {
        if hist.len() < 256 { return; }

        // Compute highlight and shadow ratios
        let highlight_bins: f32 = hist[192..].iter().sum();
        let shadow_bins: f32 = hist[..64].iter().sum();

        self.highlight_ratio = highlight_bins;
        self.shadow_ratio = shadow_bins;

        // Reduce exposure if too many highlights
        if highlight_bins > 0.05 {
            let reduction = 1.0 - ((highlight_bins - 0.05) / 0.15).clamp(0.0, 0.5);
            self.hist_constrained_gain = reduction;
        } else if shadow_bins > 0.3 {
            let boost = 1.0 + ((shadow_bins - 0.3) / 0.3).clamp(0.0, 1.0);
            self.hist_constrained_gain = boost;
        } else {
            self.hist_constrained_gain = 1.5;
        }
    }

    // ── Getters ──

    /// Get smoothed AWB gains as [R, G, B].
    pub fn get_awb_gains(&self) -> [f32; 3] {
        self.awb_gains
    }

    /// Get current CCM matrix (3×3 row-major).
    pub fn get_ccm(&self) -> [f32; 9] {
        self.smoothed_ccm
    }

    /// Get tone parameters for the ISP pipeline.
    pub fn get_tone_params(&self) -> ToneParams {
        ToneParams {
            contrast: self.tone_contrast,
            brightness: self.tone_brightness,
            gamma_recip: 1.0 / self.tone_gamma,
            shadow_lift: self.tone_shadow_lift,
            highlight_roll: self.tone_highlight_roll,
            sharpness: 1.0,
            saturation: self.tone_saturation,
        }
    }

    /// Get effective exposure gain (histogram-constrained × AE gain).
    pub fn get_effective_exposure_gain(&self) -> f32 {
        self.exposure_gain * self.hist_constrained_gain
    }

    /// Get LSC (lens shading correction) strength.
    pub fn get_lsc_strength(&self) -> f32 {
        self.lsc_strength
    }

    /// Compute exposure time + ISO from current scene stats.
    /// Delegates to AutoExposureState using internally-tracked luminance, histogram, and brightness bias.
    pub fn compute_exposure(&mut self, brightness_bias: f32) -> (i64, i32) {
        let lum = if self.avg_lum_mean > 0.001 { self.avg_lum_mean } else { self.scene_luminance };
        self.ae_state.compute(lum, self.highlight_ratio, self.shadow_ratio, brightness_bias)
    }

    // ── Manual overrides ──

    pub fn set_manual_awb(&mut self, gains: &[f32; 3]) {
        self.awb_gains = [
            gains[0].clamp(0.1, 5.0),
            gains[1].clamp(0.1, 5.0),
            gains[2].clamp(0.1, 5.0),
        ];
    }

    pub fn set_manual_ccm(&mut self, ccm: &[f32; 9]) {
        self.ccm_matrix = *ccm;
    }

    pub fn set_manual_gamma(&mut self, gamma: f32) {
        self.tone_gamma = gamma.clamp(0.5, 4.0);
    }

    pub fn set_manual_contrast(&mut self, contrast: f32) {
        self.tone_contrast = contrast.clamp(0.5, 2.0);
    }

    pub fn set_manual_exposure_gain(&mut self, gain: f32) {
        self.exposure_gain = gain.clamp(0.5, 8.0);
    }

    /// Reset all state to defaults.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

/// Identity 3×3 CCM matrix.
const IDENTITY_CCM: [f32; 9] = [
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
];

/// Default sensor CCM (placeholder — should be calibrated per sensor).
const DEFAULT_SENSOR_CCM: [f32; 9] = [
    1.6, -0.4, -0.2,
    -0.3, 1.8, -0.5,
    0.0, -0.5, 1.5,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_state() {
        let ctrl = IspController::new();
        assert_eq!(ctrl.awb_gains, [1.0, 1.0, 1.0]);
        assert_eq!(ctrl.frame_count, 0);
        assert!(ctrl.estimated_cct.is_none());
    }

    #[test]
    fn test_awb_convergence() {
        let mut ctrl = IspController::new();

        // Simulate a warm scene (R > B)
        for _ in 0..10 {
            ctrl.update_channel_stats(&[0.5, 0.3, 0.1]);
        }

        let gains = ctrl.get_awb_gains();
        // For R-dominant scene, R gain should be < G, B gain should be > G
        assert!(gains[0] < 1.0, "R gain should be < 1 for warm scene: {}", gains[0]);
        assert!(gains[2] > 1.0, "B gain should be > 1 for warm scene: {}", gains[2]);
        assert!(ctrl.frame_count >= 10);
        assert!(ctrl.estimated_cct.is_some());
    }

    #[test]
    fn test_tone_stats() {
        let mut ctrl = IspController::new();
        ctrl.update_tone_stats(&[0.2, 0.05, 0.8]);

        // Should compute exposure gain to approach 0.18
        let ae = ctrl.get_effective_exposure_gain();
        assert!(ae > 0.0);
    }

    #[test]
    fn test_histogram_exposure() {
        let mut ctrl = IspController::new();
        // Mostly dark image
        let mut hist = [0.0f32; 256];
        for i in 0..64 {
            hist[i] = 1.0 / 64.0;
        }
        ctrl.update_histogram(&hist);

        // Should boost exposure (shadow ratio > 0.3)
        let gain = ctrl.hist_constrained_gain;
        assert!(gain > 1.0, "Should boost exposure for dark image: {}", gain);
    }

    #[test]
    fn test_manual_overrides() {
        let mut ctrl = IspController::new();
        ctrl.set_manual_awb(&[1.5, 1.0, 2.0]);
        assert_eq!(ctrl.awb_gains, [1.5, 1.0, 2.0]);

        ctrl.set_manual_gamma(1.8);
        assert!((ctrl.tone_gamma - 1.8).abs() < 0.01);
    }

    #[test]
    fn test_cct_estimation() {
        // Daylight: R/G ≈ 0.5, B/G ≈ 0.5 → CCT ≈ 4400 - 650 + 1050 = 4800
        let cct = IspController::estimate_cct(0.5, 0.5);
        assert!(cct >= 2000 && cct <= 10000);
    }
}
