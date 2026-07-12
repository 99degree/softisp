//! ISP Controller — AWB, AE, CCM, Tone parameter estimation from frame statistics.
//!
//! Ported from `com.camcore.isp.pipeline.IspController` (Java).
//!
//! The controller takes per-frame statistics (channel means, tone stats, histogram)
//! and produces smoothed ISP parameters (AWB gains, CCM matrix, tone curve, exposure).

use crate::ae::AutoExposureState;
use crate::ccm_engine::{self, select_ccm};
use crate::rolling_stats::RollingStats;
use crate::scene::SceneCategory;

/// Smoothing mode for temporal filtering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SmoothingMode {
    /// Fast initial convergence (first N frames)
    Fast,
    /// Steady-state slow smoothing
    Slow,
}

// CCM clamp feedback flags — imported from ccm_engine module

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
    /// Smoothed AWB gains `[R, G, B]`.
    pub awb_gains: [f32; 3],
    /// Running averages of channel means.
    pub avg_r: f32,
    pub avg_g: f32,
    pub avg_b: f32,
    /// Raw R/G and B/G ratios for diagnostics.
    pub last_rg: f32,
    pub last_bg: f32,
    /// Estimated CCT (color temperature).
    pub estimated_cct: Option<i32>,
    /// Minimum scene luminance for CCT estimation.
    pub cct_lum_threshold: f32,
    /// AWB prior coefficients for CCT-based gain prediction.
    pub awb_prior_r: [f32; 3],
    pub awb_prior_b: [f32; 3],

    // ── CCM state ──
    /// Smoothed CCM matrix (3×3, row-major).
    pub smoothed_ccm: [f32; 9],
    /// Base sensor CCM.
    pub ccm_matrix: [f32; 9],
    /// CCM per-sensor scale (9-element, default all 1.0).
    pub ccm_scale_a: [f32; 9],
    /// CCM per-sensor offset (9-element, default all 0.0).
    pub ccm_offset_c: [f32; 9],
    /// CCM clamp flags.
    pub clamp_flags: i32,
    /// CCT at which clamping last occurred.
    pub clamp_cct_ref: i32,
    /// CCM output EMA smoothing factor (1.0 = no smoothing).
    pub ccm_smoothing: f32,
    /// Whether CCM has been initialized (for EMA start).
    pub ccm_initialized: bool,

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

    // ── Scene change tracking ──
    /// Previous frame's channel means for scene change detection.
    pub prev_channel_means: Option<[f32; 3]>,
    /// Scene change metric (0..1), computed from channel mean deltas.
    pub scene_change: f32,

    // ── User brightness control ──
    /// Normalized brightness bias (0..1, 0.5 = default).
    pub brightness_bias: f32,

    // ── Rolling stats tensors (triple-buffered) ──
    /// Three slots for lock-free rolling:
    ///   write_idx: engine writes frame N's stats here after inference
    ///   process_idx: controller reads stats from here for parameter update
    ///   ready_idx: engine reads params from here for next inference
    /// After each frame, indices rotate: write → process → ready → write
    pub stats_slots: [RollingStats; 3],
    /// Index of the slot the engine writes to (stats output from inference).
    pub write_idx: usize,
    /// Index of the slot the controller reads for processing.
    pub process_idx: usize,
    /// Index of the slot with ready-to-use params for the next inference.
    pub ready_idx: usize,
    /// Last applied brightness value.
    pub last_applied_brightness: f32,

    // ── Analog gain / exposure from camera HAL ──
    pub analog_gain: f32,
    pub exposure_time_ns: i64,

    // ── Zone stats (6×8 multi-illuminant AWB) ──
    /// Number of zone rows.
    pub zone_rows: usize,
    /// Number of zone columns.
    pub zone_cols: usize,
    /// Per-zone RGB means `[row]``[col]` = `[R, G, B]`.
    pub zone_rgb: Vec<Vec<[f32; 3]>>,
    /// Per-zone luminance.
    pub zone_lum: Vec<Vec<f32>>,
    /// Per-zone CCT (raw).
    pub zone_cct: Vec<Vec<f32>>,
    /// Temporally smoothed per-zone CCT.
    pub smoothed_zone_cct: Vec<Vec<f32>>,
    /// Whether zone CCT has been initialized.
    pub zone_cct_initialized: bool,
    /// Zone weights (center-weighted).
    pub zone_weight: Vec<f32>,
    /// Dominant CCT cluster (0=warm, 1=mid, 2=cool, -1=mixed).
    pub dominant_cct_cluster: Option<i32>,
    /// Fraction of zones in the dominant cluster.
    pub dominant_cluster_fraction: f32,
    /// Zone stats enabled flag.
    pub zone_stats_enabled: bool,

    // ── Scene adaptivity ──
    /// Current scene category for adaptive ISP tuning.
    pub scene_category: SceneCategory,
    /// CCT from last frame (for scene classification).
    pub last_cct_for_scene: u32,
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
            last_rg: 1.0,
            last_bg: 1.0,
            estimated_cct: None,
            cct_lum_threshold: 0.02,
            awb_prior_r: [3.2e-08, -4.2e-04, 2.60],
            awb_prior_b: [3.8e-08, -3.1e-04, 1.50],
            smoothed_ccm: ccm_engine::identity_ccm(),
            ccm_matrix: ccm_engine::default_sensor_ccm(),
            ccm_scale_a: [1.0f32; 9],
            ccm_offset_c: [0.0f32; 9],
            clamp_flags: 0,
            clamp_cct_ref: 0,
            ccm_smoothing: 0.85,
            ccm_initialized: false,
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
            prev_channel_means: None,
            scene_change: 0.0,
            brightness_bias: 0.5,
            last_applied_brightness: 0.5,
            analog_gain: 1.0,
            exposure_time_ns: 33_333_333,
            zone_rows: 6,
            zone_cols: 8,
            zone_rgb: Vec::new(),
            zone_lum: Vec::new(),
            zone_cct: Vec::new(),
            smoothed_zone_cct: Vec::new(),
            zone_cct_initialized: false,
            zone_weight: Vec::new(),
            dominant_cct_cluster: None,
            dominant_cluster_fraction: 0.0,
            zone_stats_enabled: false,
            scene_category: SceneCategory::Unknown,
            last_cct_for_scene: 5500,
            stats_slots: [
                RollingStats::new(),
                RollingStats::new(),
                RollingStats::new(),
            ],
            write_idx: 0,
            process_idx: 1,
            ready_idx: 2,
        }
    }

    // ── Smoothing mode ──

    // ── Rolling stats ──

    /// Rotate the triple-buffered stats slots after inference.
    /// Called by the engine after writing frame N's stats to `write_idx`.
    /// Rotates: write → process → ready → write.
    pub fn rotate_stats(&mut self) {
        self.write_idx = (self.write_idx + 1) % 3;
        self.process_idx = (self.process_idx + 1) % 3;
        self.ready_idx = (self.ready_idx + 1) % 3;
    }

    /// Get a mutable reference to the write slot (engine writes stats here).
    pub fn write_stats(&mut self) -> &mut RollingStats {
        &mut self.stats_slots[self.write_idx]
    }

    /// Get a reference to the process slot (controller reads for param update).
    pub fn process_stats(&self) -> &RollingStats {
        &self.stats_slots[self.process_idx]
    }

    /// Get a reference to the ready slot (engine reads params from here).
    pub fn ready_stats(&self) -> &RollingStats {
        &self.stats_slots[self.ready_idx]
    }

    /// Get current CCM matrix (3×3 row-major).

    /// Get tone parameters for the ISP pipeline.

    /// Get effective exposure gain (histogram-constrained × AE gain).

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

    /// Reset all state to defaults.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    // ── ControllerApi ONNX override methods ──

    /// Apply ONNX AWB gains with EMA smoothing and B-gain CCT capping.
    pub fn set_onnx_awb_gains(&mut self, gains: &[f32; 3], alpha: f32) {
        let raw_r = gains[0].clamp(0.1, 5.0);
        let mut raw_b = gains[2].clamp(0.1, 5.0);
        let cct = self.estimated_cct.unwrap_or(5000) as f32;
        let diff_to_center = cct - 5250.0;
        let warm_slope = if diff_to_center < 0.0 { 0.0006 } else { 0.0003 };
        let max_b = (1.4 + diff_to_center.abs() * warm_slope).clamp(1.4, 2.8);
        if raw_b > max_b {
            log::debug!(
                "AWB: B capped from {:.2} to {:.2} (CCT={:.0})",
                raw_b,
                max_b,
                cct
            );
            raw_b = max_b;
        }
        // Use the same smoothed fields (avg_r/avg_b serve as smoothed state)
        self.avg_r += (raw_r - self.avg_r) * alpha;
        self.avg_b += (raw_b - self.avg_b) * alpha;
        self.awb_gains = [self.avg_r, 1.0, self.avg_b];
    }

    /// Apply ONNX gamma override with EMA.
    pub fn set_onnx_gamma(&mut self, gamma: f32, alpha: f32) {
        let clamped = gamma.clamp(0.5, 4.0);
        // Use smoothing_alpha field for gamma smoothing if we had one;
        // for now the tone_gamma field itself is the smoothed state
        self.tone_gamma += (clamped - self.tone_gamma) * alpha;
    }

    /// Apply ONNX exposure gain with EMA.
    pub fn set_onnx_exposure_gain(&mut self, gain: f32, alpha: f32) {
        let clamped = gain.clamp(0.25, 8.0);
        self.exposure_gain += (clamped - self.exposure_gain) * alpha;
    }

    /// Apply ONNX CCT override with EMA and CCM recompute.
    pub fn set_onnx_cct(&mut self, cct: f32, alpha: f32) {
        let target = (cct as i32).clamp(2000, 10000);
        let prev = self.estimated_cct.unwrap_or(5500);
        let smoothed = (prev as f32 + (target - prev) as f32 * alpha) as i32;
        self.estimated_cct = Some(smoothed.clamp(2000, 10000));
        self.scene_luminance = 0.5;
        // Recompute CCM with the overridden CCT
        let ccm = select_ccm(self.estimated_cct.unwrap_or(5500));
        if ccm.len() == 9 {
            let mut arr = [0.0f32; 9];
            arr.copy_from_slice(&ccm[..9]);
            self.ccm_matrix = arr;
        }
    }

    /// Update from hardware ISP parameters (Camera2 HAL).
    pub fn update_from_hardware(
        &mut self,
        hw_awb: Option<&[f32; 4]>,
        hw_ccm: Option<&[f32; 9]>,
        _hw_gamma: Option<f32>,
    ) {
        if let Some(awb) = hw_awb {
            let r_g = awb[0].clamp(0.1, 5.0);
            let b_g = awb[3].clamp(0.1, 5.0);
            let alpha = self.smoothing_alpha;
            self.avg_r += (r_g - self.avg_r) * alpha;
            self.avg_b += (b_g - self.avg_b) * alpha;
            self.awb_gains = [self.avg_r, 1.0, self.avg_b];
        }
        if let Some(ccm) = hw_ccm {
            // Use Camera2 CCM directly (bypasses A·x + C model)
            let mut ccm_copy = *ccm;
            crate::ccm_engine::sanitize_ccm(
                &mut ccm_copy,
                self.estimated_cct.unwrap_or(5500),
                "hw",
                Some(&mut self.clamp_flags),
                Some(&mut self.clamp_cct_ref),
            );
            self.ccm_matrix = ccm_copy;
            self.smoothed_ccm = ccm_copy;
        }
        // hw_gamma is ignored — Camera2 tone map is linear in raw mode
    }

    /// Full ONNX update: stats + ONNX overrides.
    pub fn update_from_onnx(
        &mut self,
        channel_means: Option<&[f32; 3]>,
        tone_stats: Option<&[f32; 3]>,
        histogram: Option<&[f32]>,
        zone_stats: Option<&[f32]>,
        ae_gain: Option<f32>,
        cct: Option<f32>,
        awb_gains: Option<&[f32; 3]>,
        algo_gamma: Option<f32>,
    ) {
        if let Some(cm) = channel_means {
            self.update_channel_stats(cm);
        }
        if let Some(ts) = tone_stats {
            self.update_tone_stats(ts);
        }
        if let Some(h) = histogram {
            self.update_histogram(h);
        }
        if let Some(zs) = zone_stats {
            self.update_zone_stats(zs);
        }
        if let Some(gain) = ae_gain {
            self.set_onnx_exposure_gain(gain, 0.3);
        }
        if let Some(cct_val) = cct {
            self.set_onnx_cct(cct_val, 0.3);
            // Recompute CCM with the overridden CCT
            let ccm = select_ccm(self.estimated_cct.unwrap_or(5500));
            if ccm.len() == 9 {
                let mut arr = [0.0f32; 9];
                arr.copy_from_slice(&ccm[..9]);
                self.ccm_matrix = arr;
            }
        }
        if let Some(awb) = awb_gains {
            self.set_onnx_awb_gains(awb, 0.3);
        }
        if let Some(gamma) = algo_gamma {
            self.set_onnx_gamma(gamma, 0.3);
        }
    }
}

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
        // AWB should have moved from initial [1,1,1] — at least one gain should differ
        let changed = (gains[0] - 1.0).abs() > 0.01 || (gains[2] - 1.0).abs() > 0.01;
        assert!(
            changed,
            "AWB should have adapted: gains={:.3} {:.3} {:.3}",
            gains[0], gains[1], gains[2]
        );
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

    #[test]
    fn test_ccm_composition() {
        let mut ctrl = IspController::new();
        // Simulate a scene to get CCT estimated
        ctrl.frame_count = 5;
        ctrl.estimated_cct = Some(5500);
        ctrl.avg_r = 0.5;
        ctrl.avg_g = 0.5;
        ctrl.avg_b = 0.5;

        // Update CCM
        ctrl.update_channel_stats(&[0.5, 0.5, 0.5]);

        // CCM should be initialized
        assert!(ctrl.ccm_initialized);
        // All rows should sum to ≈1.0
        let ccm = ctrl.ccm_matrix;
        for row in 0..3 {
            let base = row * 3;
            let sum: f32 = ccm[base..base + 3].iter().sum();
            assert!((sum - 1.0).abs() < 0.01, "Row {} sum = {}", row, sum);
        }
        // R-B and B-R should be negative (anti-purple)
        assert!(ccm[2] < 0.0, "R-B should be negative: {}", ccm[2]);
        assert!(ccm[6] < 0.0, "B-R should be negative: {}", ccm[6]);
    }

    #[test]
    fn test_ccm_scale_offset() {
        let mut ctrl = IspController::new();
        ctrl.estimated_cct = Some(5500);
        ctrl.ccm_smoothing = 1.0; // no smoothing for test

        // Set scale = 2x diagonal, offset = 0.1 on diagonal
        ctrl.ccm_scale_a = [2.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 2.0];
        ctrl.ccm_offset_c = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];

        ctrl.update_channel_stats(&[0.5, 0.5, 0.5]);

        // After EMA init: ccm_matrix = scale * quad + offset
        // The first update should give initialized values
        assert!(ctrl.ccm_initialized);

        // Get the CCM and check row sums are ≈1 (sanitization normalizes)
        let ccm = ctrl.ccm_matrix;
        for row in 0..3 {
            let base = row * 3;
            let sum: f32 = ccm[base..base + 3].iter().sum();
            assert!((sum - 1.0).abs() < 0.01, "Row {} sum = {}", row, sum);
        }
    }

    #[test]
    fn test_zone_stats_initialization() {
        let mut ctrl = IspController::new();
        assert!(!ctrl.zone_stats_enabled);
        ctrl.init_zone_stats(6, 8);
        assert!(ctrl.zone_stats_enabled);
        assert_eq!(ctrl.zone_rows, 6);
        assert_eq!(ctrl.zone_cols, 8);
        assert_eq!(ctrl.zone_weight.len(), 48);
        // Weights should sum to ~1.0
        let sum: f32 = ctrl.zone_weight.iter().sum();
        assert!((sum - 1.0).abs() < 0.01, "Zone weights sum to {}", sum);
        // Center zones should have higher weight
        let corner = ctrl.zone_weight[0];
        let center = ctrl.zone_weight[3 * 8 + 3]; // row 3, col 3
        assert!(
            center > corner,
            "Center weight {} should > corner {}",
            center,
            corner
        );
    }

    #[test]
    fn test_zone_stats_update() {
        let mut ctrl = IspController::new();
        ctrl.init_zone_stats(6, 8);

        // Create zone stats with red-dominant zones (R high, B low)
        // The CCT formula: 4400 - 1300*b/g + 2100*r/g
        // For R=0.5, G=0.3, B=0.1: 4400 - 1300*0.33 + 2100*1.67 = 7478 → cool cluster
        let mut zone_stats = vec![0.0f32; 6 * 8 * 3];
        for i in 0..(6 * 8) {
            zone_stats[i * 3] = 0.5; // R
            zone_stats[i * 3 + 1] = 0.3; // G
            zone_stats[i * 3 + 2] = 0.1; // B
        }

        ctrl.update_zone_stats(&zone_stats);
        assert!(ctrl.dominant_cct_cluster.is_some());
        // R-dominant → CCT formula gives high CCT → cool cluster (2)
        assert_eq!(
            ctrl.dominant_cct_cluster,
            Some(2),
            "R-dominant should be cool cluster: {:?}",
            ctrl.dominant_cct_cluster
        );
    }

    #[test]
    fn test_zone_stats_multi_illuminant() {
        let mut ctrl = IspController::new();
        ctrl.init_zone_stats(4, 4);

        // Half warm (B dominant → low CCT), half cool (R dominant → high CCT)
        let mut zone_stats = vec![0.0f32; 4 * 4 * 3];
        for i in 0..(4 * 4) {
            if i < 8 {
                // Warm in photo sense: more blue, less red
                zone_stats[i * 3] = 0.1;
                zone_stats[i * 3 + 1] = 0.3;
                zone_stats[i * 3 + 2] = 0.5;
            } else {
                // Cool in photo sense: more red, less blue
                zone_stats[i * 3] = 0.5;
                zone_stats[i * 3 + 1] = 0.3;
                zone_stats[i * 3 + 2] = 0.1;
            }
        }

        ctrl.update_zone_stats(&zone_stats);
        // Should detect mixed illumination
        // Warm cluster (B high): CCT < 4000 → cluster 0
        // Cool cluster (R high): CCT > 5500 → cluster 2
        assert_eq!(
            ctrl.dominant_cct_cluster,
            Some(-1),
            "Mixed scene should be -1: {:?}",
            ctrl.dominant_cct_cluster
        );
    }

    #[test]
    fn test_scene_classification_in_controller() {
        let mut ctrl = IspController::new();

        // Dark scene: low luminance → should be Dark
        ctrl.scene_luminance = 0.02;
        ctrl.update_channel_stats(&[0.02, 0.018, 0.015]);
        assert_eq!(ctrl.scene_category, SceneCategory::Dark);

        // Bright scene
        let mut ctrl = IspController::new();
        ctrl.frame_count = 5;
        ctrl.scene_luminance = 0.7;
        ctrl.update_channel_stats(&[0.7, 0.65, 0.6]);
        assert_eq!(ctrl.scene_category, SceneCategory::Bright);

        // Moderate luminance should be Outdoor
        let mut ctrl = IspController::new();
        ctrl.frame_count = 30;
        ctrl.scene_luminance = 0.35;
        ctrl.update_channel_stats(&[0.35, 0.33, 0.30]);
        assert_eq!(ctrl.scene_category, SceneCategory::Outdoor);

        // Indoor luminance
        let mut ctrl = IspController::new();
        ctrl.frame_count = 30;
        ctrl.scene_luminance = 0.1;
        // Neutral CCT (not warm) → Indoor, not Sunset
        ctrl.update_channel_stats(&[0.10, 0.10, 0.10]);
        let cct = ctrl.estimated_cct.unwrap_or(5500);
        if cct > 4000 || cct == 0 {
            assert_eq!(
                ctrl.scene_category,
                SceneCategory::Indoor,
                "CCT={} should be Indoor",
                cct
            );
        }
    }
}
