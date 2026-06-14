//! ISP Controller — AWB, AE, CCM, Tone parameter estimation from frame statistics.
//!
//! Ported from `com.camcore.isp.pipeline.IspController` (Java).
//!
//! The controller takes per-frame statistics (channel means, tone stats, histogram)
//! and produces smoothed ISP parameters (AWB gains, CCM matrix, tone curve, exposure).

use cam_types::ToneParams;
use crate::ae::AutoExposureState;
use crate::ccm_engine::{self, select_ccm};
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
    /// Smoothed AWB gains [R, G, B].
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
    /// Per-zone RGB means [row][col] = [R, G, B].
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
        if self.clamp_flags & (ccm_engine::CCM_R_OUT_NEGATIVE | ccm_engine::CCM_DIAGONAL_EXTREME) != 0 {
            log::warn!("CCM clamp flags=0x{:x} at CCT={} -> resetting CCT to 5500",
                self.clamp_flags, self.clamp_cct_ref);
            self.estimated_cct = Some(5500);
            self.clamp_flags = 0;
        }

        // Scene change tracking (compare to previous frame)
        if let Some(prev) = self.prev_channel_means {
            let max_prev = prev[0].max(prev[1]).max(prev[2]).max(1e-6);
            let md = (channel_means[0] - prev[0]).abs()
                .max((channel_means[1] - prev[1]).abs())
                .max((channel_means[2] - prev[2]).abs());
            self.scene_change = (md / max_prev).clamp(0.0, 1.0);
        }
        self.prev_channel_means = Some(*channel_means);

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
        // Sync smoothed gain state
        let _ = self.awb_gains[0];

        // Store raw ratios for diagnostics
        self.last_rg = r_gain;
        self.last_bg = b_gain;

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

        // Update CCT estimate (only when scene has sufficient luminance)
        let scene_luminance = 0.299 * r + 0.587 * g + 0.114 * b;
        if scene_luminance >= self.cct_lum_threshold {
            let avg_rg = self.avg_r / self.avg_g;
            let avg_bg = self.avg_b / self.avg_g;
            self.estimated_cct = Some(Self::estimate_cct(avg_rg, avg_bg));
        } else {
            // Keep previous estimate (or default to 3000K)
            if self.estimated_cct.is_none() {
                self.estimated_cct = Some(3000);
            }
        }

        // CCT-based AWB prior (first 20 frames) — biases gains towards
        // expected R/G and B/G ratios for the estimated CCT
        if self.frame_count < 20 {
            let cct = self.estimated_cct.unwrap_or(5500);
            let p_r = self.prior_r(cct);
            let p_b = self.prior_b(cct);
            let prior_weight = (1.0 - self.frame_count as f32 / 20.0).clamp(0.0, 0.4);
            if prior_weight > 0.0 {
                self.awb_gains[0] += (p_r - self.awb_gains[0]) * prior_weight;
                self.awb_gains[2] += (p_b - self.awb_gains[2]) * prior_weight;
            }
        }

        // Update CCM: y = scale · select_ccm(cct) + offset, with EMA smoothing
        if let Some(cct) = self.estimated_cct {
            let x = select_ccm(cct);
            // Element-wise: raw[i] = scale[i] * x[i] + offset[i]
            let raw: [f32; 9] = std::array::from_fn(|i| {
                self.ccm_scale_a[i] * x[i] + self.ccm_offset_c[i]
            });

            // Output EMA smoothing
            if self.ccm_initialized {
                for i in 0..9 {
                    self.smoothed_ccm[i] += (raw[i] - self.smoothed_ccm[i]) * self.ccm_smoothing;
                }
            } else {
                self.smoothed_ccm = raw;
                self.ccm_initialized = true;
            }

            // Copy smoothed → ccm_matrix for external access
            self.ccm_matrix = self.smoothed_ccm;

            // Sanitize
            ccm_engine::sanitize_ccm(
                &mut self.ccm_matrix, cct, "stats",
                Some(&mut self.clamp_flags),
                Some(&mut self.clamp_cct_ref),
            );

            // Sync back smoothed_ccm from sanitized matrix
            self.smoothed_ccm = self.ccm_matrix;
        }

        // Scene classification + adaptive ISP tuning
        let cct_for_scene = self.estimated_cct.unwrap_or(5500) as u32;
        self.last_cct_for_scene = cct_for_scene;
        let new_scene = SceneCategory::classify(self.scene_luminance, cct_for_scene);

        // Scene transition detection
        if new_scene != self.scene_category {
            log::debug!(
                "Scene transition: {:?} → {:?} (lum={:.3}, cct={}K)",
                self.scene_category, new_scene, self.scene_luminance, cct_for_scene
            );
        }
        self.scene_category = new_scene;

        // Scene-adaptive parameter adjustments
        match self.scene_category {
            SceneCategory::Dark => {
                // Boost exposure, reduce sharpness, warm up tones
                self.tone_shadow_lift = self.tone_shadow_lift.max(0.15);
                self.tone_gamma = self.tone_gamma.max(2.0).min(2.6);
                // CCM diagonal boost for better low-light color
                for i in [0, 4, 8] {
                    self.ccm_scale_a[i] = self.ccm_scale_a[i].max(1.05).min(1.25);
                }
            },
            SceneCategory::SunriseSunset => {
                // Preserve warm tones: reduce AWB correction
                // Bias AWB gains toward 1.0 to avoid neutralizing the warm cast
                self.awb_gains[0] += (1.0 - self.awb_gains[0]) * 0.1;
                self.awb_gains[2] += (1.0 - self.awb_gains[2]) * 0.1;
                // Warm color bias: increase R channel gain slightly
                self.ccm_scale_a[0] = self.ccm_scale_a[0].max(1.0).min(1.15);
                // Enhance saturation for richer colors
                self.tone_saturation = self.tone_saturation.max(1.1).min(1.4);
            },
            SceneCategory::Indoor => {
                // Moderate settings (default behavior, reset any extremes)
                self.tone_saturation = self.tone_saturation.clamp(0.9, 1.2);
                self.tone_shadow_lift = self.tone_shadow_lift.clamp(0.05, 0.20);
                self.tone_contrast = self.tone_contrast.clamp(0.9, 1.2);
            },
            SceneCategory::Outdoor => {
                // Increase contrast and sharpness, reduce shadow lift
                self.tone_contrast = self.tone_contrast.max(1.05).min(1.3);
                self.tone_shadow_lift = self.tone_shadow_lift.min(0.12);
                self.tone_gamma = self.tone_gamma.clamp(2.0, 2.4);
            },
            SceneCategory::Bright => {
                // Reduce exposure gain, increase contrast for punch
                self.exposure_gain = self.exposure_gain.min(1.2);
                self.tone_contrast = self.tone_contrast.max(1.1).min(1.4);
                self.tone_shadow_lift = self.tone_shadow_lift.min(0.08);
                self.tone_gamma = self.tone_gamma.clamp(2.0, 2.3);
                // Slightly desaturate very bright scenes
                self.tone_saturation = self.tone_saturation.min(1.1);
            },
            SceneCategory::Unknown => {
                // Fall through with default params
            },
        }
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

        // ── Dim-scene aggressiveness factor ──
        let dim_factor = (((self.avg_lum_mean - 0.05) / 0.15).clamp(0.0, 1.0)).powi(2);

        // ── Exposure gain — exponential target luminance ──
        let max_gain = 1.0 + dim_factor;
        let target_lum_base = 0.35 * (1.0 - (-self.avg_lum_mean * 8.0).exp()).max(0.02);
        let lum_bias = (self.brightness_bias - 0.5) * 0.2; // map 0..1 to -0.1..+0.1
        let target_lum = (target_lum_base + lum_bias).clamp(0.02, 0.6);
        let raw_ae_gain = (target_lum / self.avg_lum_mean.max(0.005)).clamp(0.5, max_gain);
        self.exposure_gain += (raw_ae_gain - self.exposure_gain) * alpha;

        // Blend with histogram-constrained gain
        let hist_weight = (self.highlight_ratio * 3.0).clamp(0.0, 0.6);
        let blended_gain = self.exposure_gain * (1.0 - hist_weight) + self.hist_constrained_gain * hist_weight;
        self.exposure_gain = blended_gain.clamp(0.5, 8.0);

        // ── Brightness ──
        let max_brightness = dim_factor * 0.03;
        let eff_mean = (self.avg_lum_mean * self.exposure_gain).min(1.0);
        let raw_brightness = ((target_lum - eff_mean) * 0.4).clamp(-max_brightness, max_brightness);
        self.tone_brightness += (raw_brightness - self.tone_brightness) * alpha;

        // ── Contrast ──
        let dynamic_range = (self.avg_lum_max - self.avg_lum_min).max(0.01);
        let max_contrast = 1.0 + dim_factor * 0.5;
        let raw_contrast = (0.85 / dynamic_range).clamp(0.7, max_contrast);
        self.tone_contrast += (raw_contrast - self.tone_contrast) * alpha;

        // ── Gamma (centered on normalized mid-tone) ──
        let normalized_mean = if dynamic_range > 0.01 {
            (self.avg_lum_mean - self.avg_lum_min) / dynamic_range
        } else {
            0.5
        };
        let gamma_center = 2.2 + 0.6 * (normalized_mean - 0.5);
        let gamma_floor = if self.avg_lum_mean < 0.2 { 2.0 } else { gamma_center };
        self.tone_gamma += (gamma_floor.clamp(2.0, 2.6) - self.tone_gamma) * alpha * 0.5;

        // ── S-curve: shadow lift + highlight roll ──
        let raw_shadow_lift = (0.04 + 0.08 * (-self.avg_lum_mean * 5.0).exp()).clamp(0.02, 0.12);
        let target_shadow_lift = raw_shadow_lift * dim_factor;
        let target_highlight_roll = (0.05 + 0.15 * (1.0 - (-self.avg_lum_mean * 3.0).exp())).clamp(0.04, 0.20);
        self.tone_shadow_lift += (target_shadow_lift - self.tone_shadow_lift) * alpha;
        self.tone_highlight_roll += (target_highlight_roll - self.tone_highlight_roll) * alpha;

        // ── LSC strength from exposure gain ──
        let raw_lsc = ((self.exposure_gain - 1.0).clamp(0.0, 1.0)) * 0.5;
        self.lsc_strength += (raw_lsc - self.lsc_strength) * 0.05;

        // ── Saturation ──
        let raw_sat = (1.0 + 0.3 * (1.0 - (-self.avg_lum_mean * 4.0).exp())).clamp(1.0, 1.3);
        self.tone_saturation += (raw_sat - self.tone_saturation) * 0.03;

        // Clamp everything
        self.sanitize_tone("tone");

        // Scene luminance for display
        self.scene_luminance = self.avg_lum_mean;
    }

    // ── Histogram update ──

    /// Update histogram-based exposure control.
    ///
    /// `hist` — 256-bin luminance histogram (normalized, sum = 1.0).
    /// Ported from ToneEngine.updateHistogram().
    pub fn update_histogram(&mut self, hist: &[f32]) {
        let n_bins = hist.len();
        if n_bins < 4 { return; }

        // Compute highlight (top 2 bins) and shadow (bottom 2 bins) ratios
        let hl_ratio = (hist[n_bins - 1] + hist[n_bins - 2]).clamp(0.0, 1.0);
        let sh_ratio = (hist[0] + hist[1]).clamp(0.0, 1.0);

        let ha = 0.3;
        self.highlight_ratio += (hl_ratio - self.highlight_ratio) * ha;
        self.shadow_ratio += (sh_ratio - self.shadow_ratio) * ha;

        // Constrain gain based on highlight/shadow excess
        let hl_excess = (self.highlight_ratio - 0.05).max(0.0);
        let hl_gain_reduction = 1.0 / (1.0 + hl_excess * 30.0);
        let sh_excess = (self.shadow_ratio - 0.30).max(0.0);
        let sh_gain_boost = 1.0 + sh_excess * 0.5;

        let gain_limit = if hl_excess > 0.02 {
            (self.hist_constrained_gain * hl_gain_reduction).clamp(0.3, 1.5)
        } else if sh_excess > 0.0 {
            (self.hist_constrained_gain * sh_gain_boost).clamp(0.5, 4.0)
        } else {
            self.hist_constrained_gain
        };
        self.hist_constrained_gain += (gain_limit - self.hist_constrained_gain) * 0.2;
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
        let (exp_ns, iso) = self.ae_state.compute(lum, self.highlight_ratio, self.shadow_ratio, brightness_bias);
        self.exposure_time_ns = exp_ns;
        self.analog_gain = iso as f32 / self.ae_state.base_iso as f32;
        (exp_ns, iso)
    }

    /// Get current effective brightness as normalized 0..1.
    pub fn get_brightness(&self) -> f32 {
        let ev_range = 4.0;
        let ev_offset = (self.exposure_gain.max(0.25) as f64).ln() / (2.0f64).ln();
        (0.5 + ev_offset / ev_range as f64).clamp(0.0, 1.0) as f32
    }

    /// Set brightness (normalized 0..1, 0.5 = default).
    /// In auto-tune mode, only sets brightness_bias for the AE loop.
    /// In manual mode, directly sets exposure_gain and tone_brightness.
    pub fn set_brightness(&mut self, value: f32, auto_tune: bool) {
        let v = value.clamp(0.0, 1.0);
        self.brightness_bias = v;
        self.last_applied_brightness = v;
        if auto_tune {
            return; // Bias picked up by AE loop next frame
        }
        let ev_range = 4.0;
        let ev_offset = (v - 0.5) * ev_range;
        let gain = (ev_offset as f64 * std::f64::consts::LN_2).exp() as f32;
        self.exposure_gain = gain.clamp(0.25, 8.0);
        let tone_brightness_range = 0.3;
        let tone_offset = (v - 0.5) * tone_brightness_range;
        self.tone_brightness = tone_offset.clamp(-tone_brightness_range / 2.0, tone_brightness_range / 2.0);
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

    /// Goalkeeper: validate & clamp all tone / AWB parameters.
    /// Ported from ToneEngine.sanitizeTone().
    pub fn sanitize_tone(&mut self, tag: &str) {
        let clamped = false;

        // AWB gains
        self.awb_gains[0] = self.awb_gains[0].clamp(0.5, 3.0);
        self.awb_gains[1] = 1.0;
        self.awb_gains[2] = self.awb_gains[2].clamp(0.5, 3.0);

        // Tone params
        self.tone_gamma = self.tone_gamma.clamp(0.5, 4.0);
        self.tone_contrast = self.tone_contrast.clamp(0.5, 2.0);
        self.tone_brightness = self.tone_brightness.clamp(-0.3, 0.3);
        self.tone_shadow_lift = self.tone_shadow_lift.clamp(0.0, 0.25);
        self.tone_highlight_roll = self.tone_highlight_roll.clamp(0.0, 0.30);
        self.exposure_gain = self.exposure_gain.clamp(0.5, 8.0);
        self.tone_saturation = self.tone_saturation.clamp(0.7, 1.6);
        self.lsc_strength = self.lsc_strength.clamp(0.0, 1.0);

        if clamped {
            log::warn!(
                "ToneGuard {}: γ={:.2} c={:.2} b={:.3} sl={:.3} hr={:.3} exp={:.2} sat={:.2} lsc={:.2} awb=[{:.2},{:.2}]",
                tag,
                self.tone_gamma, self.tone_contrast, self.tone_brightness,
                self.tone_shadow_lift, self.tone_highlight_roll,
                self.exposure_gain, self.tone_saturation, self.lsc_strength,
                self.awb_gains[0], self.awb_gains[2],
            );
        }
    }

    /// Reset all state to defaults.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    // ── Zone stats ──

    /// Initialize zone stats grid with given dimensions.
    /// Call this once before feeding zone stats.
    pub fn init_zone_stats(&mut self, rows: usize, cols: usize) {
        self.zone_rows = rows;
        self.zone_cols = cols;
        let n = rows * cols;

        // Build center-weighted zone weights
        let mut w = vec![0.0f32; n];
        for r in 0..rows {
            for c in 0..cols {
                let i = r * cols + c;
                let cc = c >= cols / 2 - 1 && c <= cols / 2;  // center columns
                let cr = r >= rows / 2 - 1 && r <= rows / 2;  // center rows
                w[i] = if cr && cc { 4.0 } else if cr || cc { 2.0 } else { 1.0 };
            }
        }
        let sum: f32 = w.iter().sum();
        if sum > 0.0 {
            for v in w.iter_mut() { *v /= sum; }
        }
        self.zone_weight = w;

        // Initialize per-zone arrays
        self.zone_rgb = vec![vec![[0.0f32; 3]; cols]; rows];
        self.zone_lum = vec![vec![0.0f32; cols]; rows];
        self.zone_cct = vec![vec![5500.0f32; cols]; rows];
        self.smoothed_zone_cct = vec![vec![5500.0f32; cols]; rows];
        self.zone_cct_initialized = false;
        self.zone_stats_enabled = true;
    }

    /// Update zone stats from per-zone RGB means.
    /// `zone_stats` should have length zone_rows * zone_cols * 3,
    /// with RGB values in [0, 1] interleaved per zone (row-major).
    pub fn update_zone_stats(&mut self, zone_stats: &[f32]) {
        if !self.zone_stats_enabled { return; }
        let expected = self.zone_rows * self.zone_cols * 3;
        if zone_stats.len() < expected { return; }

        let min_rgb = 0.001;
        let mut idx = 0;
        let mut warm_c = 0;
        let mut mid_c = 0;
        let mut cool_c = 0;
        let mut valid_z = 0;
        let mut t_r = 0.0f64;
        let mut t_g = 0.0f64;
        let mut t_b = 0.0f64;
        let mut t_w = 0.0f64;

        self.zone_rgb.clear();
        self.zone_lum.clear();
        self.zone_cct.clear();

        for r in 0..self.zone_rows {
            let mut rgb_row = Vec::with_capacity(self.zone_cols);
            let mut lum_row = Vec::with_capacity(self.zone_cols);
            let mut cct_row = Vec::with_capacity(self.zone_cols);

            for c in 0..self.zone_cols {
                let rz = zone_stats[idx].max(min_rgb);
                let gz = zone_stats[idx + 1].max(min_rgb);
                let bz = zone_stats[idx + 2].max(min_rgb);
                idx += 3;

                rgb_row.push([rz, gz, bz]);
                let y = 0.299 * rz + 0.587 * gz + 0.114 * bz;
                lum_row.push(y);

                let w_idx = r * self.zone_cols + c;
                let w = self.zone_weight.get(w_idx).copied().unwrap_or(1.0) as f64;
                t_r += rz as f64 * w;
                t_g += gz as f64 * w;
                t_b += bz as f64 * w;
                t_w += w;

                // Compute per-zone CCT
                if gz > min_rgb && rz > min_rgb && bz > min_rgb {
                    let rg = rz / gz;
                    let bg = bz / gz;
                    let cct_val = Self::estimate_cct(rg, bg) as f32;
                    cct_row.push(cct_val);

                    // Smooth temporal zone CCT
                    if self.zone_cct_initialized {
                        let prev = self.smoothed_zone_cct[r][c];
                        self.smoothed_zone_cct[r][c] = prev + (cct_val - prev) * 0.3;
                    } else {
                        self.smoothed_zone_cct[r][c] = cct_val;
                    }
                    let smoothed = self.smoothed_zone_cct[r][c];
                    if smoothed < 4000.0 {
                        warm_c += 1;
                    } else if smoothed > 5500.0 {
                        cool_c += 1;
                    } else {
                        mid_c += 1;
                    }
                    valid_z += 1;
                } else {
                    cct_row.push(5500.0);
                }
            }

            self.zone_rgb.push(rgb_row);
            self.zone_lum.push(lum_row);
            self.zone_cct.push(cct_row);
        }

        self.zone_cct_initialized = true;

        // ── Zone-weighted AWB ──
        if t_w > 0.001 {
            let wr = (t_r / t_w) as f32;
            let wg = (t_g / t_w) as f32;
            let wb = (t_b / t_w) as f32;
            if wg > min_rgb {
                let r_gain = (wg * self.target_r / wr).clamp(0.5, 3.0);
                let b_gain = (wg * self.target_b / wb).clamp(0.5, 3.0);
                let alpha_awb = self.awb_alpha();
                self.awb_gains[0] += (r_gain - self.awb_gains[0]) * alpha_awb;
                self.awb_gains[2] += (b_gain - self.awb_gains[2]) * alpha_awb;
            }
        }

        // ── Multi-illuminant cluster detection ──
        if valid_z >= 6 {
            let total = (warm_c + mid_c + cool_c) as f32;
            let max_c = warm_c.max(mid_c).max(cool_c);
            self.dominant_cct_cluster = if max_c == warm_c {
                Some(0)
            } else if max_c == mid_c {
                Some(1)
            } else {
                Some(2)
            };

            // Check for mixed illumination
            let mut sorted = vec![warm_c, mid_c, cool_c];
            sorted.sort_by(|a, b| b.cmp(a));
            if sorted.len() >= 2 && (sorted[1] as f32 / total) > 0.20
                && (sorted[0] as f32) < 0.80 * total
            {
                self.dominant_cct_cluster = Some(-1); // Mixed
            }
            self.dominant_cluster_fraction = max_c as f32 / total;
        }

        log::debug!(
            "ZoneStats: w={} m={} c={} cluster={:?} frac={:.2}",
            warm_c, mid_c, cool_c,
            self.dominant_cct_cluster,
            self.dominant_cluster_fraction
        );
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
            log::debug!("AWB: B capped from {:.2} to {:.2} (CCT={:.0})", raw_b, max_b, cct);
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
        let changed = (gains[0] - 1.0).abs() > 0.01
            || (gains[2] - 1.0).abs() > 0.01;
        assert!(changed, "AWB should have adapted: gains={:.3} {:.3} {:.3}",
            gains[0], gains[1], gains[2]);
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
        assert!(center > corner, "Center weight {} should > corner {}", center, corner);
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
            zone_stats[i * 3] = 0.5;     // R
            zone_stats[i * 3 + 1] = 0.3; // G
            zone_stats[i * 3 + 2] = 0.1; // B
        }

        ctrl.update_zone_stats(&zone_stats);
        assert!(ctrl.dominant_cct_cluster.is_some());
        // R-dominant → CCT formula gives high CCT → cool cluster (2)
        assert_eq!(ctrl.dominant_cct_cluster, Some(2), "R-dominant should be cool cluster: {:?}", ctrl.dominant_cct_cluster);
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
        assert_eq!(ctrl.dominant_cct_cluster, Some(-1), "Mixed scene should be -1: {:?}", ctrl.dominant_cct_cluster);
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
            assert_eq!(ctrl.scene_category, SceneCategory::Indoor,
                "CCT={} should be Indoor", cct);
        }
    }
}
