//! Tone parameter estimation from frame statistics.
//!
//! Computes tone curve parameters (gamma, contrast, brightness, shadow lift,
//! highlight roll, saturation) from per-frame luminance statistics.
//!
//! Extracted from `controller.rs`.

use crate::controller::IspController;

impl IspController {
    // ── Smoothing ──

    pub(crate) fn tone_alpha(&self) -> f32 {
        if self.frame_count < 10 {
            0.5
        } else {
            self.smoothing_alpha
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
        let lum_bias = (self.brightness_bias - 0.5) * 0.2;
        let target_lum = (target_lum_base + lum_bias).clamp(0.02, 0.6);
        let raw_ae_gain = (target_lum / self.avg_lum_mean.max(0.005)).clamp(0.5, max_gain);
        self.exposure_gain += (raw_ae_gain - self.exposure_gain) * alpha;

        // Blend with histogram-constrained gain
        let hist_weight = (self.highlight_ratio * 3.0).clamp(0.0, 0.6);
        let blended_gain =
            self.exposure_gain * (1.0 - hist_weight) + self.hist_constrained_gain * hist_weight;
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
        let gamma_floor = if self.avg_lum_mean < 0.2 {
            2.0
        } else {
            gamma_center
        };
        self.tone_gamma += (gamma_floor.clamp(2.0, 2.6) - self.tone_gamma) * alpha * 0.5;

        // ── S-curve: shadow lift + highlight roll ──
        let raw_shadow_lift = (0.04 + 0.08 * (-self.avg_lum_mean * 5.0).exp()).clamp(0.02, 0.12);
        let target_shadow_lift = raw_shadow_lift * dim_factor;
        let target_highlight_roll =
            (0.05 + 0.15 * (1.0 - (-self.avg_lum_mean * 3.0).exp())).clamp(0.04, 0.20);
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

    /// Get tone parameters for the ISP pipeline.
    pub fn get_tone_params(&self) -> cam_types::ToneParams {
        cam_types::ToneParams {
            contrast: self.tone_contrast,
            brightness: self.tone_brightness,
            gamma_recip: 1.0 / self.tone_gamma,
            shadow_lift: self.tone_shadow_lift,
            highlight_roll: self.tone_highlight_roll,
            sharpness: 1.0,
            saturation: self.tone_saturation,
        }
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

    pub fn set_manual_gamma(&mut self, gamma: f32) {
        self.tone_gamma = gamma.clamp(0.5, 4.0);
    }

    pub fn set_manual_contrast(&mut self, contrast: f32) {
        self.tone_contrast = contrast.clamp(0.5, 2.0);
    }
}
