//! Exposure and histogram-based parameter estimation.
//!
//! Computes exposure time, ISO, brightness, and lens shading correction
//! from per-frame histogram and luminance statistics.
//!
//! Extracted from `controller.rs`.

use crate::controller::IspController;

impl IspController {
    // ── Histogram update ──

    /// Update histogram-based exposure control.
    ///
    /// `hist` — 256-bin luminance histogram (normalized, sum = 1.0).
    pub fn update_histogram(&mut self, hist: &[f32]) {
        let n_bins = hist.len();
        if n_bins < 4 {
            return;
        }

        let hl_ratio = (hist[n_bins - 1] + hist[n_bins - 2]).clamp(0.0, 1.0);
        let sh_ratio = (hist[0] + hist[1]).clamp(0.0, 1.0);

        let ha = 0.3;
        self.highlight_ratio += (hl_ratio - self.highlight_ratio) * ha;
        self.shadow_ratio += (sh_ratio - self.shadow_ratio) * ha;

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

    /// Get effective exposure gain (histogram-constrained × AE gain).
    pub fn get_effective_exposure_gain(&self) -> f32 {
        self.exposure_gain * self.hist_constrained_gain
    }

    /// Get LSC (lens shading correction) strength.
    pub fn get_lsc_strength(&self) -> f32 {
        self.lsc_strength
    }

    /// Compute exposure time + ISO from current scene stats.
    pub fn compute_exposure(&mut self, brightness_bias: f32) -> (i64, i32) {
        let lum = if self.avg_lum_mean > 0.001 {
            self.avg_lum_mean
        } else {
            self.scene_luminance
        };
        let (exp_ns, iso) = self.ae_state.compute(
            lum,
            self.highlight_ratio,
            self.shadow_ratio,
            brightness_bias,
        );
        self.exposure_time_ns = exp_ns;
        self.analog_gain = iso as f32 / self.ae_state.base_iso as f32;
        (exp_ns, iso)
    }

    /// Get current effective brightness as normalized 0..1.
    pub fn get_brightness(&self) -> f32 {
        let ev_range = 4.0;
        let ev_offset = (self.exposure_gain.max(0.25) as f64).ln() / (2.0f64).ln();
        (0.5 + ev_offset / ev_range).clamp(0.0, 1.0) as f32
    }

    /// Set brightness (normalized 0..1, 0.5 = default).
    pub fn set_brightness(&mut self, value: f32, auto_tune: bool) {
        let v = value.clamp(0.0, 1.0);
        self.brightness_bias = v;
        self.last_applied_brightness = v;
        if auto_tune {
            return;
        }
        let ev_range = 4.0;
        let ev_offset = (v - 0.5) * ev_range;
        let gain = (ev_offset as f64 * std::f64::consts::LN_2).exp() as f32;
        self.exposure_gain = gain.clamp(0.25, 8.0);
        let tone_brightness_range = 0.3;
        let tone_offset = (v - 0.5) * tone_brightness_range;
        self.tone_brightness =
            tone_offset.clamp(-tone_brightness_range / 2.0, tone_brightness_range / 2.0);
    }

    pub fn set_manual_exposure_gain(&mut self, gain: f32) {
        self.exposure_gain = gain.clamp(0.5, 8.0);
    }
}
