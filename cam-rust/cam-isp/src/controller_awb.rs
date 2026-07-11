//! AWB/CCT parameter estimation from frame statistics.
//!
//! Auto-white-balance and color-correction-temperature estimation from
//! per-frame channel means. Computes AWB gains, CCM matrix, and CCT.
//!
//! Extracted from `controller.rs` to separate AWB/CCT logic from the
//! rest of the ISP controller.

use crate::controller::IspController;
use crate::scene::SceneCategory;
use crate::ccm_engine::{self, select_ccm};

impl IspController {
    // ── Smoothing ──

    /// Smoothing alpha for AWB (fast convergence in first 10 frames).
    pub(crate) fn awb_alpha(&self) -> f32 {
        if self.frame_count < 10 { 0.4 } else { self.smoothing_alpha }
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

        // Scene change tracking
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

        // Update CCT estimate
        let scene_luminance = 0.299 * r + 0.587 * g + 0.114 * b;
        if scene_luminance >= self.cct_lum_threshold {
            let avg_rg = self.avg_r / self.avg_g;
            let avg_bg = self.avg_b / self.avg_g;
            self.estimated_cct = Some(Self::estimate_cct(avg_rg, avg_bg));
        } else {
            if self.estimated_cct.is_none() {
                self.estimated_cct = Some(3000);
            }
        }

        // CCT-based AWB prior (first 20 frames)
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

        // Update CCM
        if let Some(cct) = self.estimated_cct {
            let x = select_ccm(cct);
            let raw: [f32; 9] = std::array::from_fn(|i| {
                self.ccm_scale_a[i] * x[i] + self.ccm_offset_c[i]
            });

            if self.ccm_initialized {
                for i in 0..9 {
                    self.smoothed_ccm[i] += (raw[i] - self.smoothed_ccm[i]) * self.ccm_smoothing;
                }
            } else {
                self.smoothed_ccm = raw;
                self.ccm_initialized = true;
            }

            self.ccm_matrix = self.smoothed_ccm;

            ccm_engine::sanitize_ccm(
                &mut self.ccm_matrix, cct, "stats",
                Some(&mut self.clamp_flags),
                Some(&mut self.clamp_cct_ref),
            );

            self.smoothed_ccm = self.ccm_matrix;
        }

        // Scene classification
        let cct_for_scene = self.estimated_cct.unwrap_or(5500) as u32;
        self.last_cct_for_scene = cct_for_scene;
        let new_scene = SceneCategory::classify(self.scene_luminance, cct_for_scene);

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
                self.tone_shadow_lift = self.tone_shadow_lift.max(0.15);
                self.tone_gamma = self.tone_gamma.clamp(2.0, 2.6);
                for i in [0, 4, 8] {
                    self.ccm_scale_a[i] = self.ccm_scale_a[i].clamp(1.05, 1.25);
                }
            },
            SceneCategory::SunriseSunset => {
                self.awb_gains[0] += (1.0 - self.awb_gains[0]) * 0.1;
                self.awb_gains[2] += (1.0 - self.awb_gains[2]) * 0.1;
                self.ccm_scale_a[0] = self.ccm_scale_a[0].clamp(1.0, 1.15);
                self.tone_saturation = self.tone_saturation.clamp(1.1, 1.4);
            },
            SceneCategory::Indoor => {
                self.tone_saturation = self.tone_saturation.clamp(0.9, 1.2);
                self.tone_shadow_lift = self.tone_shadow_lift.clamp(0.05, 0.20);
                self.tone_contrast = self.tone_contrast.clamp(0.9, 1.2);
            },
            SceneCategory::Outdoor => {
                self.tone_contrast = self.tone_contrast.clamp(1.05, 1.3);
                self.tone_shadow_lift = self.tone_shadow_lift.min(0.12);
                self.tone_gamma = self.tone_gamma.clamp(2.0, 2.4);
            },
            SceneCategory::Bright => {
                self.exposure_gain = self.exposure_gain.min(1.2);
                self.tone_contrast = self.tone_contrast.clamp(1.1, 1.4);
                self.tone_shadow_lift = self.tone_shadow_lift.min(0.08);
                self.tone_gamma = self.tone_gamma.clamp(2.0, 2.3);
                self.tone_saturation = self.tone_saturation.min(1.1);
            },
            SceneCategory::Unknown => {}
        }
    }

    // ── Getters ──

    /// Get smoothed AWB gains as `[R, G, B]`.
    pub fn get_awb_gains(&self) -> [f32; 3] {
        self.awb_gains
    }

    /// Get current CCM matrix (3×3 row-major).
    pub fn get_ccm(&self) -> [f32; 9] {
        self.smoothed_ccm
    }
}
