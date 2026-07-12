//! Pipeline configuration — editable snapshot of ISP feature flags.
//!
//! Ported from `com.camcore.isp.pipeline.PipelineConfig` (Java).
//! Represents a complete, immutable pipeline configuration that can be
//! edited via builder methods and applied to the pipeline manager.

use crate::profile::{DemosaicQuality, PipelineProfile};

/// Complete pipeline configuration — which blocks to include.
///
/// Create from a profile with `PipelineConfig::from_profile()`, then
/// override individual flags with builder methods.
#[derive(Debug, Clone)]
pub struct PipelineConfig {
    /// Base profile.
    pub profile: PipelineProfile,
    /// Enable packed INT32 input with UnpackBlock.
    pub use_unpack: bool,
    /// Enable false color suppression (FCS).
    pub use_fcs: bool,
    /// Enable local contrast enhancement (LDCI).
    pub use_ldci: bool,
    /// Enable edge enhancement / unsharp mask (EE).
    pub use_ee: bool,
    /// Enable defective pixel correction.
    pub use_bad_pixel: bool,
    /// Demosaic quality level.
    pub demosaic_quality: DemosaicQuality,
    /// Enable local contrast enhancement.
    pub use_local_contrast: bool,
    /// Enable unsharp mask.
    pub use_unsharp: bool,
    /// Enable lens shading correction.
    pub use_lsc: bool,
    /// Enable geometric warp correction (EIS/deshake).
    pub use_warp: bool,
    /// Enable HDR merge.
    pub use_hdr: bool,
    /// Max pixel dimension for main pipeline (0 = full res).
    /// When >0, inserts AdaptiveDownscaleBlock before debayer.
    /// Toggle this for perf measurement without code mod.
    pub pipeline_downscale_target: u32,
    /// Max pixel dimension for stats blocks (0 = full res).
    /// When >0, inserts AdaptiveDownscaleBlock before stats.
    pub stats_downscale_max: u32,
    /// Human-readable label.
    pub label: String,
}

impl PipelineConfig {
    /// Predefined LITE config.
    pub const LITE: Self = Self {
        profile: PipelineProfile::LITE,
        use_unpack: true,
        use_fcs: false,
        use_ldci: false,
        use_ee: false,
        use_bad_pixel: false,
        demosaic_quality: DemosaicQuality::HqLinear,
        use_local_contrast: false,
        use_unsharp: false,
        use_lsc: false,
        use_warp: false,
        use_hdr: false,
        pipeline_downscale_target: 0,
        stats_downscale_max: 0,
        label: String::new(),
    };

    /// Create a config from a base profile.
    pub fn from_profile(profile: PipelineProfile) -> Self {
        Self {
            profile,
            use_unpack: profile.use_unpack,
            use_fcs: profile.use_fcs,
            use_ldci: profile.use_ldci,
            use_ee: profile.use_ee,
            use_bad_pixel: profile.use_bad_pixel,
            demosaic_quality: profile.demosaic_quality,
            use_local_contrast: profile.use_local_contrast,
            use_unsharp: profile.use_unsharp,
            use_lsc: profile.use_lsc,
            use_warp: profile.use_warp,
            use_hdr: profile.use_hdr,
            pipeline_downscale_target: profile.pipeline_downscale_target,
            stats_downscale_max: profile.stats_downscale_max,
            label: profile.label.to_string(),
        }
    }

    /// Merge this config back into a `PipelineProfile`, overriding fields.
    pub fn to_profile(&self) -> PipelineProfile {
        let mut p = self.profile;
        p.use_unpack = self.use_unpack;
        p.use_fcs = self.use_fcs;
        p.use_ldci = self.use_ldci;
        p.use_ee = self.use_ee;
        p.use_bad_pixel = self.use_bad_pixel;
        p.demosaic_quality = self.demosaic_quality;
        p.use_local_contrast = self.use_local_contrast;
        p.use_unsharp = self.use_unsharp;
        p.use_lsc = self.use_lsc;
        p.use_warp = self.use_warp;
        p.use_hdr = self.use_hdr;
        p.pipeline_downscale_target = self.pipeline_downscale_target;
        p.stats_downscale_max = self.stats_downscale_max;
        p
    }

    /// Whether this config differs from its base profile (custom variant).
    pub fn is_custom(&self) -> bool {
        self.use_unpack != self.profile.use_unpack
            || self.use_fcs != self.profile.use_fcs
            || self.use_ldci != self.profile.use_ldci
            || self.use_ee != self.profile.use_ee
            || self.use_bad_pixel != self.profile.use_bad_pixel
            || self.demosaic_quality != self.profile.demosaic_quality
            || self.use_local_contrast != self.profile.use_local_contrast
            || self.use_unsharp != self.profile.use_unsharp
            || self.use_lsc != self.profile.use_lsc
            || self.use_warp != self.profile.use_warp
            || self.use_hdr != self.profile.use_hdr
            || self.pipeline_downscale_target != self.profile.pipeline_downscale_target
            || self.stats_downscale_max != self.profile.stats_downscale_max
    }

    /// Build a human-readable block chain preview string.
    ///
    /// Example: `"Raw → Norm → CFA → BLC → BayerWB → Debayer → CCM → Tone → Display"`
    pub fn block_chain_preview(&self) -> String {
        let mut b: Vec<&str> = Vec::new();
        b.push("Raw");
        b.push("Norm");
        if self.use_bad_pixel {
            b.push("DPC");
        }
        b.push("CFA");
        b.push("BLC");
        if self.use_lsc {
            b.push("LSC");
        }
        b.push("BayerWB");
        match self.demosaic_quality {
            DemosaicQuality::Standard => b.push("Debayer"),
            DemosaicQuality::HqLinear => b.push("HqLinDemosaic"),
            DemosaicQuality::Edge => b.push("EdgeDemosaic"),
        }
        if self.use_warp {
            b.push("Warp");
        }
        if self.use_hdr {
            b.push("HDR");
        }
        b.push("CCM");
        b.push("Tone");
        if self.use_fcs {
            b.push("FCS");
        }
        if self.use_ldci {
            b.push("LDCI");
        }
        if self.use_ee {
            b.push("EE");
        }
        b.push("Display");
        b.join(" → ")
    }

    /// Number of blocks in this config.
    pub fn block_count(&self) -> usize {
        let mut count: usize = if self.profile.use_fused_unpack && self.profile.use_unpack {
            10
        } else if self.profile.use_unpack {
            12
        } else {
            11
        };
        if self.use_fcs {
            count += 1;
        }
        if self.use_ldci {
            count += 1;
        }
        if self.use_ee {
            count += 1;
        }
        // bad_pixel skipped in fused path
        if self.use_bad_pixel && !(self.profile.use_fused_unpack && self.profile.use_unpack) {
            count += 1;
        }
        if self.use_lsc {
            count += 1;
        }
        if self.use_warp {
            count += 1;
        }
        if self.use_hdr {
            count += 1;
        }
        count
    }

    // ── Builder methods ──

    pub fn with_unpack(mut self, v: bool) -> Self {
        self.use_unpack = v;
        self
    }
    pub fn with_fcs(mut self, v: bool) -> Self {
        self.use_fcs = v;
        self
    }
    pub fn with_ldci(mut self, v: bool) -> Self {
        self.use_ldci = v;
        self
    }
    pub fn with_ee(mut self, v: bool) -> Self {
        self.use_ee = v;
        self
    }
    pub fn with_bad_pixel(mut self, v: bool) -> Self {
        self.use_bad_pixel = v;
        self
    }
    pub fn with_demosaic_quality(mut self, v: DemosaicQuality) -> Self {
        self.demosaic_quality = v;
        self
    }
    pub fn with_local_contrast(mut self, v: bool) -> Self {
        self.use_local_contrast = v;
        self
    }
    pub fn with_unsharp(mut self, v: bool) -> Self {
        self.use_unsharp = v;
        self
    }
    pub fn with_lsc(mut self, v: bool) -> Self {
        self.use_lsc = v;
        self
    }
    pub fn with_warp(mut self, v: bool) -> Self {
        self.use_warp = v;
        self
    }
    pub fn with_hdr(mut self, v: bool) -> Self {
        self.use_hdr = v;
        self
    }
    pub fn with_pipeline_downscale(mut self, v: u32) -> Self {
        self.pipeline_downscale_target = v;
        self
    }
    pub fn with_stats_downscale(mut self, v: u32) -> Self {
        self.stats_downscale_max = v;
        self
    }
    pub fn with_label(mut self, v: impl Into<String>) -> Self {
        self.label = v.into();
        self
    }

    /// Set label from profile name + custom markers.
    pub fn auto_label(&self) -> String {
        if self.is_custom() {
            format!("{} (custom)", self.profile.label)
        } else {
            self.profile.label.to_string()
        }
    }
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self::from_profile(PipelineProfile::MED)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_profile() {
        let cfg = PipelineConfig::from_profile(PipelineProfile::HEAVY);
        assert!(cfg.use_fcs);
        assert!(cfg.use_ldci);
        assert!(cfg.use_ee);
        assert!(cfg.use_bad_pixel);
        assert!(cfg.use_lsc);
        assert!(!cfg.is_custom(), "Should match profile defaults");
    }

    #[test]
    fn test_custom_override() {
        let cfg = PipelineConfig::from_profile(PipelineProfile::LITE)
            .with_fcs(true)
            .with_ee(true);
        assert!(cfg.is_custom());
        assert!(cfg.use_fcs);
        assert!(cfg.use_ee);
        assert_eq!(cfg.auto_label(), "LITE (custom)");
    }

    #[test]
    fn test_block_chain_preview_lite() {
        let cfg = PipelineConfig::from_profile(PipelineProfile::LITE);
        let preview = cfg.block_chain_preview();
        assert!(preview.contains("Raw"));
        assert!(preview.contains("Display"));
        assert!(!preview.contains("FCS"));
        assert!(!preview.contains("LDCI"));
        assert!(!preview.contains("EE"));
    }

    #[test]
    fn test_block_chain_preview_heavy() {
        let cfg = PipelineConfig::from_profile(PipelineProfile::HEAVY);
        let preview = cfg.block_chain_preview();
        assert!(preview.contains("FCS"));
        assert!(preview.contains("LDCI"));
        assert!(preview.contains("EE"));
        assert!(preview.contains("EdgeDemosaic"));
    }

    #[test]
    fn test_block_count() {
        let lite = PipelineConfig::from_profile(PipelineProfile::LITE);
        let heavy = PipelineConfig::from_profile(PipelineProfile::HEAVY);
        // LITE (fused): 10 base, HEAVY (fused): 10 + fcs + ldci + ee + lsc = 14
        assert_eq!(lite.block_count(), 10);
        assert_eq!(heavy.block_count(), 14);
    }
}
