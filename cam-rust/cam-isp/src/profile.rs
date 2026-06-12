//! Pipeline profiles — feature complexity presets for ISP block chain construction.
//!
//! Each profile defines which blocks are enabled, trading off image quality vs
//! processing cost. Ported from `com.camcore.isp.pipeline.PipelineProfile` (Java).

use crate::blocks::*;
use crate::pipeline::IspBlock;

/// Demosaic quality selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemosaicQuality {
    /// Fast bilinear demosaic.
    Standard,
    /// Higher-quality gradient-based (Malvar 2004).
    HqLinear,
    /// Edge-aware demosaic with false color suppression.
    Edge,
}

impl Default for DemosaicQuality {
    fn default() -> Self { Self::Standard }
}

/// Feature complexity level — gates controller features.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum PipelineLevel {
    Lite = 0,
    Medium = 1,
    Heavy = 2,
    Pro = 3,
}

/// Pipeline profile defining which ISP blocks to enable.
///
/// Each profile has a fixed set of feature flags and can build a block chain
/// via `build_chain()`.
#[derive(Debug, Clone, Copy)]
pub struct PipelineProfile {
    /// Human-readable label.
    pub label: &'static str,
    /// Feature level for controller gating.
    pub level: PipelineLevel,
    /// Enable defective pixel correction (hot pixel removal).
    pub use_bad_pixel: bool,
    /// Demosaic quality.
    pub demosaic_quality: DemosaicQuality,
    /// Enable local contrast enhancement.
    pub use_local_contrast: bool,
    /// Enable unsharp mask edge enhancement.
    pub use_unsharp: bool,
    /// Enable lens shading correction.
    pub use_lsc: bool,
    /// Enable image warping (EIS/deshake).
    pub use_warp: bool,
    /// Enable HDR merge.
    pub use_hdr: bool,
}

impl PipelineProfile {
    /// Minimal pipeline: raw → norm → CFA → BLC → WB → demosaic → CCM → tone → display.
    pub const LITE: Self = Self {
        label: "LITE",
        level: PipelineLevel::Lite,
        use_bad_pixel: false,
        demosaic_quality: DemosaicQuality::HqLinear,
        use_local_contrast: false,
        use_unsharp: false,
        use_lsc: false,
        use_warp: false,
        use_hdr: false,
    };

    /// Medium: adds bad pixel correction + unsharp mask.
    pub const MED: Self = Self {
        label: "MED",
        level: PipelineLevel::Medium,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Standard,
        use_local_contrast: false,
        use_unsharp: true,
        use_lsc: false,
        use_warp: false,
        use_hdr: false,
    };

    /// Heavy: bad pixel + edge demosaic + local contrast + unsharp + LSC.
    pub const HEAVY: Self = Self {
        label: "HEAVY",
        level: PipelineLevel::Heavy,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Edge,
        use_local_contrast: true,
        use_unsharp: true,
        use_lsc: true,
        use_warp: false,
        use_hdr: false,
    };

    /// Everything-on profile: all available blocks enabled.
    pub const PRO: Self = Self {
        label: "PRO",
        level: PipelineLevel::Pro,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Edge,
        use_local_contrast: true,
        use_unsharp: true,
        use_lsc: true,
        use_warp: true,
        use_hdr: true,
    };

    /// All built-in profiles.
    pub const ALL: [Self; 4] = [Self::LITE, Self::MED, Self::HEAVY, Self::PRO];

    /// Create a custom profile with override flags.
    pub const fn custom(
        label: &'static str,
        level: PipelineLevel,
        use_bad_pixel: bool,
        demosaic_quality: DemosaicQuality,
        use_local_contrast: bool,
        use_unsharp: bool,
        use_lsc: bool,
        use_warp: bool,
        use_hdr: bool,
    ) -> Self {
        Self {
            label,
            level,
            use_bad_pixel,
            demosaic_quality,
            use_local_contrast,
            use_unsharp,
            use_lsc,
            use_warp,
            use_hdr,
        }
    }

    /// Build an ordered list of ISP blocks according to this profile.
    ///
    /// Returns `(head, blocks)` where `head` is the first block
    /// and `blocks` is the full ordered list for `GraphComposer::compose_from_vec()`.
    pub fn build_blocks(&self, target_width: u32, bayer_pattern: i32) -> Vec<Box<dyn IspBlock>> {
        let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();

        // ── Raw input ──
        blocks.push(Box::new(RawInputBlock::new()));

        // ── Normalize ──
        blocks.push(Box::new(NormalizeBlock::new()));

        // ── Defective pixel correction (optional) ──
        if self.use_bad_pixel {
            blocks.push(Box::new(BlcBlock::new())); // BLC acts as DPC in our impl
        }

        // ── CFA unpack ──
        blocks.push(Box::new(CfaBlock::new()));

        // ── Black level correction ──
        blocks.push(Box::new(BlcBlock::new()));

        // ── Lens shading correction (optional) ──
        if self.use_lsc {
            blocks.push(Box::new(CcmBlock::new())); // simplified: reuse CCM for LSC
        }

        // ── Bayer white balance ──
        blocks.push(Box::new(BayerWbBlock::new()));

        // ── Demosaic ──
        blocks.push(Box::new(DemosaicBlock::new(bayer_pattern)));

        // ── Warp (optional, EIS/deshake) ──
        if self.use_warp {
            blocks.push(Box::new(CcmBlock::new())); // placeholder
        }

        // ── Color correction matrix ──
        blocks.push(Box::new(CcmBlock::new()));

        // ── Tone curve ──
        blocks.push(Box::new(ToneBlock::new()));

        // ── Local contrast (optional) ──
        if self.use_local_contrast {
            blocks.push(Box::new(CcmBlock::new())); // placeholder
        }

        // ── Unsharp mask (optional) ──
        if self.use_unsharp {
            blocks.push(Box::new(CcmBlock::new())); // placeholder
        }

        // ── Display output ──
        blocks.push(Box::new(DisplayBlock::new(target_width)));

        blocks
    }

    /// Number of blocks in this profile's chain.
    pub fn block_count(&self) -> usize {
        let mut count: usize = 9; // base blocks
        if self.use_bad_pixel { count += 1; }
        if self.use_lsc { count += 1; }
        if self.use_local_contrast { count += 1; }
        if self.use_unsharp { count += 1; }
        if self.use_warp { count += 1; }
        if self.use_hdr { count += 1; }
        count
    }

    /// Estimate ONNX node count for this profile.
    pub fn node_estimate(&self) -> usize {
        self.block_count() * 2 + 2
    }
}

impl Default for PipelineProfile {
    fn default() -> Self { Self::MED }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lite_profile() {
        let p = PipelineProfile::LITE;
        assert_eq!(p.label, "LITE");
        assert_eq!(p.level, PipelineLevel::Lite);
        assert!(!p.use_bad_pixel);
        assert!(!p.use_unsharp);
    }

    #[test]
    fn test_heavy_profile() {
        let p = PipelineProfile::HEAVY;
        assert!(p.use_bad_pixel);
        assert!(p.use_unsharp);
        assert!(p.use_lsc);
        assert_eq!(p.demosaic_quality, DemosaicQuality::Edge);
    }

    #[test]
    fn test_build_blocks_lite() {
        let blocks = PipelineProfile::LITE.build_blocks(128, 0);
        // LITE: RawInput + Normalize + Cfa + Blc + BayerWb + Demosaic + Ccm + Tone + Display = 9
        assert_eq!(blocks.len(), 9, "LITE should have 9 blocks, got {}", blocks.len());
    }

    #[test]
    fn test_build_blocks_heavy() {
        let blocks = PipelineProfile::HEAVY.build_blocks(128, 0);
        // HEAVY: +bad pixel +LSC +local contrast +unsharp = 4 extra
        assert_eq!(blocks.len(), 13, "HEAVY should have 13 blocks, got {}", blocks.len());
    }

    #[test]
    fn test_custom_profile() {
        let p = PipelineProfile::custom(
            "CUSTOM", PipelineLevel::Pro, true,
            DemosaicQuality::Edge, true, true, true, true, false,
        );
        assert_eq!(p.label, "CUSTOM");
        assert!(p.use_warp);
        assert!(!p.use_hdr);
    }

    #[test]
    fn test_block_count() {
        assert_eq!(PipelineProfile::LITE.block_count(), 9);
        assert!(PipelineProfile::HEAVY.block_count() > 9);
    }
}
