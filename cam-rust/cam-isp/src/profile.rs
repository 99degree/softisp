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
    /// Enable packed INT32 input with UnpackBlock for true zero-copy.
    /// When true (default), RawInputBlock uses INT32 at half-width and
    /// UnpackBlock is injected after it. When false, uses legacy FLOAT path.
    pub use_unpack: bool,
    /// Enable false color suppression (FCS).
    pub use_fcs: bool,
    /// Enable local contrast enhancement (LDCI).
    pub use_ldci: bool,
    /// Enable edge enhancement / unsharp mask (EE).
    pub use_ee: bool,
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
    };

    /// Medium: adds bad pixel correction + unsharp mask.
    pub const MED: Self = Self {
        label: "MED",
        level: PipelineLevel::Medium,
        use_unpack: true,
        use_fcs: false,
        use_ldci: false,
        use_ee: true,
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
        use_unpack: true,
        use_fcs: true,
        use_ldci: true,
        use_ee: true,
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
        use_unpack: true,
        use_fcs: true,
        use_ldci: true,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Edge,
        use_local_contrast: true,
        use_unsharp: true,
        use_lsc: true,
        use_warp: true,
        use_hdr: true,
    };

    /// Test profile: minimal blocks for fast unit testing.
    /// Uses identity/placeholder blocks that skip expensive computation.
    pub const TEST: Self = Self {
        label: "TEST",
        level: PipelineLevel::Lite,
        use_unpack: true,
        use_bad_pixel: false,
        demosaic_quality: DemosaicQuality::Standard,
        use_fcs: false,
        use_ldci: false,
        use_ee: false,
        use_lsc: false,
        use_warp: false,
        use_hdr: false,
        use_local_contrast: false,
        use_unsharp: false,
    };

    /// All built-in profiles.
    pub const ALL: [Self; 5] = [Self::LITE, Self::MED, Self::HEAVY, Self::PRO, Self::TEST];

    /// Create a custom profile with override flags.
    pub const fn custom(
        label: &'static str,
        level: PipelineLevel,
        use_unpack: bool,
        use_fcs: bool,
        use_ldci: bool,
        use_ee: bool,
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
            use_unpack,
            use_fcs,
            use_ldci,
            use_ee,
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
        let full_w = target_width as i64;
        let packed_w = (target_width / 2) as i64;

        if self.use_unpack {
            // ── Packed INT32 input (true zero-copy: u16 buffer reinterpreted as i32) ──
            blocks.push(Box::new(RawInputBlock::new()
                .with_elem_type(6)  // INT32
                .with_concrete_width(packed_w)));

            // ── Unpack block (packed INT32 → interleaved INT32 at full width) ──
            blocks.push(Box::new(UnpackBlock::new()
                .with_concrete_width(full_w)));
        } else {
            // ── Legacy FLOAT input (no unpack, direct UINT16→FLOAT pipeline) ──
            blocks.push(Box::new(RawInputBlock::new()
                .with_concrete_width(full_w)));
        }

        // ── Normalize ──
        blocks.push(Box::new(NormalizeBlock::new()));

        // ── Defective pixel correction (optional) ──
        if self.use_bad_pixel {
            blocks.push(Box::new(BlcBlock::new())); // BLC acts as DPC in our impl
        }

        // ── CFA unpack ──
        blocks.push(Box::new(CfaBlock::new()));

        // ── AuxHook: source data (after CFA, before tone) ──
        blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_src")));

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

        // ── AuxHook: output data (after tone, for post-processing aux blocks) ──
        blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_out")));

        // ── Auxiliary blocks (atomic, individually controlled by profile) ──
        // Each aux block is independently gated by its profile flag, no signaling required.
        if self.use_fcs {
            blocks.push(Box::new(FcsBlock::new()));
        }
        if self.use_ldci {
            blocks.push(Box::new(LdciBlock::new()));
        }
        if self.use_ee {
            blocks.push(Box::new(EeBlock::new()));
        }

        // ── Display output ──
        blocks.push(Box::new(DisplayBlock::new(target_width)));

        // Wire blocks so each block's input_source points to the previous block's frame_tensor.
        // This is required before passing to GraphComposer::compose_from_vec.
        crate::pipeline::GraphComposer::wire_blocks(&mut blocks);

        blocks
    }

    /// Number of blocks in this profile's chain.
    pub fn block_count(&self) -> usize {
        let mut count: usize = if self.use_unpack { 12 } else { 11 }; // +2 for AuxHooks
        if self.use_fcs { count += 1; }
        if self.use_ldci { count += 1; }
        if self.use_ee { count += 1; }
        if self.use_bad_pixel { count += 1; }
        if self.use_lsc { count += 1; }
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
        assert!(!p.use_fcs);
        assert!(!p.use_ldci);
        assert!(!p.use_ee);
        assert!(!p.use_bad_pixel);
    }

    #[test]
    fn test_heavy_profile() {
        let p = PipelineProfile::HEAVY;
        assert!(p.use_fcs);
        assert!(p.use_ldci);
        assert!(p.use_ee);
        assert!(p.use_bad_pixel);
        assert!(p.use_lsc);
        assert_eq!(p.demosaic_quality, DemosaicQuality::Edge);
    }

    #[test]
    fn test_build_blocks_lite() {
        let blocks = PipelineProfile::LITE.build_blocks(128, 0);
        // LITE: raw → unpack → norm → cfa → blc → bayer_wb → demo → ccm → tone → display = 10
        assert_eq!(blocks.len(), 12, "LITE should have 12 blocks (+2 hooks), got {}", blocks.len());
    }

    #[test]
    fn test_build_blocks_heavy() {
        let blocks = PipelineProfile::HEAVY.build_blocks(128, 0);
        // HEAVY: base(10) + bad_pixel + fcs + ldci + ee + lsc = 15
        assert_eq!(blocks.len(), 17, "HEAVY should have 17 blocks (+2 hooks), got {}", blocks.len());
    }

    #[test]
    fn test_custom_profile() {
        let p = PipelineProfile::custom(
            "CUSTOM", PipelineLevel::Pro, true, true, true, true, true,
            DemosaicQuality::Edge, true, true, true, true, false,
        );
        assert_eq!(p.label, "CUSTOM");
        assert!(p.use_warp);
        assert!(!p.use_hdr);
    }

    #[test]
    fn test_legacy_float_path() {
        // Old FLOAT path: use_unpack=false, no UnpackBlock in pipeline
        let p = PipelineProfile::custom(
            "LEGACY", PipelineLevel::Lite, false, false, false, false, false,
            DemosaicQuality::Standard, false, false, false, false, false,
        );
        let blocks = p.build_blocks(128, 0);
        assert_eq!(blocks.len(), 11, "Legacy should have 11 blocks (+2 hooks), got {}", blocks.len());
        assert_eq!(blocks[0].id(), "raw_input");
        assert_eq!(blocks[1].id(), "normalize", "Second block should be Normalize (no Unpack)");
    }

    #[test]
    fn test_block_count() {
        // LITE: 12 base blocks (+2 hooks), no extras
        assert_eq!(PipelineProfile::LITE.block_count(), 12);
        // HEAVY: 12 base + fcs + ldci + ee + bad_pixel + lsc = 17
        assert_eq!(PipelineProfile::HEAVY.block_count(), 17);
    }
}
