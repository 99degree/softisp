//! Pipeline profiles — feature complexity presets for ISP block chain construction.
//!
//! Each profile defines which blocks are enabled, trading off image quality vs
//! processing cost. Ported from `com.camcore.isp.pipeline.PipelineProfile` (Java).

use crate::blocks::*;
use crate::pipeline::IspBlock;
use log::info;

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
    /// Fuse unpack+norm+CFA into a single block (faster, fewer sessions).
    pub use_fused_unpack: bool,
    /// Fuse demosaic+CCM into a single block (saves 1 session).
    pub use_demosaic_ccm: bool,
    /// Run aux blocks (FCS, LDCI, EE) at half resolution.
    /// Inserts ResizeBlock down/up around the aux chain, 4× fewer pixels.
    pub use_aux_half_res: bool,
    /// Enable per-zone RGB means (AveragePool) for multi-illuminant AWB.
    pub use_zone_stats: bool,
    /// Enable global channel means (ReduceMean) for grey-world AWB.
    pub use_channel_means: bool,
    /// Enable tone stats (luma mean/min/max, clipped/shadows) for AE.
    pub use_tone_stats: bool,
    /// Enable coarse luminance histogram (16 bins) for AE metering.
    pub use_histogram: bool,
    /// Maximum pixel dimension for stats processing.
    /// If the input resolution exceeds this, the stats blocks are preceded
    /// by a ResizeBlock that downsamples to ≤ this dimension (preserving
    /// aspect ratio).  Set to 0 to always process at full resolution.
    ///
    /// Example: for 4K input (2160×3840), setting this to 1080 downsamples
    /// stats to 540×960 — 16× fewer pixels, 16× faster stats.
    pub stats_downscale_max: u32,
    /// Maximum pixel dimension for the main pipeline processing.
    /// If the sensor resolution exceeds this, a ResizeBlock is inserted
    /// after CFA/BLC to downsample before the expensive blocks
    /// (demosaic, CCM, tone, cosmetic, display).
    ///
    /// Set to 0 to always process at full resolution.
    /// Example: 4K sensor (3840) → set to 1920 for FHD-equivalent processing.
    pub pipeline_downscale_target: u32,
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
        use_fused_unpack: true,
        use_demosaic_ccm: true,
        use_aux_half_res: false,
        use_zone_stats: true,      // zone RGB means for basic AWB
        use_channel_means: true,   // channel means for grey-world AWB
        use_tone_stats: false,     // skip AE tone stats (LITE)
        use_histogram: false,      // skip histogram (LITE)
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
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
        use_fused_unpack: true,
        use_demosaic_ccm: true,
        use_aux_half_res: false,
        use_zone_stats: true,      // zone stats for multi-illuminant AWB
        use_channel_means: true,   // channel means for grey-world AWB
        use_tone_stats: true,      // tone stats for AE metering
        use_histogram: false,      // skip histogram (MED)
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
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
        use_fused_unpack: true,
        use_demosaic_ccm: true,
        use_aux_half_res: false,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        use_histogram: true,
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
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
        use_fused_unpack: true,
        use_demosaic_ccm: true,
        use_aux_half_res: false,
        use_zone_stats: true,      // zone stats for multi-illuminant AWB
        use_channel_means: true,   // channel means for grey-world AWB
        use_tone_stats: true,      // tone stats for AE metering
        use_histogram: true,       // 16-bin histogram (full AE stats)
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
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
        use_fused_unpack: true,
        use_demosaic_ccm: true,
        use_aux_half_res: false,
        use_zone_stats: true,      // zone stats for AWB (single block test)
        use_channel_means: false,  // skip channel means
        use_tone_stats: false,     // skip tone stats
        use_histogram: false,      // skip histogram
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
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
        use_fused_unpack: bool,
        use_demosaic_ccm: bool,
        use_aux_half_res: bool,
        use_zone_stats: bool,      // per-zone RGB means (AveragePool) for multi-illuminant AWB
        use_channel_means: bool,   // global channel means (ReduceMean) for grey-world AWB
        use_tone_stats: bool,      // luma mean/min/max + clipped/shadows for AE metering
        use_histogram: bool,       // 16-bin luminance histogram for AE and clipping detection
        stats_downscale_max: u32,  // max pixel dimension for stats (0 = full res)
        pipeline_downscale_target: u32, // max pixel dimension for main pipeline (0 = full res)
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
            use_fused_unpack,
            use_demosaic_ccm,
            use_aux_half_res,
            use_zone_stats,      // per-zone RGB means (AWB multi-illuminant)
            use_channel_means,   // global channel means (grey-world AWB)
            use_tone_stats,      // luma stats for AE metering
            use_histogram,       // 16-bin luminance histogram
            stats_downscale_max, // adaptive stats downscale
            pipeline_downscale_target, // main pipeline downscale target
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

        // RawInputBlock (always first)
        if self.use_unpack || self.use_fused_unpack {
            // Packed INT32 input (true zero-copy)
            blocks.push(Box::new(RawInputBlock::new()
                .with_elem_type(6)  // INT32
                .with_concrete_width(packed_w)));

            if self.use_fused_unpack {
                // ── Fused: unpack + normalize + CFA + BLC (single block) ──
                blocks.push(Box::new(UnpackCfaBlock::new()
                    .with_concrete_width(full_w)
                    .with_blc(true)));
            } else {
                // ── Standard packed: unpack → normalize → CFA ──
                blocks.push(Box::new(UnpackBlock::new()
                    .with_concrete_width(full_w)));
                blocks.push(Box::new(NormalizeBlock::new()));
                if self.use_bad_pixel {
                    blocks.push(Box::new(BlcBlock::new()));
                }
                blocks.push(Box::new(CfaBlock::new()));
            }
        } else {
            // Legacy FLOAT input
            blocks.push(Box::new(RawInputBlock::new()
                .with_concrete_width(full_w)));
            blocks.push(Box::new(NormalizeBlock::new()));
            if self.use_bad_pixel {
                blocks.push(Box::new(BlcBlock::new()));
            }
            blocks.push(Box::new(CfaBlock::new()));
        }

        // ── Black level correction (skipped if fused into UnpackCfaBlock) ──
        if !self.use_fused_unpack {
            blocks.push(Box::new(BlcBlock::new()));
        }

        // ── AuxHook: source data (after BLC, clean reference for aux blocks) ──
        blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_src")));

        // ── Pipeline downscale (optional, for high-res sensors) ──
        // If the sensor width exceeds pipeline_downscale_target, insert a
        // ResizeBlock to downsample before the expensive heavy blocks
        // (demosaic, CCM, tone, cosmetic, display).
        let pipeline_factor = if self.pipeline_downscale_target > 0 {
            let tw = target_width as f32;
            let target = self.pipeline_downscale_target as f32;
            if tw > target { Some(target / tw) } else { None }
        } else {
            None
        };
        if let Some(factor) = pipeline_factor {
            info!("Pipeline downscale: width={} → {} (factor={:.3})",
                target_width, (target_width as f32 * factor) as u32, factor);
            blocks.push(Box::new(ResizeBlock::new(factor)));
        }

        // ── Lens shading correction (optional) ──
        if self.use_lsc {
            blocks.push(Box::new(CcmBlock::new())); // simplified: reuse CCM for LSC
        }

        // ── Bayer white balance ──
        blocks.push(Box::new(BayerWbBlock::new()));

        // ── Demosaic + CCM ──
        if self.use_demosaic_ccm {
            // Fused: demosaic+CCM in one block (no separate CCM block)
            blocks.push(Box::new(DemosaicCcmBlock::new(bayer_pattern)));
        } else {
            blocks.push(Box::new(DemosaicBlock::new(bayer_pattern)));
            // ── Color correction matrix (standalone) ──
            blocks.push(Box::new(CcmBlock::new()));
        }

        // ── Warp (optional, EIS/deshake) ──
        if self.use_warp {
            blocks.push(Box::new(CcmBlock::new())); // placeholder
        }

        // ── Tone curve ──
        blocks.push(Box::new(ToneBlock::new()));

        // ── AuxHook: output data (after tone, for post-processing aux blocks) ──
        blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_out")));

        // ── Auxiliary blocks (atomic, individually controlled by profile) ──
        // Each aux block is independently gated by its profile flag.
        // When use_aux_half_res is set, wrap the aux chain with Resize down/up
        // to run FCS, LDCI, EE at 4× fewer pixels.
        let any_aux = self.use_fcs || self.use_ldci || self.use_ee;
        if any_aux && self.use_aux_half_res {
            blocks.push(Box::new(crate::blocks::ResizeBlock::new(0.5)));
        }
        if self.use_fcs {
            blocks.push(Box::new(FcsBlock::new()));
        }
        if self.use_ldci {
            blocks.push(Box::new(LdciBlock::new()));
        }
        if self.use_ee {
            blocks.push(Box::new(EeBlock::new()));
        }
        if any_aux && self.use_aux_half_res {
            blocks.push(Box::new(crate::blocks::ResizeBlock::new(2.0)));
        }

        // ── Display output ──
        blocks.push(Box::new(DisplayBlock::new(target_width)));

        // Wire blocks so each block's input_source points to the previous block's frame_tensor.
        // This is required before passing to GraphComposer::compose_from_vec.
        crate::pipeline::GraphComposer::wire_blocks(&mut blocks);

        blocks
    }

    /// Build auxiliary ONNX blocks (stats blocks) that branch off from
    /// the main pipeline.  These are passed as `aux_blocks` to
    /// `GraphComposer::compose_from_vec` and produce separate graph outputs
    /// that the engine reads after inference (zone stats, channel means,
    /// tone stats, histogram, etc.).
    ///
    /// IMPORTANT: Each aux block must have its `input_source` set to the
    /// name of the tensor it reads from (usually `aux_hook_src/out`).
    /// The caller is responsible for setting this before passing to
    /// `compose_from_vec`.
    pub fn build_aux_blocks(&self, input_h: i64, input_w: i64) -> Vec<Box<dyn IspBlock>> {
        let mut aux: Vec<Box<dyn IspBlock>> = Vec::new();

        // All stats blocks read from aux_hook_src (BLC-corrected linear RGB)
        // by default.  If stats_downscale_max > 0 and the input exceeds it,
        // a ResizeBlock is inserted first so stats process downscaled pixels.
        let mut stats_input_owned = String::from("aux_hook_src/out");

        // Adaptive downscale: compute factor so the longer side ≤ stats_downscale_max
        let downscale = if self.stats_downscale_max > 0 {
            let max_dim = input_h.max(input_w) as f32;
            let target = self.stats_downscale_max as f32;
            if max_dim > target {
                Some(target / max_dim)
            } else {
                None
            }
        } else {
            None
        };

        if let Some(factor) = downscale {
            let out_h = (input_h as f32 * factor).ceil() as i64;
            let out_w = (input_w as f32 * factor).ceil() as i64;
            let mut resize = ResizeBlock::new(factor)
                .with_concrete_dims(out_h, out_w);
            resize.set_input_source(&stats_input_owned);
            info!("Stats downscale: {}×{} → {}×{} (factor={:.3})",
                input_h, input_w, out_h, out_w, factor);
            stats_input_owned = resize.frame_tensor.clone();
            aux.push(Box::new(resize));
        }

        let stats_input: &str = &stats_input_owned;

        if self.use_zone_stats {
            let mut b = ZoneStatsBlock::new(6, 8)
                .with_concrete_dims(input_h, input_w);
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }
        if self.use_channel_means {
            let mut b = ChannelMeansBlock::new();
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }
        if self.use_tone_stats {
            let mut b = ToneStatsBlock::new();
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }
        if self.use_histogram {
            let mut b = CoarseHistogramBlock::new(16);
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }

        aux
    }

    /// Number of blocks in this profile's chain.
    pub fn block_count(&self) -> usize {
        let mut count: usize = if self.use_fused_unpack && self.use_unpack {
            if self.use_demosaic_ccm {
                // Fused, demosaic_ccm: raw + unpack_cfa + hook + bayer_wb + demosaic_ccm + tone + hook_out + display
                8
            } else {
                // Fused, separate demosaic+ccm: same + 1 extra
                9
            }
        } else if self.use_unpack {
            12  // standard packed: raw + unpack + norm + cfa + hooks + main + display
        } else {
            11  // legacy: raw + norm + cfa + hooks + main + display
        };
        let any_aux = self.use_fcs || self.use_ldci || self.use_ee;
        if self.use_fcs { count += 1; }
        if self.use_ldci { count += 1; }
        if self.use_ee { count += 1; }
        // Half-res aux wraps in resize_down + resize_up (2 extra blocks)
        if any_aux && self.use_aux_half_res { count += 2; }
        // bad_pixel (DPC) is handled by a separate BLC only in non-fused path
        if self.use_bad_pixel && !(self.use_fused_unpack && self.use_unpack) { count += 1; }
        if self.use_lsc { count += 1; }
        if self.use_warp { count += 1; }
        if self.use_hdr { count += 1; }
        if self.use_zone_stats { count += 1; }
        if self.use_channel_means { count += 1; }
        if self.use_tone_stats { count += 1; }
        if self.use_histogram { count += 1; }
        // Pipeline downscale (1 block if target > 0)
        if self.pipeline_downscale_target > 0 { count += 1; }
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
        assert_eq!(blocks.len(), 8, "LITE (fused, demosaic_ccm) should have 8 blocks, got {}", blocks.len());
    }

    #[test]
    fn test_build_blocks_heavy() {
        let blocks = PipelineProfile::HEAVY.build_blocks(128, 0);
        // HEAVY: base(10) + bad_pixel + fcs + ldci + ee + lsc = 15
        // Fused: 10 base + fcs + ldci + ee + lsc = 14
        // HEAVY: 8 base + fcs + ldci + ee + lsc = 12
        assert_eq!(blocks.len(), 12, "HEAVY (fused, demosaic_ccm) should have 12 blocks, got {}", blocks.len());
    }

    #[test]
    fn test_custom_profile() {
        let p = PipelineProfile::custom(
            "CUSTOM", PipelineLevel::Pro, true, true, true, true, true,
            DemosaicQuality::Edge, true, true, true, true, false, false,
            true,  // use_demosaic_ccm
            false, // use_aux_half_res
            true,  // use_zone_stats
            false, // use_channel_means
            true,  // use_tone_stats
            false, // use_histogram
            0,     // stats_downscale_max
            0,     // pipeline_downscale_target
        );
        assert_eq!(p.label, "CUSTOM");
        assert!(p.use_warp);
        assert!(!p.use_hdr);
        assert!(p.use_zone_stats);
        assert!(!p.use_channel_means);
        assert!(p.use_tone_stats);
        assert!(!p.use_histogram);
    }

    #[test]
    fn test_legacy_float_path() {
        // Old FLOAT path: use_unpack=false, no UnpackBlock in pipeline
        let p = PipelineProfile::custom(
            "LEGACY", PipelineLevel::Lite, false, false, false, false, false,
            DemosaicQuality::Standard, false, false, false, false, false, false,
            false, // use_demosaic_ccm
            false, // use_aux_half_res
            false, // use_zone_stats
            false, // use_channel_means
            false, // use_tone_stats
            false, // use_histogram
            0,     // stats_downscale_max
            0,     // pipeline_downscale_target
        );
        let blocks = p.build_blocks(128, 0);
        assert_eq!(blocks.len(), 11, "Legacy should have 11 blocks, got {}", blocks.len());
        assert_eq!(blocks[0].id(), "raw_input");
        assert_eq!(blocks[1].id(), "normalize", "Second block should be Normalize (no Unpack)");
    }

    #[test]
    fn test_block_count() {
        assert_eq!(PipelineProfile::LITE.block_count(), 10, "LITE: 8 main + 2 stats");
        assert_eq!(PipelineProfile::HEAVY.block_count(), 16, "HEAVY: 12 main + 4 stats");
    }

    #[test]
    fn test_build_aux_blocks_lite() {
        let aux = PipelineProfile::LITE.build_aux_blocks(1080, 1920);
        // LITE: zone_stats + channel_means
        assert_eq!(aux.len(), 2, "LITE stats: zone + channel means");
        assert_eq!(aux[0].id(), "zone_stats");
        assert_eq!(aux[1].id(), "channel_means");
    }

    #[test]
    fn test_build_aux_blocks_pro() {
        let aux = PipelineProfile::PRO.build_aux_blocks(1080, 1920);
        // PRO: zone_stats + channel_means + tone_stats + histogram
        assert_eq!(aux.len(), 4, "PRO stats: all 4 enabled");
    }

    #[test]
    fn test_build_aux_blocks_test() {
        let aux = PipelineProfile::TEST.build_aux_blocks(1080, 1920);
        // TEST: only zone_stats
        assert_eq!(aux.len(), 1, "TEST stats: zone_stats only");
    }
}
