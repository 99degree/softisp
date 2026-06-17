//! Pipeline profiles — feature complexity presets for ISP block chain construction.
//!
//! Each profile defines which blocks are enabled, trading off image quality vs
//! processing cost. Ported from `com.camcore.isp.pipeline.PipelineProfile` (Java).

use crate::blocks::*;
use crate::engine::OutputFormat;
use crate::pipeline::IspBlock;
use log::info;

/// Orientation transform constants (32-bit integer for fast comparison).
pub const ROTATE_NONE:  i32 = 0; // No transform
pub const ROTATE_ROT90: i32 = 1; // 90° clockwise
pub const ROTATE_ROT180: i32 = 2; // 180°
pub const ROTATE_ROT270: i32 = 3; // 90° counter-clockwise
pub const ROTATE_HFLIP: i32 = 4; // Horizontal mirror
pub const ROTATE_VFLIP: i32 = 5; // Vertical mirror

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
    /// Fuse tone (contrast × brightness) into DemosaicCcmBlock Conv.
    /// Saves one full-frame Mul+Add pass.
    pub use_fused_tone: bool,
    /// Orientation transform: 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip.
    pub rotate_mode: i32,
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
    /// EIS margin fraction (0.05 = 5%). Passed to AdaptiveDownscaleBlock for
    /// reserving edge pixels for EIS/deshake warp shifts.
    pub eis_margin: f64,
    /// Output pixel format.
    pub output_format: OutputFormat,
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
        use_fused_tone: true,
        rotate_mode: 0,
        use_zone_stats: true,      // zone RGB means for basic AWB
        use_channel_means: true,   // channel means for grey-world AWB
        use_tone_stats: false,     // skip AE tone stats (LITE)
        use_histogram: false,      // skip histogram (LITE)
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
        eis_margin: 0.0,
        output_format: OutputFormat::PackedRgb,
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
        use_fused_tone: true,
        rotate_mode: 0,
        use_zone_stats: true,      // zone stats for multi-illuminant AWB
        use_channel_means: true,   // channel means for grey-world AWB
        use_tone_stats: true,      // tone stats for AE metering
        use_histogram: false,      // skip histogram (MED)
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
        eis_margin: 0.0,
        output_format: OutputFormat::PackedRgb,
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
        use_fused_tone: true,
        rotate_mode: 0,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        use_histogram: true,
        stats_downscale_max: 540,   // stats read from ~540p (downscaled from aux_hook_src)
        pipeline_downscale_target: 0,    // 0 = disabled; set to e.g. 1920 for perf measurement
        eis_margin: 0.0,
        output_format: OutputFormat::PackedRgb,
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
        use_fused_tone: true,
        rotate_mode: 0,
        use_zone_stats: true,      // zone stats for multi-illuminant AWB
        use_channel_means: true,   // channel means for grey-world AWB
        use_tone_stats: true,      // tone stats for AE metering
        use_histogram: true,       // 16-bin histogram (full AE stats)
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
        eis_margin: 0.0,
        output_format: OutputFormat::PackedRgb,
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
        use_fused_tone: true,
        rotate_mode: 0,
        use_zone_stats: true,      // zone stats for AWB (single block test)
        use_channel_means: false,  // skip channel means
        use_tone_stats: false,     // skip tone stats
        use_histogram: false,      // skip histogram
        stats_downscale_max: 0,    // full resolution
        pipeline_downscale_target: 0,
        eis_margin: 0.0,
        output_format: OutputFormat::PackedRgb,
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
        use_fused_tone: bool,
        rotate_mode: i32,         // 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip
        use_zone_stats: bool,      // per-zone RGB means (AveragePool) for multi-illuminant AWB
        use_channel_means: bool,   // global channel means (ReduceMean) for grey-world AWB
        use_tone_stats: bool,      // luma mean/min/max + clipped/shadows for AE metering
        use_histogram: bool,       // 16-bin luminance histogram for AE and clipping detection
        stats_downscale_max: u32,  // max pixel dimension for stats (0 = full res)
        pipeline_downscale_target: u32, // max pixel dimension for main pipeline (0 = full res)
        eis_margin: f64,         // EIS margin fraction (0.05 = 5%), default 0.0
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
            use_fused_tone,
            rotate_mode,
            use_zone_stats,      // per-zone RGB means (AWB multi-illuminant)
            use_channel_means,   // global channel means (grey-world AWB)
            use_tone_stats,      // luma stats for AE metering
            use_histogram,       // 16-bin luminance histogram
            stats_downscale_max, // adaptive stats downscale
            pipeline_downscale_target, // main pipeline downscale target
            eis_margin,          // EIS margin fraction
            output_format: OutputFormat::PackedRgb,
        }
    }

    /// Build an ordered list of ISP blocks according to this profile.
    ///
    /// Returns `(head, blocks)` where `head` is the first block
    /// and `blocks` is the full ordered list for `GraphComposer::compose_from_vec()`.
    pub fn build_blocks(&self, target_width: u32, bayer_pattern: i32) -> Vec<Box<dyn IspBlock>> {
        let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();
        info!("build_blocks: profile={}, target_width={}, bayer_pattern={}",
            self.label, target_width, bayer_pattern);
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
            // Use AdaptiveDownscaleBlock in "pad" mode: scales to fit within
            // target bounds, black pillarbox/letterbox bars for remainder.
            // Preserves ALL content — no edge cropping, no distortion.
            // The stats downscale (2nd AdaptiveDownscaleBlock, "crop" mode)
            // will crop out these black bars before AWB/CCT computation.
            // NOTE: If target dims match source AR, no bars appear.
            let down_w = (target_width as f64 * factor as f64).round() as i64;
            let down_h = (target_width as f64 * factor as f64 / 1.5).round() as i64; // approx
            blocks.push(Box::new(AdaptiveDownscaleBlock::new(
                down_w.max(1), down_h.max(1), 0, "edge", "pad")
                .with_margin(self.eis_margin)));
            // ── AuxHook after downscale: AWB/CCT/stats read from downscaled data ──
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_ds")));
        }

        // ── Lens shading correction (optional) ──
        if self.use_lsc {
            info!("  lsc: CcmBlock");
            blocks.push(Box::new(CcmBlock::new()));
        } else {
            info!("  lsc: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("lsc")));
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

        // ── Tone curve (fused into DemosaicCcmBlock if use_fused_tone) ──
        if self.use_fused_tone {
            info!("  tone: IDENTITY (fused into DemosaicCcmBlock)");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("tone")));
        } else {
            info!("  tone: ToneBlock");
            blocks.push(Box::new(ToneBlock::new()));
        }

        // ── AuxHook: output data (after tone, for post-processing aux blocks) ──
        blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_out")));

        // ── Auxiliary blocks (atomic, individually controlled by profile) ──
        if self.use_fcs {
            info!("  fcs: FcsBlock");
            blocks.push(Box::new(FcsBlock::new()));
        } else {
            info!("  fcs: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("fcs")));
        }
        if self.use_ldci {
            info!("  ldci: LdciBlock");
            blocks.push(Box::new(LdciBlock::new()));
        } else {
            info!("  ldci: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("ldci")));
        }
        if self.use_ee {
            info!("  ee: EeBlock");
            blocks.push(Box::new(EeBlock::new()));
        } else {
            info!("  ee: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("ee")));
        }

        // ── Display output + orientation transform (fused) ──
        let display_h = (target_width as f64 / 1.5).round() as i64; // 16:9 approx
        blocks.push(Box::new(DisplayBlock::new(target_width)
            .with_rotate(self.rotate_mode)
            .with_output_format(self.output_format)
            .with_concrete_dims(display_h, target_width as i64)));

        info!("  blocks: {} total", blocks.len());
        for (i, b) in blocks.iter().enumerate() {
            info!("    [{:2}] {}", i, b.id());
        }

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
    /// name of the tensor it reads from.  When pipeline_downscale_target is
    /// active, stats read from `aux_hook_ds/out` (downscaled Bayer);
    /// otherwise from `aux_hook_src/out` (full-res Bayer).
    pub fn build_aux_blocks(&self, input_h: i64, input_w: i64) -> Vec<Box<dyn IspBlock>> {
        let mut aux: Vec<Box<dyn IspBlock>> = Vec::new();
        info!("build_aux_blocks: profile={}, input={}×{}", self.label, input_h, input_w);

        // Stats blocks read from aux_hook_ds (downscaled Bayer) when pipeline
        // downscale is active, otherwise fall back to aux_hook_src (full-res).
        // Full-resolution aux_hook_src is always available for AF on CPU.
        let stats_input_base = if self.pipeline_downscale_target > 0 {
            "aux_hook_ds/out"
        } else {
            "aux_hook_src/out"
        };
        let mut stats_input_owned = String::from(stats_input_base);

        // Stats downscale: AdaptiveDownscaleBlock in "crop" mode.
        // Crops to match target aspect ratio then resizes — maintains aspect
        // ratio without distortion, no black bars. This also handles removing
        // any pillarbox/letterbox from the pipeline
        // downscale, then further reducing resolution for cheap stats.
        // The target is computed from stats_downscale_max to limit the
        // longer side, preserving aspect ratio.
        let stats_max = self.stats_downscale_max;
        if stats_max > 0 {
            let max_dim = input_h.max(input_w) as f32;
            let target = stats_max as f32;
            if max_dim > target {
                let factor = target / max_dim;
                // Compute the actual dims after pipeline downscale (if any)
                let pipe_factor = if self.pipeline_downscale_target > 0 {
                    let pw = input_w as f32;
                    let pt = self.pipeline_downscale_target as f32;
                    if pw > pt { Some(pt / pw) } else { None }
                } else {
                    None
                };
                // Effective dims feeding into this stats downscale
                let (src_h, src_w) = match pipe_factor {
                    Some(pf) => {
                        let ph = (input_h as f32 * pf).round() as i64;
                        let pw = (input_w as f32 * pf).round() as i64;
                        (ph.max(1), pw.max(1))
                    }
                    None => (input_h, input_w),
                };
                // Target dims from stats_max constraint (crop mode keeps aspect)
                let tgt_h = (src_h as f32 * factor).ceil() as i64;
                let tgt_w = (src_w as f32 * factor).ceil() as i64;
                let mut ds = AdaptiveDownscaleBlock::new(
                    tgt_w.max(1), tgt_h.max(1), 0, "constant", "crop")
                    .with_concrete_dims(src_h, src_w);
                ds.set_input_source(&stats_input_owned);
                info!("Stats downscale: {}×{} → {}×{} (factor={:.3}, crop mode)",
                    src_h, src_w, tgt_h.max(1), tgt_w.max(1), factor);
                stats_input_owned = ds.frame_tensor.clone();
                aux.push(Box::new(ds));
            }
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
        // Calibration stats for AF (quad-level means/vars/mins/maxs)
        // Always enabled when AF is active.
        {
            let mut b = CalibrationBlock::new()
                .with_concrete_dims(input_h, input_w);
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }

        info!("  aux blocks: {} total", aux.len());
        for (i, b) in aux.iter().enumerate() {
            info!("    [{:2}] {}", i, b.id());
        }

        aux
    }
    pub fn block_count(&self) -> usize {
        self.build_blocks(128, 0).len() + self.build_aux_blocks(128, 128).len()
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
        // LITE: always has 12 main-chain blocks (identity placeholders for disabled features)
        assert_eq!(blocks.len(), 12, "LITE (fused, demosaic_ccm) should have 12 blocks, got {}", blocks.len());
    }

    #[test]
    fn test_build_blocks_heavy() {
        let blocks = PipelineProfile::HEAVY.build_blocks(128, 0);
        // HEAVY: all blocks present (no identity placeholders)
        assert_eq!(blocks.len(), 12, "HEAVY (fused, demosaic_ccm) should have 12 blocks, got {}", blocks.len());
    }

    #[test]
    fn test_custom_profile() {
        let p = PipelineProfile::custom(
            "CUSTOM", PipelineLevel::Pro, true, true, true, true, true,
            DemosaicQuality::Edge, true, true, true, true, false, false,
            true,  // use_demosaic_ccm
            true,  // use_fused_tone
            0,     // rotate_mode: none
            true,  // use_zone_stats
            false, // use_channel_means
            true,  // use_tone_stats
            false, // use_histogram
            0,     // stats_downscale_max
            0,     // pipeline_downscale_target
            0.0,   // eis_margin
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
            false, // use_fused_tone
            0,     // rotate_mode: none
            false, // use_zone_stats
            false, // use_channel_means
            false, // use_tone_stats
            false, // use_histogram
            0,     // stats_downscale_max
            0,     // pipeline_downscale_target
            0.0,   // eis_margin
        );
        let blocks = p.build_blocks(128, 0);
        assert_eq!(blocks.len(), 15, "Legacy should have 15 blocks (all with identity placeholders), got {}", blocks.len());
        assert_eq!(blocks[0].id(), "raw_input");
        assert_eq!(blocks[1].id(), "normalize", "Second block should be Normalize (no Unpack)");
    }

    #[test]
    fn test_block_count() {
        assert_eq!(PipelineProfile::LITE.block_count(), 14, "LITE: 12 main + 2 stats");
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
