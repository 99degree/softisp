//! Pipeline profile builder — constructs ISP block chains from profile config.
//!
//! Extracted from `profile.rs` to separate profile constants from block
//! construction logic. Each ``PipelineProfile`` can build its block chain
//! via the methods here.

use crate::blocks::PipelineAllBlock;
use crate::blocks::*;
use crate::pipeline::{BlockOpsetMode, IspBlock};
use crate::profile::PipelineProfile;
use log::info;

impl PipelineProfile {
    /// Opset mode for `ToneBlock`, derived from `opset_mode` + `FuseStages`.
    pub fn tone_opset_mode(&self) -> BlockOpsetMode {
        if self
            .fuse_stages
            .allows(self.opset_mode, self.fuse_stages.tone)
        {
            BlockOpsetMode::MnnIspMin
        } else {
            BlockOpsetMode::Primitive
        }
    }

    /// Opset mode for `ToneLutBlock`, derived from `opset_mode` + `FuseStages`.
    pub fn tone_lut_opset_mode(&self) -> BlockOpsetMode {
        if self
            .fuse_stages
            .allows(self.opset_mode, self.fuse_stages.tonelut)
        {
            BlockOpsetMode::MnnIspMin
        } else {
            BlockOpsetMode::Primitive
        }
    }

    /// Opset mode for `DisplayBlock`, derived from `opset_mode` + `FuseStages`.
    pub fn display_opset_mode(&self) -> BlockOpsetMode {
        if self
            .fuse_stages
            .allows(self.opset_mode, self.fuse_stages.display)
        {
            BlockOpsetMode::MnnIspMin
        } else {
            BlockOpsetMode::Primitive
        }
    }

    /// Build an ordered list of ISP blocks according to this profile.
    pub fn build_blocks(
        &self,
        target_width: u32,
        sensor_height: u32,
        bayer_pattern: i32,
    ) -> Vec<Box<dyn IspBlock>> {
        let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();
        info!(
            "build_blocks: profile={}, target_width={}, sensor_height={}, bayer_pattern={}, input_type={}",
            self.label, target_width, sensor_height, bayer_pattern, self.input_type
        );
        let full_w = target_width as i64;
        let packed_w = (target_width / 2) as i64;

        let concrete_h = sensor_height as i64;
        assert!(
            concrete_h > 0,
            "{} profile: concrete_h must be > 0, got {} for width {}",
            self.label,
            concrete_h,
            target_width
        );
        assert!(
            packed_w > 0,
            "{} profile: packed_w must be > 0, got {} for width {}",
            self.label,
            packed_w,
            target_width
        );

        // Handle different input types
        match self.input_type {
            "yuv" => {
                // YUV input path: YUV input block -> YuvToRgbBlock
                info!("  Using YUV input pipeline");
                blocks.push(Box::new(
                    Yuv420InputBlock::new()
                        .with_concrete_width(full_w)
                        .with_concrete_height(concrete_h),
                ));
                blocks.push(Box::new(YuvToRgbBlock::new()));
            }
            "raw" => {
                // ── WHOLE PIPELINE ──────────────────────────────────────
                //
                // One `isp.pipeline` dispatch from the sensor plane to the
                // display tensor. Checked first because it subsumes every
                // per-stage decision below: there is no unpack, no normalize, no
                // demosaic node — the op's stage mask is the chain.
                //
                // It takes a full-resolution float raw plane, so the head is
                // `RawInput16Block` (declared INT16, upcast to f32 by the
                // converter) even though the per-stage chain uses the packed
                // INT32 head. That is not an inconsistency: the fused op reads
                // one sample per pixel, so there is no packed pair to split.
                if self
                    .fuse_stages
                    .allows(self.opset_mode, self.fuse_stages.whole_pipeline)
                {
                    let stages = crate::blocks::PipelineStages::PREVIEW
                        // The 3A needs BLC/WB/CCM applied to the *same* values it
                        // measures; with no stats readback in this chain there is
                        // nothing to close the loop on, so leave them running but
                        // fed from the controller snapshot the engine writes.
                        .0;
                    // `LIFT` is the S-curve; with shadow_lift 0 it is a no-op, so
                    // leave it on — the bit is what makes `ToneParams` reachable
                    // if a caller ever sets it.
                    info!(
                        "  head: isp.pipeline (whole RAW->display chain, stage mask 0x{stages:x})"
                    );
                    blocks.push(Box::new(
                        RawInput16Block::new()
                            .with_concrete_width(full_w)
                            .with_concrete_height(concrete_h),
                    ));
                    blocks.push(Box::new(
                        PipelineAllBlock::new(
                            full_w,
                            concrete_h,
                            target_width as i64 / 2,
                            (sensor_height / 2) as i64,
                        )
                        .with_bayer_pattern(bayer_pattern)
                        .with_stages(crate::blocks::PipelineStages(
                            stages | crate::blocks::PipelineStages::LEVELS,
                        )),
                    ));
                    // Done: the op *is* the chain. Everything below builds the
                    // per-stage version of the same picture, and appending it
                    // after the fused op applies the whole ISP twice — which
                    // renders (a second demosaic of an RGB frame) but is not what
                    // this profile means.
                    return crate::pipeline::GraphComposer::elide_noops(blocks);
                } else if self
                    .fuse_stages
                    .allows(self.opset_mode, self.fuse_stages.unpack32_blc_normal_cfa)
                {
                    // One `isp.unpack32_blc_normal_cfa` dispatch replaces the four
                    // blocks RawInput16 → UnpackCfa → Normalize → Blc, converting
                    // the packed INT32 RAW word pair into float, dividing by
                    // sensor_max, subtracting BLC, and extracting the CFA quad —
                    // all in one GPU pass. The host passes the sensor buffer
                    // verbatim (W*H u16 samples are W/2 int32 words per row); no
                    // host-side split and no widening.
                    info!(
                        "  head: isp.unpack32_blc_normal_cfa \
                         (fused unpack+normalize+blc+cfa from packed INT32 RAW, owns the graph input)"
                    );
                    blocks.push(Box::new(
                        NormalizeBlcUnpack32Cfa::new()
                            .with_concrete_dims(concrete_h, full_w)
                            .with_sensor_max(65535.0)
                            .with_bayer_pattern(bayer_pattern)
                            .with_opset_mode(BlockOpsetMode::MnnIspMin),
                    ));
                } else if self
                    .fuse_stages
                    .allows(self.opset_mode, self.fuse_stages.unpack16_blc_normal_cfa)
                {
                    // One `isp.unpack16_blc_normal_cfa` dispatch replaces the four
                    // blocks RawInput16 → UnpackCfa → Normalize → Blc, converting
                    // INT16→FLOAT32, dividing by sensor_max, subtracting BLC, and
                    // extracting the CFA quad — all in one GPU pass. The host passes
                    // the INT16 sensor buffer verbatim; no host-side even/odd split.
                    info!(
                        "  head: isp.unpack16_blc_normal_cfa \
                         (fused unpack+normalize+blc+cfa from INT16, owns the graph input)"
                    );
                    blocks.push(Box::new(
                        NormalizeBlcUnpack16Cfa::new()
                            .with_concrete_dims(concrete_h, full_w)
                            .with_sensor_max(65535.0)
                            .with_bayer_pattern(bayer_pattern)
                            .with_opset_mode(BlockOpsetMode::MnnIspMin),
                    ));
                } else if self
                    .fuse_stages
                    .allows(self.opset_mode, self.fuse_stages.unpack_blc_normal_cfa)
                {
                    // One `isp.unpack_blc_normal_cfa` dispatch replaces the five
                    // blocks that cast, split, divide, clip and slice the
                    // full-resolution frame. The graph still starts with the
                    // packed input block — the shader reads those INT32 words
                    // directly — so this changes the middle of the head, not its
                    // interface.
                    // One block, not two. The shader declares
                    // `[1,2,H,W/2]` INT32 on the graph boundary itself, so a
                    // `RawInputPackedBlock` in front of it would be a second
                    // declaration of a boundary this block already owns — two
                    // blocks and one ONNX node.
                    info!(
                        "  head: isp.unpack_blc_normal_cfa \
                         (fused unpack+normalize+blc+cfa, owns the graph input)"
                    );
                    blocks.push(Box::new(
                        NormalizeBlcUnpackCfa::new()
                            .with_concrete_dims(concrete_h, full_w)
                            .with_sensor_max(65535.0)
                            .with_bayer_pattern(bayer_pattern)
                            .with_opset_mode(BlockOpsetMode::MnnIspMin),
                    ));
                } else if self.force_input16 {
                    // One FLOAT32 sample per pixel, widened by the engine from
                    // the u16 sensor buffer before inference. MNN's Vulkan
                    // backend cannot execute the INT16→FLOAT Cast that the
                    // native-INT16 head would put in the graph, so the head
                    // declares FLOAT32 and UnpackCfaBlock goes straight to
                    // its stride-2 Conv → [1,4,H/2,W/2].
                    blocks.push(Box::new(
                        RawFloatInputBlock::new()
                            .with_concrete_width(full_w)
                            .with_concrete_height(concrete_h),
                    ));
                    blocks.push(Box::new(
                        crate::blocks::UnpackCfaBlock::new()
                            .with_mode(crate::blocks::UnpackMode::NativeFloat)
                            .with_fast_unpack(true) // Conv-based extract (unpack_w/unpack_b)
                            .with_downscale(2) // stride_w=2 for proper 2×2 CFA extraction
                            .with_concrete_dims(concrete_h, full_w),
                    ));
                } else if self.use_unpack {
                    // Packed 2×int16-as-int32 input [1,2,H,W/2]: engine splits the u16
                    // pairs into even/odd lanes on the host; the graph only needs a
                    // Vulkan-safe Cast INT32→FLOAT32 (UnpackBlock).
                    blocks.push(Box::new(
                        RawInputPackedBlock::new()
                            .with_concrete_width(full_w) // full sensor width; block halves it
                            .with_concrete_height(concrete_h),
                    ));
                    blocks.push(Box::new(UnpackBlock::new().with_concrete_width(full_w)));
                    // NormalizeBlock (Cast + Div by sensor_max) is what puts the
                    // frame in [0,1]; every downstream block clamps against
                    // `<ns>/one`. `normalize_at_display` skips it here and folds
                    // the same factor into the display weights instead.
                    if !self.normalize_at_display {
                        blocks.push(Box::new(NormalizeBlock::new()));
                    }
                    // UnpackBlock splits packed INT32 into even/odd channels [1,2,H,W/2];
                    // CfaBlockPacked consumes them as 2 conv input channels (kernel
                    // [2,1], stride [2,1]) instead of an interleaved [1,1,H,W] tensor —
                    // keeps every op rank-4 and resize-safe for MNN resizeSession.
                    blocks.push(Box::new(CfaBlockPacked::new()));
                } else {
                    blocks.push(Box::new(
                        RawInputBlock::new()
                            .with_concrete_width(full_w)
                            .with_concrete_height(concrete_h),
                    ));
                    if !self.normalize_at_display {
                        blocks.push(Box::new(NormalizeBlock::new()));
                    }
                    blocks.push(Box::new(CfaBlock::new()));
                }
            }
            _ => {
                // Default to RAW pipeline for backward compatibility
                if self.force_input16 {
                    blocks.push(Box::new(
                        RawFloatInputBlock::new()
                            .with_concrete_width(full_w)
                            .with_concrete_height(concrete_h),
                    ));
                    blocks.push(Box::new(
                        crate::blocks::UnpackCfaBlock::new()
                            .with_mode(crate::blocks::UnpackMode::NativeFloat)
                            .with_fast_unpack(true)
                            .with_downscale(2) // stride_w=2 for proper 2×2 CFA extraction
                            .with_concrete_dims(concrete_h, full_w),
                    ));
                } else if self.use_unpack {
                    blocks.push(Box::new(
                        RawInputPackedBlock::new()
                            .with_concrete_width(full_w)
                            .with_concrete_height(concrete_h),
                    ));
                    blocks.push(Box::new(UnpackBlock::new().with_concrete_width(full_w)));
                    if !self.normalize_at_display {
                        blocks.push(Box::new(NormalizeBlock::new()));
                    }
                    blocks.push(Box::new(CfaBlockPacked::new()));
                } else {
                    blocks.push(Box::new(
                        RawInputBlock::new()
                            .with_concrete_width(full_w)
                            .with_concrete_height(concrete_h),
                    ));
                    if !self.normalize_at_display {
                        blocks.push(Box::new(NormalizeBlock::new()));
                    }
                    blocks.push(Box::new(CfaBlock::new()));
                }
            }
        }

        // The fused head subtracts the black level itself, on the packed
        // samples before the quad split — which is where it belongs. Emitting
        // the BLC blocks as well would subtract it twice.
        let head_did_blc = self
            .fuse_stages
            .allows(self.opset_mode, self.fuse_stages.unpack_blc_normal_cfa);
        if head_did_blc {
            info!("  blc: fused into the head (isp.unpack_blc_normal_cfa)");
        } else {
            if self.use_bad_pixel {
                blocks.push(Box::new(BlcBlock::with_instance("dpc")));
            }
            blocks.push(Box::new(BlcBlock::new()));
        }

        // Tap point: the stats blocks read `aux_hook_src/out`, so this identity
        // must survive elision or the stats branches lose their source.
        blocks.push(Box::new(
            crate::blocks::IdentityBlock::new("aux_hook_src").with_tap("aux_hook_src/out"),
        ));

        let pipeline_factor = if self.pipeline_downscale_target > 0 {
            let tw = target_width as f32;
            let target = self.pipeline_downscale_target as f32;
            if tw > target {
                Some(target / tw)
            } else {
                None
            }
        } else {
            None
        };
        if let Some(factor) = pipeline_factor {
            info!(
                "Pipeline downscale: width={} → {} (factor={:.3})",
                target_width,
                (target_width as f32 * factor) as u32,
                factor
            );
            let down_w = (target_width as f64 * factor as f64).round() as i64;
            let down_h = (target_width as f64 * factor as f64 / 1.5).round() as i64;
            blocks.push(Box::new(
                AdaptiveDownscaleBlock::new(down_w.max(1), down_h.max(1), 0, "edge", "pad")
                    .with_margin(self.eis_margin),
            ));
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_ds")));
        }

        if self.use_lsc {
            // Lens shading is a radial gain on the raw quad, applied here —
            // before demosaic, so the four Bayer sites each get corrected at
            // their own position rather than after they have been mixed.
            //
            // The gain map is a runtime graph input (the engine fills it at the
            // frame's geometry), which is why this is the vignetting block with
            // a 4-plane map and not a CCM: the previous 4x4 identity Conv here
            // was a placeholder that did nothing.
            info!("  lsc: VignettingBlock (4-plane, post-CFA)");
            blocks.push(Box::new(
                // The post-CFA quad is the demosaiced geometry: half the
                // sensor in each axis.
                crate::blocks::VignettingBlock::new(
                    (target_width / 2).max(1),
                    (sensor_height / 2).max(1),
                    self.lsc_strength,
                )
                .with_instance("lsc")
                .with_channels(4),
            ));
        } else {
            info!("  lsc: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("lsc")));
        }

        // The fused demosaic+CCM op already multiplies the CFA quad by a 3x4
        // matrix, so per-plane WB is a scale on that matrix's columns. Whenever
        // that op is present, do not emit a separate BayerWbBlock: the engine
        // folds the controller gains into `DEMOSAICCCMBLOCK/W` instead.
        // `wb_into_demosaic` is kept for compatibility but is not what decides
        // block presence — a fused chain without the allow bit must still be
        // able to apply WB.
        let demosaic_ccm_enabled = self
            .fuse_stages
            .allows(self.opset_mode, self.fuse_stages.demosaic_ccm);
        if demosaic_ccm_enabled {
            info!("  white balance: folded into isp.demosaic_ccm weights");
        } else {
            blocks.push(Box::new(BayerWbBlock::new()));
        }

        // Demosaic and CCM are one fused op or two primitive ones — never a mix.
        // `isp.demosaic_ccm` applies `CCM x demosaic` weights, so leaving a
        // separate CcmBlock behind would apply the colour matrix twice.
        if demosaic_ccm_enabled {
            info!("  demosaic+ccm: DemosaicCcmBlock (fused isp.demosaic_ccm)");
            // UnpackCfaBlock halves both edges, so its output is H/2 × W/2.
            // DemosaicCcmBlock does 4→3 channels at the SAME spatial resolution.
            let cfa_h = (concrete_h / 2).max(1);
            let cfa_w = (full_w / 2).max(1);
            blocks.push(Box::new(
                DemosaicCcmBlock::new(bayer_pattern)
                    .with_concrete_dims(cfa_h, cfa_w)
                    .with_opset_mode(BlockOpsetMode::MnnIspMin),
            ));
        } else {
            // UnpackCfaBlock (or CfaBlockPacked/CfaBlock) halves both edges.
            // DemosaicBlock does 4→3 channels at the SAME spatial resolution.
            let cfa_h = (concrete_h / 2).max(1);
            let cfa_w = (full_w / 2).max(1);
            blocks.push(Box::new(
                DemosaicBlock::new(bayer_pattern).with_concrete_dims(cfa_h, cfa_w),
            ));
            // Unique instance names: multiple CcmBlocks share the default
            // "CcmBlock/applied|frame" tensors, which alias in the ONNX→MNN
            // converter and silently drop the demosaic chain.
            //
            // CCM can emit a fused `isp.ccm` instead of Conv(1x1) + Clip, but
            // only while its bias is zero — `isp_ccm.comp` has coefficients for
            // the matrix and none for an offset.
            let ccm_mode = if self
                .fuse_stages
                .allows(self.opset_mode, self.fuse_stages.ccm)
            {
                BlockOpsetMode::MnnIspMin
            } else {
                BlockOpsetMode::Primitive
            };
            blocks.push(Box::new(
                CcmBlock::with_instance("ccm").with_opset_mode(ccm_mode),
            ));
        }

        // Post-CCM downscale: everything from here on is RGB at 3+ channels,
        // so this is the last point where shrinking the frame is cheap. Bayer
        // stages above run on mosaics; tone/EE/bilateral/sharpen/display below
        // all pay per RGB pixel, and the display tensor is the largest
        // allocation in the graph.
        //
        // `tail_*` is the geometry every block below this point sees, so the
        // ones that declare concrete dims (vignetting gain map, display
        // output) have to be told the reduced size rather than the sensor's.
        // Demosaic halves both edges: a 2592x1940 Bayer frame becomes
        // 1296x970 RGB (the CFA conv is a 2x2 stride-2 extract). Everything
        // below this point therefore lives in *RGB* coordinates, half the
        // sensor's, and `post_ccm_downscale` is expressed in those too.
        let rgb_w = (target_width / 2).max(1);
        let rgb_h = ((concrete_h as u32) / 2).max(1);
        let (ds_w, ds_h) = self.post_ccm_downscale;
        let (tail_w, tail_h) = if ds_w > 0 && ds_h > 0 && ds_w != rgb_w {
            info!("  post-CCM downscale: {rgb_w}x{rgb_h} -> {ds_w}x{ds_h}");

            // Fused: one `isp.downscale` dispatch instead of a `Resize` node.
            //
            // The shader does bilinear + aspect-fit + letterbox, which is what
            // this stage wants. `AdaptiveDownscaleBlock` also offers `crop` and
            // an EIS margin, and the shader understands neither — so those
            // configurations keep the primitive form rather than quietly
            // changing the geometry.
            let fused = self
                .fuse_stages
                .allows(self.opset_mode, self.fuse_stages.tail_downscale);
            if fused {
                let ds = RgbDownscaleBlock::new(ds_w as i64, ds_h as i64)
                    .with_concrete_dims(rgb_h as i64, rgb_w as i64);
                blocks.push(Box::new(ds));
                (ds_w, ds_h)
            } else {
                let ds = AdaptiveDownscaleBlock::new(ds_w as i64, ds_h as i64, 0, "edge", "fit")
                    .with_concrete_dims(rgb_h as i64, rgb_w as i64);
                // Use the block's *own* idea of its output size, not the
                // requested target: MNN derives the Resize extent as
                // `scale * input` in float, so an inexact ratio lands a pixel or
                // two away and every downstream shape check would disagree with
                // the tensor.
                let (tail_h_i, tail_w_i) = ds.out_dims();
                blocks.push(Box::new(ds));
                (tail_w_i as u32, tail_h_i as u32)
            }
        } else {
            (rgb_w, rgb_h)
        };

        if self.use_warp {
            blocks.push(Box::new(CcmBlock::with_instance("warp")));
        }

        let tone_mode = self.tone_opset_mode();
        if tone_mode == BlockOpsetMode::MnnIspMin {
            info!("  tone: isp.tone (fused)");
        }
        let tone_gamma = if self.use_gamma { 2.2 } else { 1.0 };
        blocks.push(Box::new(
            ToneBlock::new()
                .with_opset_mode(tone_mode)
                .with_gamma(tone_gamma),
        ));

        // Tap point: tone stats read `aux_hook_out/out`.
        blocks.push(Box::new(
            crate::blocks::IdentityBlock::new("aux_hook_out").with_tap("aux_hook_out/out"),
        ));

        if self.use_fcs {
            info!("  fcs: FcsBlock");
            blocks.push(Box::new(FcsBlock::new()));
        } else {
            info!("  fcs: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("fcs")));
        }
        if self.use_ldci {
            info!("  ldci: LdciBlock (fused={})", self.fuse_stages.ldci);
            blocks.push(Box::new(
                LdciBlock::new().with_opset_mode(
                    if self
                        .fuse_stages
                        .allows(self.opset_mode, self.fuse_stages.ldci)
                    {
                        BlockOpsetMode::MnnIspMin
                    } else {
                        BlockOpsetMode::Primitive
                    },
                ),
            ));
        } else {
            info!("  ldci: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("ldci")));
        }
        if self.use_ee {
            info!("  ee: EeBlock");
            // EE emits a fused `isp.ee` extra op, which MNN's converter rewrites
            // into a Vulkan Extra op backed by a SPIR-V shader — but only when
            // the node is reachable as the chain's sole consumer of its input.
            // Any pass-through block sitting directly in front of it (`ldci`
            // here is an Identity when LDCI is off) makes the converter drop the
            // fusion, leaving a dead `isp.ee` whose output reads as zeros. The
            // `Primitive` form is a plain `Conv`, which has no such requirement.
            // Primitive stays the default because the fused form needs an exact
            // upstream pattern and silently yields zeros when the converter
            // declines to rewrite it. An explicit MnnIspMin request from the
            // profile overrides that so fusion can be tried deliberately.
            blocks.push(Box::new(
                EeBlock::new().with_opset_mode(
                    if self
                        .fuse_stages
                        .allows(self.opset_mode, self.fuse_stages.ee)
                    {
                        BlockOpsetMode::MnnIspMin
                    } else {
                        BlockOpsetMode::Primitive
                    },
                ),
            ));
        } else {
            info!("  ee: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("ee")));
        }

        if self.use_bilateral {
            info!("  bilateral: BilateralBlock");
            blocks.push(Box::new(crate::blocks::BilateralBlock::new_default()));
        } else {
            info!("  bilateral: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("bilateral")));
        }

        if self.use_vignetting {
            info!("  vignetting: VignettingBlock");
            blocks.push(Box::new(crate::blocks::VignettingBlock::new_default(
                tail_w,
                (tail_w as f64 * 9.0 / 16.0) as u32,
            )));
        } else {
            info!("  vignetting: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("vignetting")));
        }

        if self.use_saturation {
            info!("  saturation: SaturationBlock");
            blocks.push(Box::new(crate::blocks::SaturationBlock::new_default()));
        } else {
            info!("  saturation: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("saturation")));
        }

        if self.use_colorspace {
            info!("  colorspace: ColorSpaceBlock");
            blocks.push(Box::new(crate::blocks::ColorSpaceBlock::rgb_to_hsv()));
        } else {
            info!("  colorspace: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("colorspace")));
        }

        // Gamma is owned by ToneBlock (primitive Pow / fused isp.tone), which
        // already sits in this chain. A separate GammaBlock would apply the
        // curve twice. See docs/isp/20261126-isp-display-absorption-audit.md.
        info!("  gamma: IDENTITY (encoded by tone stage)");
        blocks.push(Box::new(crate::blocks::IdentityBlock::new("gamma")));

        if self.use_sharpen {
            info!("  sharpen: SharpenBlock");
            blocks.push(Box::new(crate::blocks::SharpenBlock::new(0.5)));
        } else {
            info!("  sharpen: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("sharpen")));
        }

        if self.use_wavelet_denoise {
            info!("  wavelet_denoise: WaveletDenoiseBlock");
            blocks.push(Box::new(crate::blocks::WaveletDenoiseBlock::new()));
        } else {
            info!("  wavelet_denoise: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new(
                "wavelet_denoise",
            )));
        }

        if self.use_auto_contrast {
            info!("  auto_contrast: AutoContrastBlock");
            blocks.push(Box::new(crate::blocks::AutoContrastBlock::new(1.0)));
        } else {
            info!("  auto_contrast: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("auto_contrast")));
        }

        blocks.push(Box::new(
            DisplayBlock::new(tail_w)
                .with_rotate(self.rotate_mode)
                .with_output_format(self.output_format)
                .with_opset_mode(if self.display_opset_mode() == BlockOpsetMode::MnnIspMin {
                    2
                } else {
                    0
                })
                .with_concrete_dims(tail_h as i64, tail_w as i64),
        ));

        info!("  blocks: {} total", blocks.len());
        for (i, b) in blocks.iter().enumerate() {
            info!("    [{:2}] {}", i, b.id());
        }

        // Identity bridges between every pair of blocks are off by default.
        //
        // They emit `Transpose(perm=[0,1,2,3])` — a no-op that still reads and
        // writes a full frame on the GPU. On the FUSED chain that is 19 of 33
        // nodes, so most of the graph's dispatches would compute nothing.
        //
        // The original reason for them was debuggability: each bridge names its
        // position (`id_{prev}_{next}`) so a graph dump is readable. That is
        // worth it while building a pipeline and not worth 58% of the node count
        // while running one, so it is a switch rather than a constant.
        let staged = blocks.len();
        let blocks = if crate::profile::BRIDGE_IDENTITIES {
            // Debugging aid: keep every position addressable in a graph dump.
            crate::pipeline::GraphComposer::wire_blocks_with_identities(blocks)
        } else {
            crate::pipeline::GraphComposer::elide_noops(blocks)
        };

        info!(
            "  blocks wired: {} of {} stages ({} pass-through elided)",
            blocks.len(),
            staged,
            staged.saturating_sub(blocks.len())
        );
        blocks
    }

    /// Build auxiliary ONNX blocks (stats blocks) for the pipeline.
    pub fn build_aux_blocks(&self, input_h: i64, input_w: i64) -> Vec<Box<dyn IspBlock>> {
        let mut aux: Vec<Box<dyn IspBlock>> = Vec::new();
        info!(
            "build_aux_blocks: profile={}, input={}×{}",
            self.label, input_h, input_w
        );

        let stats_input_base = if self.pipeline_downscale_target > 0 {
            "aux_hook_ds/out"
        } else {
            "aux_hook_src/out"
        };

        // ── ONE downscale for all the statistics ────────────────────────────
        //
        // Every statistics block reads the same reduced tensor. Each shrinking the
        // frame for itself would traverse it once per block; one pool followed by
        // four reductions of a small tensor traverses it once.
        //
        // A mean is exact under a box filter, so this is a cheaper *input*, not an
        // approximation: the same numbers from fewer samples. `stats_downscale_max`
        // is the lever (0 keeps full resolution).
        let stat_factor: i64 = {
            let longest = input_h.max(input_w).max(1) as f64;
            let target = self.stats_downscale_max as f64;
            if target > 0.0 && longest > target {
                (longest / target).round().max(1.0) as i64
            } else {
                1
            }
        };

        let (stat_h, stat_w) = if stat_factor > 1 {
            let f = stat_factor;
            (
                ((input_h + f - 1) / f).max(1),
                ((input_w + f - 1) / f).max(1),
            )
        } else {
            (input_h, input_w)
        };

        let stats_input_owned = if stat_factor > 1 {
            // The tap carries a CFA quad (4 planes) at this point in the chain, and
            // the pool is plane-agnostic, so the same block serves the 3-plane RGB
            // consumers and `CalibrationBlock`'s 4 planes.
            let mut ds = crate::blocks::StatsDownscaleBlock::new(stat_factor, 4)
                .with_concrete_dims(input_h, input_w);
            ds.set_input_source(stats_input_base);
            info!(
                "  stats downscale: {}×{} → {}×{} (shared by every statistics block)",
                input_h, input_w, stat_h, stat_w
            );
            let out = ds.frame_tensor.clone();
            aux.push(Box::new(ds));
            out
        } else {
            stats_input_base.to_string()
        };

        let stats_input: &str = &stats_input_owned;

        if self.use_zone_stats {
            // The kernel is `stat_h / rows`, so the block is told the size of the
            // tensor it will actually read.
            let mut b = ZoneStatsBlock::new(6, 8).with_concrete_dims(stat_h, stat_w);
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }
        if self.use_channel_means {
            let mut b = ChannelMeansBlock::new()
                .with_concrete_dims(stat_h, stat_w)
                .with_downscale(stat_factor as u32);
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }
        if self.use_tone_stats {
            let mut b = ToneStatsBlock::new()
                .with_concrete_dims(stat_h, stat_w)
                .with_downscale(stat_factor as u32);
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }
        if self.use_histogram {
            let mut b = CoarseHistogramBlock::new(16);
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }
        // Calibration stats for AF (quad-level means/vars/mins/maxs).
        //
        // Only when a stats block actually runs. `CalibrationBlock` reads
        // `aux_hook_src/out`, and that tap is a pass-through the builder only
        // creates for a chain that has a mid-pipeline point to tap. A profile with
        // every stats block off — `WHOLE`, where the fused op's only output is the
        // display frame — has nothing for it to consume, and the graph is rejected
        // for reading a tensor nothing produces.
        if self.use_zone_stats
            || self.use_channel_means
            || self.use_tone_stats
            || self.use_histogram
        {
            let mut b = CalibrationBlock::new().with_concrete_dims(stat_h, stat_w);
            b.set_input_source(stats_input);
            aux.push(Box::new(b));
        }

        info!("  aux blocks: {} total", aux.len());
        for (i, b) in aux.iter().enumerate() {
            info!("    [{:2}] {}", i, b.id());
        }

        aux
    }

    /// Count total blocks (main + aux) for estimate purposes.
    pub fn block_count(&self) -> usize {
        self.build_blocks(128, 72, 0).len() + self.build_aux_blocks(72, 128).len()
    }

    /// Estimate ONNX node count for this profile.
    pub fn node_estimate(&self) -> usize {
        self.block_count() * 2 + 2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gamma_stage_is_identity_because_tone_owns_gamma() {
        let blocks = PipelineProfile::HEAVY.build_blocks(1920, 1080, 0);
        // The identity gamma stage is elided by `elide_noops`; what matters is
        // that no standalone GammaBlock survives in the composed chain.
        assert!(
            blocks
                .iter()
                .all(|b| !format!("{:?}", b).contains("GammaBlock")),
            "standalone GammaBlock must not be emitted when tone owns gamma"
        );

        let tone = blocks
            .iter()
            .find(|b| b.id() == "tone")
            .expect("tone stage must exist");
        let dbg = format!("{:?}", tone);
        assert!(dbg.contains("2.2"), "tone must receive gamma: {}", dbg);
    }

    #[test]
    fn test_display_opset_mode_tracks_fuse_stages() {
        assert_eq!(
            PipelineProfile::HEAVY.display_opset_mode(),
            BlockOpsetMode::Primitive
        );
        assert_eq!(
            PipelineProfile::FUSED.display_opset_mode(),
            BlockOpsetMode::MnnIspMin
        );
    }
}
