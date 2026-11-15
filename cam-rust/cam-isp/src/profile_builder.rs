//! Pipeline profile builder — constructs ISP block chains from profile config.
//!
//! Extracted from `profile.rs` to separate profile constants from block
//! construction logic. Each ``PipelineProfile`` can build its block chain
//! via the methods here.

use crate::blocks::*;
use crate::pipeline::{BlockOpsetMode, IspBlock};
use crate::profile::PipelineProfile;
use log::info;

impl PipelineProfile {
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
                // Standard RAW Bayer pipeline
                if self.force_input16 {
                    // Pure 16-bit raw input [1,1,H,W] INT16 — no packed tensor, no
                    // split. MNN's converter upcasts the INT16 input to FLOAT32, and
                    // the engine converts the u16 buffer to f32 (RawFloat mode).
                    // UnpackCfaBlock casts INT16→FLOAT32 and extracts the 4 Bayer
                    // quadrants via a stride-2 conv → [1,4,H/2,W/2].
                    blocks.push(Box::new(
                        RawInput16Block::new()
                            .with_concrete_width(full_w)
                            .with_concrete_height(concrete_h),
                    ));
                    blocks.push(Box::new(
                        crate::blocks::UnpackCfaBlock::new()
                            .with_mode(crate::blocks::UnpackMode::NativeInt16)
                            .with_fast_unpack(true) // Conv-based extract (unpack_w/unpack_b)
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
                        RawInput16Block::new()
                            .with_concrete_width(full_w)
                            .with_concrete_height(concrete_h),
                    ));
                    blocks.push(Box::new(
                        crate::blocks::UnpackCfaBlock::new()
                            .with_mode(crate::blocks::UnpackMode::NativeInt16)
                            .with_fast_unpack(true)
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

        if self.use_bad_pixel {
            blocks.push(Box::new(BlcBlock::with_instance("dpc")));
        }
        blocks.push(Box::new(BlcBlock::new()));

        blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_src")));

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

        blocks.push(Box::new(BayerWbBlock::new()));

        blocks.push(Box::new(DemosaicBlock::new(bayer_pattern)));
        // Unique instance names: multiple CcmBlocks share the default
        // "CcmBlock/applied|frame" tensors, which alias in the ONNX→MNN
        // converter and silently drop the demosaic chain.
        blocks.push(Box::new(CcmBlock::with_instance("ccm")));

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
            let ds = AdaptiveDownscaleBlock::new(ds_w as i64, ds_h as i64, 0, "edge", "fit")
                .with_concrete_dims(rgb_h as i64, rgb_w as i64);
            // Use the block's *own* idea of its output size, not the requested
            // target: MNN derives the Resize extent as `scale * input` in
            // float, so an inexact ratio lands a pixel or two away and every
            // downstream shape check would disagree with the tensor.
            let (tail_h_i, tail_w_i) = ds.out_dims();
            blocks.push(Box::new(ds));
            (tail_w_i as u32, tail_h_i as u32)
        } else {
            (rgb_w, rgb_h)
        };

        if self.use_warp {
            blocks.push(Box::new(CcmBlock::with_instance("warp")));
        }

        blocks.push(Box::new(ToneBlock::new()));

        blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_out")));

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
            // EE emits a fused `isp.ee` extra op, which MNN's converter rewrites
            // into a Vulkan Extra op backed by a SPIR-V shader — but only when
            // the node is reachable as the chain's sole consumer of its input.
            // Any pass-through block sitting directly in front of it (`ldci`
            // here is an Identity when LDCI is off) makes the converter drop the
            // fusion, leaving a dead `isp.ee` whose output reads as zeros. The
            // `Primitive` form is a plain `Conv`, which has no such requirement.
            blocks.push(Box::new(
                EeBlock::new().with_opset_mode(BlockOpsetMode::Primitive),
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

        if self.use_gamma {
            info!("  gamma: GammaBlock");
            blocks.push(Box::new(crate::blocks::GammaBlock::new(2.2)));
        } else {
            info!("  gamma: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("gamma")));
        }

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
                .with_opset_mode(self.opset_mode)
                .with_concrete_dims(tail_h as i64, tail_w as i64),
        ));

        info!("  blocks: {} total", blocks.len());
        for (i, b) in blocks.iter().enumerate() {
            info!("    [{:2}] {}", i, b.id());
        }

        let blocks = crate::pipeline::GraphComposer::wire_blocks_with_identities(blocks);

        info!("  blocks + identities: {} total", blocks.len());
        for (i, b) in blocks.iter().enumerate() {
            info!("    [{:2}] {}", i, b.id());
        }

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
        let mut stats_input_owned = String::from(stats_input_base);

        // Stats downscale when pipeline_downscale_target > 0
        let stats_max = self.stats_downscale_max;
        if stats_max > 0 {
            let max_dim = input_h.max(input_w) as f32;
            let target = stats_max as f32;
            if max_dim > target {
                let factor = target / max_dim;
                let pipe_factor = if self.pipeline_downscale_target > 0 {
                    let pw = input_w as f32;
                    let pt = self.pipeline_downscale_target as f32;
                    if pw > pt {
                        Some(pt / pw)
                    } else {
                        None
                    }
                } else {
                    None
                };
                let (src_h, src_w) = match pipe_factor {
                    Some(pf) => {
                        let ph = (input_h as f32 * pf).round() as i64;
                        let pw = (input_w as f32 * pf).round() as i64;
                        (ph.max(1), pw.max(1))
                    }
                    None => (input_h, input_w),
                };
                let tgt_h = (src_h as f32 * factor).ceil() as i64;
                let tgt_w = (src_w as f32 * factor).ceil() as i64;
                let mut ds =
                    AdaptiveDownscaleBlock::new(tgt_w.max(1), tgt_h.max(1), 0, "constant", "crop")
                        .with_concrete_dims(src_h, src_w);
                ds.set_input_source(&stats_input_owned);
                info!(
                    "Stats downscale: {}×{} → {}×{} (factor={:.3}, crop mode)",
                    src_h,
                    src_w,
                    tgt_h.max(1),
                    tgt_w.max(1),
                    factor
                );
                stats_input_owned = ds.frame_tensor.clone();
                aux.push(Box::new(ds));
            }
        }

        let stats_input: &str = &stats_input_owned;

        if self.use_zone_stats {
            let mut b = ZoneStatsBlock::new(6, 8).with_concrete_dims(input_h, input_w);
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
        {
            let mut b = CalibrationBlock::new().with_concrete_dims(input_h, input_w);
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
