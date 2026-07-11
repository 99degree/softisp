//! Pipeline profile builder — constructs ISP block chains from profile config.
//!
//! Extracted from `profile.rs` to separate profile constants from block
//! construction logic. Each ``PipelineProfile`` can build its block chain
//! via the methods here.

use crate::blocks::*;
use crate::profile::PipelineProfile;
use crate::pipeline::IspBlock;
use log::info;

impl PipelineProfile {
    /// Build an ordered list of ISP blocks according to this profile.
    pub fn build_blocks(&self, target_width: u32, bayer_pattern: i32) -> Vec<Box<dyn IspBlock>> {
        let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();
        info!("build_blocks: profile={}, target_width={}, bayer_pattern={}",
            self.label, target_width, bayer_pattern);
        let full_w = target_width as i64;
        let packed_w = (target_width / 2) as i64;

        let concrete_h = (target_width as f64 / 16.0 * 9.0).round() as i64;
        assert!(concrete_h > 0, "{} profile: concrete_h must be > 0, got {} for width {}", self.label, concrete_h, target_width);
        assert!(packed_w > 0, "{} profile: packed_w must be > 0, got {} for width {}", self.label, packed_w, target_width);
        if self.use_unpack || self.use_fused_unpack {
            blocks.push(Box::new(RawInputBlock::new()
                .with_elem_type(6)
                .with_concrete_width(packed_w)
                .with_concrete_height(concrete_h)));

            if self.use_fused_unpack {
                blocks.push(Box::new(UnpackCfaBlock::new()
                    .with_concrete_width(full_w)
                    .with_blc(true)));
            } else {
                blocks.push(Box::new(UnpackBlock::new()
                    .with_concrete_width(full_w)));
                blocks.push(Box::new(NormalizeBlock::new()));
                if self.use_bad_pixel {
                    blocks.push(Box::new(BlcBlock::new()));
                }
                blocks.push(Box::new(CfaBlock::new()));
            }
        } else {
            blocks.push(Box::new(RawInputBlock::new()
                .with_concrete_width(full_w)
                .with_concrete_height(concrete_h)));
            blocks.push(Box::new(NormalizeBlock::new()));
            if self.use_bad_pixel {
                blocks.push(Box::new(BlcBlock::new()));
            }
            blocks.push(Box::new(CfaBlock::new()));
        }

        if !self.use_fused_unpack {
            blocks.push(Box::new(BlcBlock::new()));
        }

        blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_src")));

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
            let down_w = (target_width as f64 * factor as f64).round() as i64;
            let down_h = (target_width as f64 * factor as f64 / 1.5).round() as i64;
            blocks.push(Box::new(AdaptiveDownscaleBlock::new(
                down_w.max(1), down_h.max(1), 0, "edge", "pad")
                .with_margin(self.eis_margin)));
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("aux_hook_ds")));
        }

        if self.use_lsc {
            info!("  lsc: CcmBlock");
            blocks.push(Box::new(CcmBlock::new()));
        } else {
            info!("  lsc: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("lsc")));
        }

        blocks.push(Box::new(BayerWbBlock::new()));

        if self.use_demosaic_ccm {
            blocks.push(Box::new(DemosaicCcmBlock::new(bayer_pattern)));
        } else {
            blocks.push(Box::new(DemosaicBlock::new(bayer_pattern)));
            blocks.push(Box::new(CcmBlock::new()));
        }

        if self.use_warp {
            blocks.push(Box::new(CcmBlock::new()));
        }

        if self.use_fused_tone {
            info!("  tone: IDENTITY (fused into DemosaicCcmBlock)");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("tone")));
        } else {
            info!("  tone: ToneBlock");
            blocks.push(Box::new(ToneBlock::new()));
        }

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
            blocks.push(Box::new(EeBlock::new()));
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
                target_width,
                (target_width as f64 * 9.0 / 16.0) as u32,
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
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("wavelet_denoise")));
        }

        if self.use_auto_contrast {
            info!("  auto_contrast: AutoContrastBlock");
            blocks.push(Box::new(crate::blocks::AutoContrastBlock::new(1.0)));
        } else {
            info!("  auto_contrast: IDENTITY");
            blocks.push(Box::new(crate::blocks::IdentityBlock::new("auto_contrast")));
        }

        blocks.push(Box::new(DisplayBlock::new(target_width)
            .with_rotate(self.rotate_mode)
            .with_output_format(self.output_format)));

        info!("  blocks: {} total", blocks.len());
        for (i, b) in blocks.iter().enumerate() {
            info!("    [{:2}] {}", i, b.id());
        }

        crate::pipeline::GraphComposer::wire_blocks(&mut blocks);
        blocks
    }

    /// Build auxiliary ONNX blocks (stats blocks) for the pipeline.
    pub fn build_aux_blocks(&self, input_h: i64, input_w: i64) -> Vec<Box<dyn IspBlock>> {
        let mut aux: Vec<Box<dyn IspBlock>> = Vec::new();
        info!("build_aux_blocks: profile={}, input={}×{}", self.label, input_h, input_w);

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
                    if pw > pt { Some(pt / pw) } else { None }
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

    /// Count total blocks (main + aux) for estimate purposes.
    pub fn block_count(&self) -> usize {
        self.build_blocks(128, 0).len() + self.build_aux_blocks(128, 128).len()
    }

    /// Estimate ONNX node count for this profile.
    pub fn node_estimate(&self) -> usize {
        self.block_count() * 2 + 2
    }
}
