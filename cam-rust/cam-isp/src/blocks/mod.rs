//! Atomic ISP blocks — each is a separate ONNX graph fragment.
//!
//! This module contains all the individual image processing blocks that make up
//! the ISP pipeline. Each block performs a specific transformation on the image data.
//!
//! # Block Categories
//!
//! ## Input/Output
//! - `RawInputBlock` - Interprets raw sensor data
//! - `DisplayBlock` - Converts to display format (BGRA/RGBA)
//!
//! ## Color Processing
//! - `CcmBlock` - Color Correction Matrix
//! - `ToneBlock` - Tone mapping curves
//! - `GammaBlock` - Gamma correction
//!
//! ## Demosaicing
//! - `DemosaicBlock` - Standard demosaicing
//! - `DemosaicCcmBlock` - Demosaic + CCM fusion
//! - `BayerDemosaicBlock` - Bayer-specific demosaicing
//!
//! ## Noise Reduction
//! - `LdciBlock` - Local Digital Contrast Improvement
//! - `FcsBlock` - False Color Suppression
//! - `EeBlock` - Edge Enhancement
//!
//! ## Geometric
//! - `WarpBlock` - Geometric correction (lens distortion, EIS)
//! - `ResizeBlock` - Scale up/down
//!
//! # Pipeline Flow
//!
//! ```text
//! RawInput → Normalize → CFA → BLC → WB → Demosaic → CCM → Tone → Display
//! ```
mod raw_input;
mod normalize;
mod cfa;
mod blc;
mod bayer_wb;
mod demosaic;
mod ccm;
mod tone;
mod display;
mod ee;
mod fcs;
mod ldci;
mod warp;
mod identity;
mod grayscale;
mod pyramid;
mod unpack;
mod demosaic_ccm;
mod unpack_cfa;
mod unpack_bayer_fp16;
mod resize;
mod adaptive_downscale;
mod stats;
mod hdr_merge;
mod demosaic_interp;
mod bayer_demosaic;
mod chromatic_aberration;
mod auto_contrast;
mod temporal_denoise;
mod noise_estimate;
mod gamma;
mod stereo_depth;
mod sharpen;
mod colorspace;
mod aspect_crop;
mod dyn_resize;
mod hdr_tone;
mod wavelet_denoise;
mod plugin;
mod laplacian_pyramid;
mod passthrough;
mod stage;
mod hdr_debayer;
mod blc50;
mod watermark;
mod super_res;
mod runtime_warp;

pub use raw_input::RawInputBlock;
pub use normalize::NormalizeBlock;
pub use cfa::CfaBlock;
pub use blc::BlcBlock;
pub use bayer_wb::BayerWbBlock;
pub use demosaic::DemosaicBlock;
pub use ccm::CcmBlock;
pub use tone::ToneBlock;
pub use display::DisplayBlock;
pub use ee::EeBlock;
pub use fcs::FcsBlock;
pub use ldci::LdciBlock;
pub use warp::*;
pub use identity::{IdentityBlock, FastDemosaicBlock};
pub use grayscale::GrayscaleBlock;
pub use pyramid::PyramidBlock;
pub use unpack_cfa::{UnpackCfaBlock, UnpackMode};
pub use unpack_bayer_fp16::UnpackBayerToFp16Block;
pub use unpack::UnpackBlock;
pub use demosaic_ccm::DemosaicCcmBlock;
pub use bayer_proc::{BayerProcBlock, BayerMode, BayerPattern};
pub use resize::ResizeBlock;
pub use adaptive_downscale::AdaptiveDownscaleBlock;
pub use stats::{ZoneStatsBlock, ChannelMeansBlock, ToneStatsBlock, CoarseHistogramBlock, CalibrationBlock};
pub use hdr_merge::HdrMergeBlock;
pub use demosaic_interp::DemosaicInterpBlock;
pub use bayer_demosaic::{BayerDemosaicBlock, DemosaicAlgo};
pub use chromatic_aberration::ChromaticAberrationBlock;
pub use auto_contrast::AutoContrastBlock;
pub use temporal_denoise::TemporalDenoiseBlock;
pub use noise_estimate::NoiseEstimateBlock;
pub use gamma::GammaBlock;
pub use stereo_depth::StereoDepthBlock;
pub use sharpen::SharpenBlock;
pub use colorspace::{ColorSpaceBlock, ColorSpace};
pub use aspect_crop::AspectCropBlock;
pub use dyn_resize::DynResizeBlock;
pub use hdr_tone::{HdrToneBlock, ToneOperator};
pub use wavelet_denoise::WaveletDenoiseBlock;
pub use plugin::PluginBlock;
pub use laplacian_pyramid::LaplacianPyramidBlock;
pub use passthrough::PassthroughBlock;
pub use stage::StageBlock;
pub use hdr_debayer::HdrDebayerBlock;
pub use blc50::Blc50Block;
pub use watermark::WatermarkBlock;
pub use runtime_warp::RuntimeWarpBlock;
pub use super_res::SuperResBlock;

/// Register all built-in blocks with the engine registry.
/// Called by `cam_isp::init()`.
pub fn register_builtin_blocks() {
    // No engine registration needed for blocks themselves.
}

/// Build output value info for NCHW float tensors (shared helper).
#[allow(dead_code)]
pub(crate) fn nchw_value_info(name: &str, c: i64, elem_type: i32) -> Vec<u8> {
    crate::onnx::proto::Proto::value_info(name, &[
        crate::onnx::proto::Proto::tensor_dim_value(1),
        crate::onnx::proto::Proto::tensor_dim_value(c),
        crate::onnx::proto::Proto::tensor_dim_param("H"),
        crate::onnx::proto::Proto::tensor_dim_param("W"),
    ], elem_type)
}
pub mod bayer_proc;
