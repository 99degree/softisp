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
mod adaptive_downscale;
mod aspect_crop;
mod auto_contrast;
mod bayer_demosaic;
mod bayer_wb;
pub mod bilateral;
mod blc;
mod blc50;
mod ccm;
mod cfa;
mod chromatic_aberration;
mod colorspace;
mod demosaic;
mod demosaic_ccm;
mod demosaic_interp;
mod display;
mod dyn_resize;
mod ee;
mod fcs;
mod gamma;
pub mod gpu_warp;
mod grayscale;
mod hdr_debayer;
mod hdr_merge;
mod hdr_tone;
mod identity;
mod laplacian_pyramid;
mod ldci;
mod noise_estimate;
mod normalize;
mod passthrough;
mod plugin;
mod pyramid;
mod raw_input;
mod resize;
pub mod runtime_warp;
pub mod saturation;
mod sharpen;
mod stage;
mod stats;
mod stereo_depth;
mod super_res;
mod temporal_denoise;
mod tone;
mod unpack;
mod unpack_bayer_fp16;
mod unpack_blc16;
mod unpack_cfa;
pub mod vignetting;
mod warp;
mod watermark;
mod wavelet_denoise;

pub use adaptive_downscale::AdaptiveDownscaleBlock;
pub use aspect_crop::AspectCropBlock;
pub use auto_contrast::AutoContrastBlock;
pub use bayer_demosaic::{BayerDemosaicBlock, DemosaicAlgo};
pub use bayer_proc::{BayerMode, BayerPattern, BayerProcBlock};
pub use bayer_wb::BayerWbBlock;
pub use bilateral::BilateralBlock;
pub use blc::BlcBlock;
pub use blc50::Blc50Block;
pub use ccm::CcmBlock;
pub use cfa::{CfaBlock, CfaBlockPacked};
pub use chromatic_aberration::ChromaticAberrationBlock;
pub use colorspace::{ColorSpace, ColorSpaceBlock};
pub use demosaic::DemosaicBlock;
pub use demosaic_ccm::DemosaicCcmBlock;
pub use demosaic_interp::DemosaicInterpBlock;
pub use display::DisplayBlock;
pub use dyn_resize::DynResizeBlock;
pub use ee::EeBlock;
pub use fcs::FcsBlock;
pub use gamma::GammaBlock;
pub use gpu_warp::GpuWarpBlock;
pub use grayscale::GrayscaleBlock;
pub use hdr_debayer::HdrDebayerBlock;
pub use hdr_merge::HdrMergeBlock;
pub use hdr_tone::{HdrToneBlock, ToneOperator};
pub use identity::{BridgeIdentityBlock, FastDemosaicBlock, IdentityBlock};
pub use laplacian_pyramid::LaplacianPyramidBlock;
pub use ldci::LdciBlock;
pub use noise_estimate::NoiseEstimateBlock;
pub use normalize::NormalizeBlock;
pub use passthrough::PassthroughBlock;
pub use plugin::PluginBlock;
pub use pyramid::PyramidBlock;
pub use raw_input::{RawInput16Block, RawInputBlock, RawInputPackedBlock};
pub use resize::ResizeBlock;
pub use runtime_warp::RuntimeWarpBlock;
pub use saturation::SaturationBlock;
pub use sharpen::SharpenBlock;
pub use stage::StageBlock;
pub use stats::{
    CalibrationBlock, ChannelMeansBlock, CoarseHistogramBlock, ToneStatsBlock, ZoneStatsBlock,
};
pub use stereo_depth::StereoDepthBlock;
pub use super_res::SuperResBlock;
pub use temporal_denoise::TemporalDenoiseBlock;
pub use tone::ToneBlock;
pub use unpack::UnpackBlock;
pub use unpack_bayer_fp16::UnpackBayerToFp16Block;
pub use unpack_blc16::UnpackBlc16Block;
pub use unpack_cfa::{UnpackCfaBlock, UnpackMode};
pub use vignetting::VignettingBlock;
pub use warp::*;
pub use watermark::WatermarkBlock;
pub use wavelet_denoise::WaveletDenoiseBlock;

/// Register all built-in blocks with the engine registry.
/// Called by `cam_isp::init()`.
pub fn register_builtin_blocks() {
    // No engine registration needed for blocks themselves.
}

/// Build output value info for NCHW float tensors (shared helper).
#[allow(dead_code)]
pub(crate) fn nchw_value_info(name: &str, c: i64, elem_type: i32) -> Vec<u8> {
    crate::onnx::proto::Proto::value_info(
        name,
        &[
            crate::onnx::proto::Proto::tensor_dim_value(1),
            crate::onnx::proto::Proto::tensor_dim_value(c),
            crate::onnx::proto::Proto::tensor_dim_param("H"),
            crate::onnx::proto::Proto::tensor_dim_param("W"),
        ],
        elem_type,
    )
}
pub mod bayer_proc;
