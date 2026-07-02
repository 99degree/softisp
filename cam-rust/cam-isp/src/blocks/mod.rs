//! Atomic ISP blocks — each is a separate ONNX graph fragment.
//! Ported from com.camcore.isp.pipeline.processing.* in Java.
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
pub use warp::*;
pub use warp::*;
