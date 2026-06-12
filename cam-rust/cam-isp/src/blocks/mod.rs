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
pub use warp::WarpBlock;

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
