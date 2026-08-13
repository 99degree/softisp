//! ISP (Image Signal Processing) pipeline for the camera HAL.
//! Ported from com.camcore.isp

#![allow(unsafe_op_in_unsafe_fn)] // FFI wrappers require raw pointer access
#![allow(clippy::not_unsafe_ptr_arg_deref)] // FFI functions take raw pointers
#![allow(clippy::needless_range_loop)] // ISP loops are clearer with explicit indices
#![allow(clippy::too_many_arguments)] // ISP pipeline functions require all parameters
#![allow(clippy::arc_with_non_send_sync)] // Internal Arc usage for single-threaded buffers
#![allow(rustdoc::broken_intra_doc_links)] // Tensor shape [1,3,H,W] are not links
#![allow(unused_imports)] // Some imports used only with certain features
#![allow(clippy::needless_return)] // Some returns are explicit for clarity

pub mod ae;
pub mod blocks;
pub mod config;
pub mod controller;
pub mod controller_api;
pub mod controller_awb;
pub mod controller_exposure;
pub mod controller_tone;
pub mod controller_zone;
pub mod cpu;
pub mod engine;
pub mod error;
pub mod fused;
pub mod isp_controller;
pub mod isp_params;
pub mod manager;
pub mod mnn_opset_matcher;
pub mod neural_controller;
pub mod pipeline;
pub mod profile;
pub mod profile_builder;
pub mod rectifier_model;
pub mod unified_pipeline;
pub mod warp;
#[cfg(feature = "mnn")]
pub mod mnn {
    pub use super::engine::IspEngine;
    pub use super::mnn_converter;
    pub use super::mnn_sys;
    pub use super::mnnengine::*;
}
pub mod onnx;
pub mod postprocess;
// MNN-dependent modules (only compiled when `mnn` feature is enabled)
pub mod hdr;
#[cfg(feature = "mnn")]
#[cfg(feature = "mnn_buffer")]
pub mod mnn_buffer;
#[cfg(feature = "mnn")]
pub mod mnn_converter;
#[cfg(feature = "mnn")]
pub mod mnn_session_pool;
#[cfg(feature = "mnn")]
pub mod mnnengine;
pub mod warp_engine;
pub use warp_engine::GpuWarpParams;

pub mod auto_profile;
pub mod frame_rate;
pub mod pipeline_builder;
pub mod pipeline_builder_smart;
pub mod pipeline_config;

pub mod af;
pub mod calibration;
pub mod ccm_engine;
pub mod demosaic;
pub mod deshake;
pub mod eis;
pub mod format_convert;
pub mod genetic;
pub mod gpu_watchdog;
#[cfg(feature = "mnn_buffer")]
pub mod integration;
pub mod isp_ops;
pub mod r#match;
pub mod npu;
pub mod optimizer;
pub mod predictor;
pub mod regression;
pub mod rolling_stats;
pub mod scene;
pub mod serializer;
pub mod simd;
pub mod stats;
pub mod store;
pub mod synth_bayer;
pub mod temporal;

// C ABI — enables building cam-isp as a cdylib (.so) with a minimal pipeline API.
#[cfg(feature = "cabi")]
pub mod cabi;

use log::info;
use std::sync::Once;

/// Initialize the ISP library — registers all built-in engines.
/// Safe to call multiple times.
///
/// Registers CPU, MNN (if `mnn` feature), and ONNX (if `ort` feature)
/// engines. After calling `init()`, use `engine::select_engine()` or
/// `engine::select_engine_by_name()` to get an engine instance.
pub fn init() {
    static INIT: Once = Once::new();
    INIT.call_once(|| {
        info!(
            "cam_isp::init — SIMD backend: {}",
            simd::selector::active_backend_name()
        );
        cpu::register_cpu_engine();
        #[cfg(feature = "mnn")]
        {
            mnnengine::MnnEngine::register_factories();
        }
        #[cfg(feature = "ort")]
        {
            // OnnxEngine is available via engine selection
            // No explicit factory registration needed for ORT
        }
        blocks::register_builtin_blocks();
    });
}

// MNN FFI bindings (only compiled when `mnn` feature is enabled)
#[cfg(feature = "mnn")]
pub mod mnn_sys;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_init_registers_engines() {
        init();
        let engine = engine::select_engine();
        assert!(engine.is_some());
    }

    #[test]
    fn test_init_is_idempotent() {
        init();
        init(); // second call is a no-op via Once
        let e1 = engine::select_engine();
        let e2 = engine::select_engine();
        assert!(e1.is_some());
        assert!(e2.is_some());
    }
}
