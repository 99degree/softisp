//! ISP (Image Signal Processing) pipeline for the camera HAL.
//! Ported from com.camcore.isp

pub mod engine;
pub mod pipeline;
pub mod blocks;
pub mod cpu;
pub mod controller;
pub mod ae;
pub mod profile;
pub mod config;
pub mod fused;
pub mod manager;
pub mod onnx;
// MNN-dependent modules (only compiled when `mnn` feature is enabled)
#[cfg(feature = "mnn")]
pub mod mnnengine;
#[cfg(feature = "mnn")]
pub mod mnn_converter;
#[cfg(feature = "mnn")]
pub mod mnn_host;
#[cfg(feature = "mnn")]
#[cfg(feature = "mnn_buffer")]
pub mod mnn_buffer;

pub mod ccm_engine;
pub mod eis;
pub mod af;
pub mod calibration;
pub mod scene;
pub mod r#match;
pub mod predictor;
pub mod regression;
pub mod store;
pub mod genetic;
pub mod demosaic;
pub mod isp_ops;
pub mod warp;
pub mod stats;
pub mod simd;

use std::sync::Once;
use log::info;

/// Initialize the ISP library — registers all built-in engines.
/// Safe to call multiple times.
///
/// Registers CPU, MNN (if `mnn` feature), and ONNX (if `ort` feature)
/// engines. After calling `init()`, use `engine::select_engine()` or
/// `engine::select_engine_by_name()` to get an engine instance.
pub fn init() {
    static INIT: Once = Once::new();
    INIT.call_once(|| {
        info!("cam_isp::init — SIMD backend: {}", simd::selector::active_backend_name());
        cpu::register_cpu_engine();
        #[cfg(feature = "mnn")]
        {
            mnnengine::MnnEngine::register_factories();
        }
        #[cfg(feature = "ort")]
        {
            onnx::OnnxEngine::register_factories();
        }
        blocks::register_builtin_blocks();
    });
}

// MNN FFI bindings (only compiled when `mnn` feature is enabled)
#[cfg(feature = "mnn")]
pub mod mnn_sys;
