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
pub mod predictor;
pub mod regression;
pub mod store;
pub mod genetic;
pub mod demosaic;
pub mod isp_ops;
pub mod warp;
pub mod stats;

use std::sync::Once;

/// Initialize the ISP library — registers all built-in engines.
/// Safe to call multiple times.
pub fn init() {
    static INIT: Once = Once::new();
    INIT.call_once(|| {
        cpu::register_cpu_engine();
        blocks::register_builtin_blocks();
    });
}

// MNN FFI bindings (only compiled when `mnn` feature is enabled)
#[cfg(feature = "mnn")]
pub mod mnn_sys;
