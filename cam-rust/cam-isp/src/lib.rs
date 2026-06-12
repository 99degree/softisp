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
pub mod mnn;

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
