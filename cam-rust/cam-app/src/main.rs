//! Cam Application — Rust Camera HAL Service.
//!
//! Initializes the ISP pipeline, registers engines, creates the binder service,
//! and starts the Android Camera HAL.
//!
//! ## Usage
//! ```bash
//! # Run on host (simulated, generates ONNX model)
//! RUST_LOG=info cargo run -p cam-app -- --width 1280
//!
//! # Cross-compile for Android with binder support
//! cargo build --target aarch64-linux-android --features android -p cam-app
//! ```

use std::sync::Arc;
use log::{info, warn, error};
use clap::Parser;

use cam_types::{Frame, FrameFormat, CameraSourceType};
use cam_isp::engine::*;
use cam_isp::onnx::OrtBackend;
use cam_isp::mnn::MnnBackend;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::blocks::*;
use cam_isp::{register_mnn_engine, register_onnx_engine};
use cam_core::pipeline::PipelineConfig;
use cam_binder::provider::CameraProviderService;
use cam_binder::CAMERA_HAL_SERVICE_NAME;

/// Command-line arguments.
#[derive(Parser, Debug)]
#[command(name = "cam-app", version, about = "Rust Camera HAL Service")]
struct Args {
    /// Camera ID to use.
    #[arg(short, long, default_value = "0")]
    camera_id: String,

    /// ISP backend engine (mnn, onnx).
    #[arg(short, long, default_value = "mnn")]
    backend: String,

    /// Output width for processed frames.
    #[arg(short, long, default_value_t = 1280)]
    width: u32,

    /// Enable verbose logging.
    #[arg(short, long)]
    verbose: bool,

    /// Register as binder service (Android only).
    #[arg(long)]
    binder: bool,
}

/// Build the 9-block ISP pipeline and compose the ONNX model.
fn build_pipeline(width: u32) -> Result<Vec<u8>, String> {
    let mut raw_input = RawInputBlock::new();
    let mut normalize = NormalizeBlock::new();
    let mut cfa = CfaBlock::new();
    let mut blc = BlcBlock::new();
    let mut bayer_wb = BayerWbBlock::new();
    let mut demosaic = DemosaicBlock::new(2); // GBRG
    let mut ccm = CcmBlock::new();
    let mut tone = ToneBlock::new();
    let mut display = DisplayBlock::new(width);

    // Link input sources
    normalize.set_input_source(raw_input.frame_tensor().unwrap_or(""));
    cfa.set_input_source(normalize.frame_tensor().unwrap_or(""));
    blc.set_input_source(cfa.frame_tensor().unwrap_or(""));
    bayer_wb.set_input_source(blc.frame_tensor().unwrap_or(""));
    demosaic.set_input_source(bayer_wb.frame_tensor().unwrap_or(""));
    ccm.set_input_source(demosaic.frame_tensor().unwrap_or(""));
    tone.set_input_source(ccm.frame_tensor().unwrap_or(""));
    display.set_input_source(tone.frame_tensor().unwrap_or(""));

    let pipeline: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![
        &raw_input, &normalize, &cfa, &blc, &bayer_wb,
        &demosaic, &ccm, &tone, &display,
    ];

    GraphComposer::compose_from_vec(&pipeline, &[], 16)
}

fn main() {
    // Initialize logging
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();
    info!("=== Cam App v{} ===", env!("CARGO_PKG_VERSION"));
    info!(
        "Starting with camera={} backend={} width={} binder={}",
        args.camera_id, args.backend, args.width, args.binder
    );

    // ── Register ISP engines ──────────────────────────────────────────
    register_mnn_engine!(MnnBackend::Vulkan);
    register_mnn_engine!(MnnBackend::CpuNeon);
    register_onnx_engine!(OrtBackend::Nnapi);
    register_onnx_engine!(OrtBackend::Xnnpack);
    register_onnx_engine!(OrtBackend::Cpu);

    // ── Build and compose the ISP pipeline ────────────────────────────
    match build_pipeline(args.width) {
        Ok(model_bytes) => {
            info!("Pipeline composed: {} bytes, {} engines registered",
                model_bytes.len(), 5);

            // Save model
            let model_path = format!("/data/local/tmp/isp_pipeline_{}.onnx", args.width);
            if let Err(e) = std::fs::write(&model_path, &model_bytes) {
                warn!("Could not save to {}: {}", model_path, e);
                let _ = std::fs::write("isp_pipeline.onnx", &model_bytes);
                info!("Saved to ./isp_pipeline.onnx");
            } else {
                info!("Saved model to {}", model_path);
            }
        }
        Err(e) => {
            error!("Pipeline composition failed: {}", e);
        }
    }

    // ── Initialize camera provider service ────────────────────────────
    let provider = CameraProviderService::new();
    let cameras = provider.get_camera_id_list();
    info!("Available cameras: {:?}", cameras);

    // Register with binder if requested
    if args.binder {
        let provider = Arc::new(provider);
        match CameraProviderService::register(provider) {
            Ok(_) => info!("Camera HAL registered as '{}'", CAMERA_HAL_SERVICE_NAME),
            Err(e) => error!("Failed to register HAL: {}", e),
        }
    } else {
        // Keep provider alive for the lifetime of the app
        let _provider = std::sync::Arc::new(provider);
    }

    // ── Main loop ─────────────────────────────────────────────────────
    info!("Cam App initialized. Waiting for binder requests...");

    // Keep running
    loop {
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
