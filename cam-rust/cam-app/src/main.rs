//! Cam App — ISP pipeline model generator.
//!
//! Builds the ISP pipeline and composes it into an ONNX model.
//! Does not capture real camera frames.

use log::{info, warn, error};
use clap::Parser;

use cam_isp::mnn::MnnBackend;
use cam_isp::onnx::OrtBackend;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::blocks::*;
use cam_isp::{register_mnn_engine, register_onnx_engine};

/// Command-line arguments.
#[derive(Parser, Debug)]
#[command(name = "cam-app", version, about = "ISP Pipeline Model Generator")]
struct Args {
    /// Output width for processed frames.
    #[arg(short, long, default_value_t = 1280)]
    width: u32,

    /// Backend for registration (mnn, onnx, auto).
    #[arg(short, long, default_value = "auto")]
    backend: String,
}

/// Build the 9-block ISP pipeline and compose the ONNX model.
fn build_pipeline(width: u32) -> Result<Vec<u8>, String> {
    let raw_input = RawInputBlock::new();
    let mut normalize = NormalizeBlock::new();
    let mut cfa = CfaBlock::new();
    let mut blc = BlcBlock::new();
    let mut bayer_wb = BayerWbBlock::new();
    let mut demosaic = DemosaicBlock::new(2); // GBRG
    let mut ccm = CcmBlock::new();
    let mut tone = ToneBlock::new();
    let mut display = DisplayBlock::new(width);

    // Link blocks (set input sources)
    normalize.set_input_source(raw_input.frame_tensor().unwrap_or(""));
    cfa.set_input_source(normalize.frame_tensor().unwrap_or(""));
    blc.set_input_source(cfa.frame_tensor().unwrap_or(""));
    bayer_wb.set_input_source(blc.frame_tensor().unwrap_or(""));
    demosaic.set_input_source(bayer_wb.frame_tensor().unwrap_or(""));
    ccm.set_input_source(demosaic.frame_tensor().unwrap_or(""));
    tone.set_input_source(ccm.frame_tensor().unwrap_or(""));
    display.set_input_source(tone.frame_tensor().unwrap_or(""));

    let pipeline: Vec<&dyn IspBlock> = vec![
        &raw_input, &normalize, &cfa, &blc, &bayer_wb,
        &demosaic, &ccm, &tone, &display,
    ];

    GraphComposer::compose_from_vec(&pipeline, &[], 16)
}

fn main() {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();
    info!("=== Cam App v{} ===", env!("CARGO_PKG_VERSION"));
    info!("Building pipeline with width={}", args.width);

    // Register engines
    register_mnn_engine!(MnnBackend::Vulkan);
    register_mnn_engine!(MnnBackend::Cpu);
    register_onnx_engine!(OrtBackend::Nnapi);
    register_onnx_engine!(OrtBackend::Cpu);

    // Build pipeline and compose ONNX
    match build_pipeline(args.width) {
        Ok(model_bytes) => {
            info!("Pipeline composed: {} bytes", model_bytes.len());
            let path = "isp_pipeline.onnx";
            if std::fs::write(path, &model_bytes).is_ok() {
                info!("Saved model to {}", path);
            } else {
                warn!("Could not save to disk, but model is in memory");
            }
        }
        Err(e) => {
            error!("Pipeline composition failed: {}", e);
            std::process::exit(1);
        }
    }

    info!("Cam App complete");
}
