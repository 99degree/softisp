//! Cam App — ISP pipeline model generator.
//!
//! Builds the ISP pipeline and composes it into an ONNX model.
//! Optionally converts ONNX to MNN format.
//! Does not capture real camera frames.

#![allow(unused_imports)] // Some imports used only with certain features
#![allow(clippy::needless_return)] // Some returns are explicit for clarity

use clap::Parser;
use log::{error, info, warn};

use cam_isp::cpu::register_cpu_engine;
use cam_isp::engine::select_engine_by_name;
use cam_isp::pipeline::GraphComposer;
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

#[cfg(feature = "mnn")]
use cam_isp::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};
#[cfg(feature = "mnn")]
use cam_isp::mnnengine::MnnBackend;
#[cfg(feature = "ort")]
use cam_isp::onnx::OrtBackend;
#[cfg(feature = "mnn")]
use cam_isp::register_mnn_engine;
use cam_isp::register_onnx_engine;

/// Command-line arguments.
#[derive(Parser, Debug)]
#[command(name = "cam-app", version, about = "ISP Pipeline Model Generator")]
struct Args {
    /// Output width for processed frames.
    #[arg(short, long, default_value_t = 1280)]
    width: u32,

    /// Backend for pipeline (cpu, ort, mnn, auto).
    #[arg(short, long, default_value = "auto", value_parser = ["cpu", "ort", "mnn", "auto"])]
    backend: String,

    /// Pipeline profile (lite, med, heavy, pro, test).
    #[arg(long, default_value = "lite", value_parser = ["lite", "med", "heavy", "pro", "test"])]
    profile: String,

    /// Convert ONNX to MNN format after generation.
    #[arg(long, default_value_t = false)]
    convert_to_mnn: bool,

    /// Output path for MNN model.
    #[arg(long, default_value = "isp_pipeline.mnn")]
    mnn_output: String,

    /// Input element type: 1=FLOAT, 4=UINT16, 5=INT16 (only for test profile).
    #[arg(long, default_value_t = 1)]
    elem_type: i32,

    /// Use FP16 for MNN conversion.
    #[arg(long, default_value_t = false)]
    mnn_fp16: bool,

    /// Optimization level for MNN conversion (0=none, 1=safe, 2=aggressive).
    #[arg(long, default_value_t = 1)]
    mnn_optimize: u8,

    /// Generate ISP fusion pattern header file for MNN IspChainFusion.cpp.
    #[arg(long, value_name = "OUTPUT_PATH")]
    gen_isp_patterns: Option<String>,
}

/// Build the ISP pipeline for the given profile and compose the ONNX model.
fn build_pipeline(profile_name: &str, width: u32) -> Result<Vec<u8>, String> {
    let profile = match profile_name.to_lowercase().as_str() {
        "lite" => PipelineProfile::LITE,
        "med" => PipelineProfile::MED,
        "heavy" => PipelineProfile::HEAVY,
        "pro" => PipelineProfile::PRO,
        "test" => PipelineProfile::TEST,
        _ => return Err(format!("Unknown profile: {}", profile_name)),
    };
    let blocks = profile.build_blocks(width, 2);
    // build_blocks already calls wire_blocks_with_identities
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();

    info!("GraphComposer: Pipeline: {} blocks", blocks.len());
    for b in &blocks {
        info!("  {}", b.id());
    }

    GraphComposer::compose_from_vec(&block_refs, &[], 16)
}

/// Register engines based on backend selection.
fn register_engines(backend: &str) {
    match backend {
        "cpu" => {
            info!("Registering CPU engine only");
            register_cpu_engine();
        }
        "ort" => {
            #[cfg(feature = "ort")]
            {
                info!("Registering ONNX Runtime engines");
                register_onnx_engine!(OrtBackend::Nnapi);
                register_onnx_engine!(OrtBackend::Cpu);
            }
            #[cfg(not(feature = "ort"))]
            {
                warn!("ORT feature not enabled");
            }
        }
        "mnn" => {
            #[cfg(feature = "mnn")]
            {
                info!("Registering MNN engine");
                register_mnn_engine!(MnnBackend::Vulkan);
            }
            #[cfg(not(feature = "mnn"))]
            {
                warn!("MNN feature not enabled");
            }
        }
        _ => {
            info!("Registering all engines (auto mode)");
            #[cfg(feature = "mnn")]
            {
                register_mnn_engine!(MnnBackend::Vulkan);
            }
            #[cfg(feature = "ort")]
            {
                register_onnx_engine!(OrtBackend::Nnapi);
                register_onnx_engine!(OrtBackend::Cpu);
            }
        }
    }
}

fn main() {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();
    info!("=== Cam App v{} ===", env!("CARGO_PKG_VERSION"));
    info!("Backend: {}", args.backend);
    info!("Profile: {}", args.profile);
    info!("Building pipeline with width={}", args.width);

    // Generate ISP fusion pattern header if requested
    if let Some(ref path) = args.gen_isp_patterns {
        info!("Generating ISP fusion patterns header to {}", path);
        match cam_isp::mnn_opset_matcher::save_cpp_header(path) {
            Ok(()) => {
                let content = std::fs::read_to_string(path).unwrap();
                let entries = content.matches("ExactPattern(").count();
                info!("Generated header: {} ExactPattern entries", entries);
                println!("Generated {} ({} ExactPattern entries)", path, entries);
            }
            Err(e) => {
                error!("Failed to generate header: {}", e);
                std::process::exit(1);
            }
        }
        return;
    }

    // Register engines based on backend selection
    register_engines(&args.backend);

    // Select and initialize the engine
    match select_engine_by_name(&args.backend) {
        Some(engine) => {
            info!("Selected engine: {}", engine.backend_name());
            info!("Engine priority: {}", engine.priority());
        }
        None => {
            error!("No engine available for backend '{}'", args.backend);
            std::process::exit(1);
        }
    }

    // Build pipeline and compose ONNX
    let onnx_path = "isp_pipeline.onnx";
    let model_bytes = match build_pipeline(&args.profile, args.width) {
        Ok(bytes) => bytes,
        Err(e) => {
            error!("Pipeline composition failed: {}", e);
            std::process::exit(1);
        }
    };
    info!("Pipeline composed: {} bytes", model_bytes.len());
    if std::fs::write(onnx_path, &model_bytes).is_ok() {
        info!("Saved ONNX model to {}", onnx_path);
    } else {
        warn!("Could not save ONNX to disk, but model is in memory");
    }

    // Convert to MNN if requested
    if args.convert_to_mnn {
        #[cfg(feature = "mnn")]
        {
            info!("Converting ONNX to MNN...");
            let opts = MnnConvertOptions {
                fp16: args.mnn_fp16,
                optimize_level: args.mnn_optimize,
                ..Default::default()
            };

            match convert_onnx_to_mnn(onnx_path, &args.mnn_output, Some(&opts)) {
                Ok(msg) => info!("MNN conversion succeeded: {}", msg),
                Err(e) => {
                    error!("MNN conversion failed: {}", e);
                    std::process::exit(1);
                }
            }
        }
        #[cfg(not(feature = "mnn"))]
        {
            warn!("MNN feature not enabled, cannot convert");
        }
    }

    info!("Cam App complete");
}
