//! Shared logic for cam-ipa-test-cli polyglot ELF.
//!
//! Capabilities:
//!   - Generate synthetic 4K test frames (always available)
//!   - Capture real frames via V4L2 / cam-hal-linux (feature `v4l2`)
//!   - Process frames through cam-ipa IPA pipeline (MnnEngine Vulkan)
//!   - Selectable pipeline profile (LITE, MED, FUSED, HEAVY, PRO, TEST, UNIFIED, HDR, YUV)
//!   - Compiled as:
//!     - cdylib (libcam_ipa_test_cli.so) — exports `ipa_test_main` for dlopen,
//!       and `my_entry` as ELF entry point for direct execution.
//!     - included by src/main.rs for the `ipa_test_cli` binary.
//!
//! Build:
//!   cargo build -p cam-ipa-test-cli              # synthetic frames only
//!   cargo build -p cam-ipa-test-cli --features v4l2  # + V4L2 capture
//!
//! Run:
//!   ./target/debug/libcam_ipa_test_cli.so          # direct execution
//!   # or dlopen and call ipa_test_main()
//!   ./target/debug/ipa_test_cli --profile HEAVY --width 3840 --height 2160

use cam_ipa::{
    IpaConfig, IpaSettings, IpaSensorInfo, IpaFormat, IpaCallbacks,
    ControlList, MemBuffer, IpaContext,
    Size, Rectangle, Buffer, EngineBackend,
};
use cam_isp::profile::PipelineProfile;
use clap::Parser;

#[cfg(feature = "v4l2")]
use cam_hal_linux::{create_v4l2_adapter, list_v4l2_devices, capture_single_v4l2_frame};

/// CLI argument parser for the IPA test tool
#[derive(Parser, Debug)]
#[command(author, version, about = "cam-ipa test CLI — 4K frame generator + ISP pipeline")]
struct Args {
    /// Pipeline profile preset
    #[arg(long, default_value = "fused", value_enum)]
    profile: ProfileArg,

    /// Output frame width
    #[arg(long, default_value_t = 3840)]
    width: u32,

    /// Output frame height
    #[arg(long, default_value_t = 2160)]
    height: u32,

    /// Output format
    #[arg(long, default_value = "bgra", value_enum)]
    format: FormatArg,

    /// Backend engine
    #[arg(long, default_value = "mnn_vulkan", value_enum)]
    backend: BackendArg,

    /// Enable neural controller (isp-rectifier)
    #[arg(long)]
    neural: bool,

    /// Model directory for neural processor
    #[arg(long)]
    model_dir: Option<String>,
}

/// Profile argument enum for clap
#[derive(clap::ValueEnum, Clone, Debug)]
enum ProfileArg {
    Lite, Med, Fused, Heavy, Pro, Test, Unified, Hdr, Yuv,
}

/// Format argument enum for clap
#[derive(clap::ValueEnum, Clone, Debug)]
enum FormatArg {
    Rgb, Rgba, Bgra, Argb, Abgr, Yuv, Nv12, P010,
}

/// Backend argument enum for clap
#[derive(clap::ValueEnum, Clone, Debug)]
enum BackendArg {
    MnnVulkan, Cpu, OnnxRuntime,
}

impl From<BackendArg> for EngineBackend {
    fn from(arg: BackendArg) -> Self {
        match arg {
            BackendArg::MnnVulkan => EngineBackend::MnnVulkan,
            BackendArg::Cpu => EngineBackend::Cpu,
            BackendArg::OnnxRuntime => EngineBackend::OnnxRuntime,
        }
    }
}

/// Library entry point — callable via dlopen
#[no_mangle]
pub extern "C" fn ipa_test_main() -> i32 {
    run_cli()
}

/// ELF entry point — allows the cdylib to be executed directly.
/// Called by the dynamic linker when the .so is run as ./libcam_ipa_test_cli.so
#[no_mangle]
pub extern "C" fn my_entry() -> ! {
    std::process::exit(run_cli())
}

fn run_cli() -> i32 {
    let args = Args::parse();
    env_logger::init();

    println!("=== cam-ipa Test CLI (polyglot ELF) ===");
    println!("Features: {}", feature_list());
    println!("Profile: {:?} ({:?})", args.profile, std::any::type_name::<PipelineProfile>());
    println!("Resolution: {}x{}", args.width, args.height);
    println!("Format: {:?}", args.format);
    println!("Backend: {:?}", args.backend);
    println!("Neural: {}", args.neural);
    if let Some(dir) = &args.model_dir {
        println!("Model dir: {}", dir);
    }

    // 1. Create config with selected profile
    let profile = match args.profile {
        ProfileArg::Lite => PipelineProfile::LITE,
        ProfileArg::Med => PipelineProfile::MED,
        ProfileArg::Fused => PipelineProfile::FUSED,
        ProfileArg::Heavy => PipelineProfile::HEAVY,
        ProfileArg::Pro => PipelineProfile::PRO,
        ProfileArg::Test => PipelineProfile::TEST,
        ProfileArg::Unified => PipelineProfile::UNIFIED,
        ProfileArg::Hdr => PipelineProfile::HDR,
        ProfileArg::Yuv => PipelineProfile::YUV,
    };

    let config = IpaConfig {
        image_width: args.width,
        image_height: args.height,
        format: match args.format {
            FormatArg::Rgb => IpaFormat::RGB,
            FormatArg::Rgba => IpaFormat::RGBA,
            FormatArg::Bgra => IpaFormat::BGRA,
            FormatArg::Argb => IpaFormat::ARGB,
            FormatArg::Abgr => IpaFormat::ABGR,
            FormatArg::Yuv => IpaFormat::YUV,
            FormatArg::Nv12 => IpaFormat::NV12,
            FormatArg::P010 => IpaFormat::P010,
        },
        num_output_buffers: 2,
        backend: match args.backend {
            BackendArg::MnnVulkan => EngineBackend::MnnVulkan,
            BackendArg::Cpu => EngineBackend::Cpu,
            BackendArg::OnnxRuntime => EngineBackend::OnnxRuntime,
        },
        profile,
        neural_enabled: args.neural,
        model_dir: args.model_dir.clone().map(std::path::PathBuf::from),
        ..IpaConfig::default()
    };
    let config_for_acquire = config.clone();
    println!("Config: {}x{} {:?}", config.image_width, config.image_height, config.format);

    // 2. Create IPA context
    let mut ctx = match IpaContext::new(config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create IPA context: {}", e);
            return 1;
        }
    };
    println!("IPA context created");

    // 3. Set up callbacks
    let callbacks = IpaCallbacks {
        metadata_ready: Some(Box::new(|frame, metadata| {
            println!("[callback] metadata_ready: frame={} keys={:?}",
                frame, metadata.data.keys().collect::<Vec<_>>());
        })),
        frame_done: Some(Box::new(|frame, buffer_id| {
            println!("[callback] frame_done: frame={} buffer_id={}", frame, buffer_id);
        })),
        set_sensor_controls: Some(Box::new(|controls| {
            println!("[callback] set_sensor_controls: keys={:?}",
                controls.data.keys().collect::<Vec<_>>());
        })),
        set_isp_params: Some(Box::new(|| {
            println!("[callback] set_isp_params");
        })),
    };
    ctx.set_callbacks(callbacks);
    println!("Callbacks registered");

    // 4. Create sensor info
    let sensor_info = IpaSensorInfo {
        model: "test_sensor".to_string(),
        bits_per_pixel: 10,
        cfa_pattern: 0,
        active_area_size: Size { width: args.width, height: args.height },
        analog_crop: Rectangle { x: 0, y: 0, width: args.width, height: args.height },
        output_size: Size { width: args.width, height: args.height },
        pixel_rate: 0,
        min_line_length: 0,
        max_line_length: 0,
        min_frame_length: 0,
        max_frame_length: 0,
    };

    // 5. Initialize
    let settings = IpaSettings {
        config_file: None,
        sensor_model: "test_sensor".to_string(),
    };
    match ctx.init(&settings, &sensor_info) {
        Ok(init_result) => {
            println!("IPA init OK: ccm_enabled={}", init_result.ccm_enabled);
        }
        Err(e) => {
            eprintln!("IPA init failed: {}", e);
            return 1;
        }
    }
    println!("Initialized: {}", ctx.is_initialized());

    // 6. Start
    match ctx.start() {
        Ok(()) => println!("IPA started"),
        Err(e) => {
            eprintln!("IPA start failed: {}", e);
            return 1;
        }
    }
    println!("Started: {}", ctx.is_started());

    // 7. Acquire frame input — V4L2 if available, otherwise synthetic
    let (width, height, input_data) = acquire_frame(&config_for_acquire);
    let mut input_buf = MemBuffer::new(input_data.len());
    input_buf.data.copy_from_slice(&input_data);

    let output_size = (width * height * 4) as usize; // BGRA
    let mut output_buf = MemBuffer::new(output_size);

    // 8. Queue request
    let mut controls = ControlList::new();
    controls.set_f32("AnalogueGain", 2.0);
    controls.set_i32("ExposureTime", 2000);
    controls.set_f32_array("ColourGains", &[1.5, 1.2]);

    match ctx.queue_request(1, &controls) {
        Ok(()) => println!("queue_request(1) OK"),
        Err(e) => eprintln!("queue_request failed: {}", e),
    }

    // 9. Compute params
    match ctx.compute_params(1) {
        Ok(()) => println!("compute_params(1) OK"),
        Err(e) => eprintln!("compute_params failed: {}", e),
    }

    // 10. Process stats
    let stats_buf = MemBuffer::new(((width / 4) * (height / 4) * 4) as usize);
    match ctx.process_stats(1, 100, &stats_buf) {
        Ok(metadata) => {
            println!("process_stats OK: keys={:?}",
                metadata.data.keys().collect::<Vec<_>>());
        }
        Err(e) => eprintln!("process_stats failed: {}", e),
    }

    // 11. Process frame through full ISP pipeline
    match ctx.process_frame(
        1, 100,
        &mut input_buf, &mut output_buf,
        0, 0,
        width, height,
        &controls,
    ) {
        Ok(()) => {
            println!("process_frame(1) OK — output {} bytes", output_buf.len());
            // Write output to file for inspection
            if let Err(e) = std::fs::write("/data/data/com.termux/files/home/softisp/output_frame.bgra", &output_buf.data) {
                eprintln!("Failed to write output frame: {}", e);
            } else {
                println!("Output frame written to output_frame.bgra ({} bytes)", output_buf.data.len());
            }
        }
        Err(e) => eprintln!("process_frame failed: {}", e),
    }

    // 12. Configure frame backend
    match ctx.configure_frame_backend(IpaFormat::RGBA, 3) {
        Ok(()) => println!("configure_frame_backend(RGBA, 3) OK"),
        Err(e) => eprintln!("configure_frame_backend failed: {}", e),
    }

    // 13. Stop
    ctx.stop();
    println!("Stopped");

    println!("=== cam-ipa Test CLI Complete ===");
    0
}

/// Acquire a frame: V4L2 if feature enabled and device available, otherwise synthetic.
fn acquire_frame(config: &IpaConfig) -> (u32, u32, Vec<u8>) {
    #[cfg(feature = "v4l2")]
    {
        // Try V4L2 first
        let devices = list_v4l2_devices();
        if let Some(dev) = devices.first() {
            println!("V4L2 device found: {}", dev);
            match capture_single_v4l2_frame(dev, config.image_width, config.image_height) {
                Ok((w, h, data)) => {
                    println!("Captured {}x{} frame from V4L2 ({} bytes)", w, h, data.len());
                    return (w, h, data);
                }
                Err(e) => {
                    println!("V4L2 capture failed: {}, falling back to synthetic", e);
                }
            }
        } else {
            println!("No V4L2 device found, using synthetic frame");
        }
    }
    #[cfg(not(feature = "v4l2"))]
    {
        println!("V4L2 feature not enabled, using synthetic frame");
    }

    // Synthetic 4K Bayer-like pattern
    let w = config.image_width;
    let h = config.image_height;
    let size = (w * h * 2) as usize; // 16-bit Bayer
    let mut data = vec![0u8; size];
    for (i, chunk) in data.chunks_mut(2).enumerate() {
        let val = (i as u16) ^ ((i >> 11) as u16); // spatial pattern
        chunk[0] = val as u8;
        chunk[1] = (val >> 8) as u8;
    }
    println!("Generated synthetic {}x{} 16-bit Bayer frame ({} bytes)", w, h, data.len());
    (w, h, data)
}

/// Return a comma-separated list of active features for the banner.
fn feature_list() -> String {
    let mut feats = Vec::new();
    feats.push("synthetic-4k".to_string());
    #[cfg(feature = "v4l2")]
    feats.push("v4l2".to_string());
    feats.join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_profile_arg_conversion() {
        for arg in [
            ProfileArg::Lite, ProfileArg::Med, ProfileArg::Fused,
            ProfileArg::Heavy, ProfileArg::Pro, ProfileArg::Test,
            ProfileArg::Unified, ProfileArg::Hdr, ProfileArg::Yuv,
        ] {
            let _profile: PipelineProfile = arg.into();
        }
    }

    #[test]
    fn test_format_arg_conversion() {
        for arg in [
            FormatArg::Rgb, FormatArg::Rgba, FormatArg::Bgra,
            FormatArg::Argb, FormatArg::Abgr, FormatArg::Yuv,
            FormatArg::Nv12, FormatArg::P010,
        ] {
            let _fmt: IpaFormat = arg.into();
        }
    }

    #[test]
    fn test_backend_arg_conversion() {
        for arg in [BackendArg::MnnVulkan, BackendArg::Cpu, BackendArg::OnnxRuntime] {
            let _backend: EngineBackend = arg.into();
        }
    }
}