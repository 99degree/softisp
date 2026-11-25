//! cam-ipa Swiss Army Knife CLI — polyglot ELF for softisp
//!
//! One binary that does most of the softisp IPA jobs:
//!   - `generate`   — synthetic test frame generator (Bayer 16-bit, configurable res)
//!   - `capture`    — V4L2 frame capture via cam-hal-linux (feature `v4l2`)
//!   - `process`    — run a frame through the full ISP pipeline (3A + neural)
//!   - `bench`      — FPS/latency benchmark over N frames
//!   - `stats`      — collect per-frame stats for CMOS sensor characterization
//!   - `zerocopy`   — test memfd/dmabuf zero-copy buffer path
//!   - `probe`      — probe profile capabilities and engine info
//!
//! Compiled as:
//!   - cdylib (libcam_ipa_sk.so) — exports `ipa_sk_main` for dlopen,
//!     and `my_entry` as ELF entry point for direct execution.
//!   - included by src/main.rs for the `ipa_sk` binary.
//!
//! Build:
//!   cargo build -p cam-ipa-sk                    # synthetic only
//!   cargo build -p cam-ipa-sk --features v4l2    # + V4L2 capture
//!
//! Run:
//!   ./target/debug/libcam_ipa_sk.so --help       # direct execution (polyglot)
//!   ./target/debug/ipa_sk --help                 # binary wrapper
//!   ./target/debug/ipa_sk generate --count 1 --profile fused --width 3840 --height 2160

use cam_ipa::{
    IpaConfig, IpaSettings, IpaSensorInfo, IpaFormat, IpaCallbacks,
    ControlList, MemBuffer, IpaContext, Buffer, EngineBackend,
    Size, Rectangle,
};
use cam_isp::profile::PipelineProfile;
use clap::{Parser, Subcommand, ValueEnum};
use std::path::PathBuf;
use std::time::Instant;

#[cfg(feature = "v4l2")]
use cam_hal_linux::{list_v4l2_devices, capture_single_v4l2_frame};

/// Top-level CLI argument parser
#[derive(Parser, Debug)]
#[command(author, version, about = "cam-ipa Swiss Army Knife — softisp IPA toolkit", subcommand_required = true)]
struct Args {
    #[command(subcommand)]
    command: Commands,
}

/// Profile argument enum
#[derive(ValueEnum, Clone, Debug, Copy)]
enum ProfileArg {
    Lite, Med, Fused, Heavy, Pro, Test, Unified, Hdr, Yuv,
}

/// Format argument enum
#[derive(ValueEnum, Clone, Debug, Copy)]
enum FormatArg {
    Rgb, Rgba, Bgra, Argb, Abgr, Yuv, Nv12, P010,
}

/// Backend argument enum
#[derive(ValueEnum, Clone, Debug, Copy)]
enum BackendArg {
    MnnVulkan, Cpu, OnnxRuntime,
}

impl From<ProfileArg> for PipelineProfile {
    fn from(arg: ProfileArg) -> Self {
        match arg {
            ProfileArg::Lite => PipelineProfile::LITE,
            ProfileArg::Med => PipelineProfile::MED,
            ProfileArg::Fused => PipelineProfile::FUSED,
            ProfileArg::Heavy => PipelineProfile::HEAVY,
            ProfileArg::Pro => PipelineProfile::PRO,
            ProfileArg::Test => PipelineProfile::TEST,
            ProfileArg::Unified => PipelineProfile::UNIFIED,
            ProfileArg::Hdr => PipelineProfile::HDR,
            ProfileArg::Yuv => PipelineProfile::YUV,
        }
    }
}

impl From<FormatArg> for IpaFormat {
    fn from(arg: FormatArg) -> Self {
        match arg {
            FormatArg::Rgb => IpaFormat::RGB,
            FormatArg::Rgba => IpaFormat::RGBA,
            FormatArg::Bgra => IpaFormat::BGRA,
            FormatArg::Argb => IpaFormat::ARGB,
            FormatArg::Abgr => IpaFormat::ABGR,
            FormatArg::Yuv => IpaFormat::YUV,
            FormatArg::Nv12 => IpaFormat::NV12,
            FormatArg::P010 => IpaFormat::P010,
        }
    }
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

/// Subcommands
#[derive(Subcommand, Debug)]
enum Commands {
    /// Generate synthetic test frames and process through ISP
    Generate {
        /// Number of frames to generate
        #[arg(short, long, default_value_t = 1)]
        count: u32,
        /// Write output frame to file
        #[arg(short, long)]
        output: Option<PathBuf>,
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
        #[arg(long, default_value = "mnn-vulkan", value_enum)]
        backend: BackendArg,
        /// Enable neural controller (isp-rectifier)
        #[arg(long)]
        neural: bool,
        /// Model directory for neural processor
        #[arg(long)]
        model_dir: Option<String>,
    },
    /// Capture real frames from V4L2 device and process
    Capture {
        /// V4L2 device path (e.g., /dev/video0)
        #[arg(long, default_value = "/dev/video0")]
        device: String,
        /// Number of frames to capture
        #[arg(short, long, default_value_t = 1)]
        count: u32,
        /// Write output frame to file
        #[arg(short, long)]
        output: Option<PathBuf>,
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
        #[arg(long, default_value = "mnn-vulkan", value_enum)]
        backend: BackendArg,
        /// Enable neural controller (isp-rectifier)
        #[arg(long)]
        neural: bool,
        /// Model directory for neural processor
        #[arg(long)]
        model_dir: Option<String>,
    },
    /// Benchmark FPS/latency over N frames
    Bench {
        /// Number of frames
        #[arg(short, long, default_value_t = 30)]
        count: u32,
        /// Warmup frames (not timed)
        #[arg(long, default_value_t = 5)]
        warmup: u32,
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
        #[arg(long, default_value = "mnn-vulkan", value_enum)]
        backend: BackendArg,
        /// Enable neural controller (isp-rectifier)
        #[arg(long)]
        neural: bool,
        /// Model directory for neural processor
        #[arg(long)]
        model_dir: Option<String>,
    },
    /// Collect per-frame stats for CMOS sensor characterization
    Stats {
        /// Number of frames
        #[arg(short, long, default_value_t = 100)]
        count: u32,
        /// Write stats to JSON file
        #[arg(short, long)]
        output: Option<PathBuf>,
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
        #[arg(long, default_value = "mnn-vulkan", value_enum)]
        backend: BackendArg,
        /// Enable neural controller (isp-rectifier)
        #[arg(long)]
        neural: bool,
        /// Model directory for neural processor
        #[arg(long)]
        model_dir: Option<String>,
    },
    /// Test zero-copy buffer path (memfd on Linux, dmabuf on Android)
    Zerocopy {
        /// Buffer size in bytes
        #[arg(long, default_value_t = 1048576)]
        size: usize,
        /// Number of write/read iterations
        #[arg(long, default_value_t = 1000)]
        iterations: u32,
    },
    /// Probe profile capabilities and engine info
    Probe,
}

/// Build an IpaConfig from common options
fn build_ipa_config(
    profile: ProfileArg,
    width: u32,
    height: u32,
    format: FormatArg,
    backend: BackendArg,
    neural: bool,
    model_dir: Option<String>,
) -> IpaConfig {
    IpaConfig {
        image_width: width,
        image_height: height,
        format: format.into(),
        num_output_buffers: 2,
        backend: backend.into(),
        profile: profile.into(),
        neural_enabled: neural,
        model_dir: model_dir.map(PathBuf::from),
        ..IpaConfig::default()
    }
}

/// Library entry point — callable via dlopen
#[no_mangle]
pub extern "C" fn ipa_sk_main() -> i32 {
    run_cli()
}

/// ELF entry point — allows the cdylib to be executed directly
#[no_mangle]
pub extern "C" fn my_entry() -> ! {
    std::process::exit(run_cli())
}

fn run_cli() -> i32 {
    let args = Args::parse();
    env_logger::init();

    match args.command {
        Commands::Generate { count, output, profile, width, height, format, backend, neural, model_dir } => {
            println!("=== cam-ipa Swiss Army Knife ===");
            println!("Command: generate");
            println!("Profile: {:?}  Res: {}x{}  Format: {:?}  Backend: {:?}  Neural: {}",
                profile, width, height, format, backend, neural);
            let config = build_ipa_config(profile, width, height, format, backend, neural, model_dir);
            cmd_generate(&config, count, output)
        }
        Commands::Capture { device, count, output, profile, width, height, format, backend, neural, model_dir } => {
            println!("=== cam-ipa Swiss Army Knife ===");
            println!("Command: capture");
            println!("Profile: {:?}  Res: {}x{}  Format: {:?}  Backend: {:?}  Neural: {}",
                profile, width, height, format, backend, neural);
            let config = build_ipa_config(profile, width, height, format, backend, neural, model_dir);
            cmd_capture(&config, device, count, output)
        }
        Commands::Bench { count, warmup, profile, width, height, format, backend, neural, model_dir } => {
            println!("=== cam-ipa Swiss Army Knife ===");
            println!("Command: bench");
            println!("Profile: {:?}  Res: {}x{}  Format: {:?}  Backend: {:?}  Neural: {}",
                profile, width, height, format, backend, neural);
            let config = build_ipa_config(profile, width, height, format, backend, neural, model_dir);
            cmd_bench(&config, count, warmup)
        }
        Commands::Stats { count, output, profile, width, height, format, backend, neural, model_dir } => {
            println!("=== cam-ipa Swiss Army Knife ===");
            println!("Command: stats");
            println!("Profile: {:?}  Res: {}x{}  Format: {:?}  Backend: {:?}  Neural: {}",
                profile, width, height, format, backend, neural);
            let config = build_ipa_config(profile, width, height, format, backend, neural, model_dir);
            cmd_stats(&config, count, output)
        }
        Commands::Zerocopy { size, iterations } => {
            cmd_zerocopy(size, iterations)
        }
        Commands::Probe => cmd_probe(),
    }
}

/// cmd_generate — synthetic frame generation + ISP processing
fn cmd_generate(config: &IpaConfig, count: u32, output: Option<PathBuf>) -> i32 {
    println!("\n--- Generate mode: {} synthetic frames ---", count);

    let mut ctx = match IpaContext::new(config.clone()) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create IPA context: {}", e);
            return 1;
        }
    };

    let sensor_info = IpaSensorInfo {
        model: "synthetic_sensor".to_string(),
        bits_per_pixel: 10,
        cfa_pattern: 0,
        active_area_size: Size { width: config.image_width, height: config.image_height },
        analog_crop: Rectangle { x: 0, y: 0, width: config.image_width, height: config.image_height },
        output_size: Size { width: config.image_width, height: config.image_height },
        pixel_rate: 0,
        min_line_length: 0,
        max_line_length: 0,
        min_frame_length: 0,
        max_frame_length: 0,
    };

    let settings = IpaSettings {
        config_file: None,
        sensor_model: "synthetic_sensor".to_string(),
    };
    if let Err(e) = ctx.init(&settings, &sensor_info) {
        eprintln!("IPA init failed: {}", e);
        return 1;
    }
    if let Err(e) = ctx.start() {
        eprintln!("IPA start failed: {}", e);
        return 1;
    }

    let w = config.image_width;
    let h = config.image_height;
    let input_size = (w * h * 2) as usize;

    for frame in 1..=count {
        // Generate synthetic Bayer frame
        let mut input_data = vec![0u8; input_size];
        for (i, chunk) in input_data.chunks_mut(2).enumerate() {
            let val = (i as u16).wrapping_add((frame as u16).wrapping_mul(7));
            chunk[0] = val as u8;
            chunk[1] = (val >> 8) as u8;
        }

        let mut input_buf = MemBuffer::new(input_data.len());
        input_buf.data.copy_from_slice(&input_data);

        let output_size = (w * h * 4) as usize;
        let mut output_buf = MemBuffer::new(output_size);

        let mut controls = ControlList::new();
        controls.set_f32("AnalogueGain", 2.0);
        controls.set_i32("ExposureTime", 2000);
        controls.set_f32_array("ColourGains", &[1.5, 1.2]);

        if let Err(e) = ctx.queue_request(frame, &controls) {
            eprintln!("queue_request(frame {}) failed: {}", frame, e);
            continue;
        }
        if let Err(e) = ctx.compute_params(frame) {
            eprintln!("compute_params(frame {}) failed: {}", frame, e);
            continue;
        }

        let stats_buf = MemBuffer::new(((w / 4) * (h / 4) * 4) as usize);
        if let Err(e) = ctx.process_stats(frame, frame, &stats_buf) {
            eprintln!("process_stats(frame {}) failed: {}", frame, e);
            continue;
        }

        if let Err(e) = ctx.process_frame(
            frame, frame,
            &mut input_buf, &mut output_buf,
            0, 0, w, h, &controls,
        ) {
            eprintln!("process_frame(frame {}) failed: {}", frame, e);
            continue;
        }

        if let Some(ref path) = output {
            if let Err(e) = std::fs::write(path, &output_buf.data) {
                eprintln!("Failed to write output frame: {}", e);
            } else {
                println!("Frame {} written to {} ({} bytes)", frame, path.display(), output_buf.data.len());
            }
        } else {
            println!("Frame {} processed: {} bytes output", frame, output_buf.len());
        }
    }

    ctx.stop();
    println!("Generate complete");
    0
}

/// cmd_capture — V4L2 capture + ISP processing
fn cmd_capture(config: &IpaConfig, device: String, count: u32, output: Option<PathBuf>) -> i32 {
    println!("\n--- Capture mode: {} frames from {} ---", count, device);

    #[cfg(feature = "v4l2")]
    {
        let devices = cam_hal_linux::list_v4l2_devices();
        if !devices.iter().any(|d| d == &device) {
            eprintln!("V4L2 device {} not found. Available: {:?}", device, devices);
            return 1;
        }

        let mut ctx = match IpaContext::new(config.clone()) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Failed to create IPA context: {}", e);
                return 1;
            }
        };

        let sensor_info = IpaSensorInfo {
            model: "v4l2_sensor".to_string(),
            bits_per_pixel: 10,
            cfa_pattern: 0,
            active_area_size: Size { width: config.image_width, height: config.image_height },
            analog_crop: Rectangle { x: 0, y: 0, width: config.image_width, height: config.image_height },
            output_size: Size { width: config.image_width, height: config.image_height },
            pixel_rate: 0,
            min_line_length: 0,
            max_line_length: 0,
            min_frame_length: 0,
            max_frame_length: 0,
        };

        let settings = IpaSettings {
            config_file: None,
            sensor_model: "v4l2_sensor".to_string(),
        };
        if let Err(e) = ctx.init(&settings, &sensor_info) {
            eprintln!("IPA init failed: {}", e);
            return 1;
        }
        if let Err(e) = ctx.start() {
            eprintln!("IPA start failed: {}", e);
            return 1;
        }

        let w = config.image_width;
        let h = config.image_height;

        for frame in 1..=count {
            match cam_hal_linux::capture_single_v4l2_frame(&device, w, h) {
                Ok((cw, ch, data)) => {
                    println!("Captured {}x{} frame {} ({} bytes)", cw, ch, frame, data.len());

                    let mut input_buf = MemBuffer::new(data.len());
                    input_buf.data.copy_from_slice(&data);

                    let output_size = (cw * ch * 4) as usize;
                    let mut output_buf = MemBuffer::new(output_size);

                    let mut controls = ControlList::new();
                    controls.set_f32("AnalogueGain", 2.0);
                    controls.set_i32("ExposureTime", 2000);

                    if let Err(e) = ctx.queue_request(frame, &controls) {
                        eprintln!("queue_request(frame {}) failed: {}", frame, e);
                        continue;
                    }
                    if let Err(e) = ctx.process_frame(
                        frame, frame,
                        &mut input_buf, &mut output_buf,
                        0, 0, cw, ch, &controls,
                    ) {
                        eprintln!("process_frame(frame {}) failed: {}", frame, e);
                        continue;
                    }

                    if let Some(ref path) = output {
                        let fname = format!("{}_{}.bgra", path.display(), frame);
                        if let Err(e) = std::fs::write(&fname, &output_buf.data) {
                            eprintln!("Failed to write {}: {}", fname, e);
                        } else {
                            println!("Frame {} written to {} ({} bytes)", frame, fname, output_buf.data.len());
                        }
                    }
                }
                Err(e) => {
                    eprintln!("V4L2 capture frame {} failed: {}", frame, e);
                }
            }
        }

        ctx.stop();
        println!("Capture complete");
        0
    }
    #[cfg(not(feature = "v4l2"))]
    {
        eprintln!("V4L2 feature not enabled. Rebuild with: cargo build -p cam-ipa-sk --features v4l2");
        1
    }
}

/// cmd_bench — FPS/latency benchmark
fn cmd_bench(config: &IpaConfig, count: u32, warmup: u32) -> i32 {
    println!("\n--- Bench mode: {} frames ({} warmup) ---", count, warmup);

    let mut ctx = match IpaContext::new(config.clone()) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create IPA context: {}", e);
            return 1;
        }
    };

    let sensor_info = IpaSensorInfo {
        model: "bench_sensor".to_string(),
        bits_per_pixel: 10,
        cfa_pattern: 0,
        active_area_size: Size { width: config.image_width, height: config.image_height },
        analog_crop: Rectangle { x: 0, y: 0, width: config.image_width, height: config.image_height },
        output_size: Size { width: config.image_width, height: config.image_height },
        pixel_rate: 0,
        min_line_length: 0,
        max_line_length: 0,
        min_frame_length: 0,
        max_frame_length: 0,
    };

    let settings = IpaSettings {
        config_file: None,
        sensor_model: "bench_sensor".to_string(),
    };
    if let Err(e) = ctx.init(&settings, &sensor_info) {
        eprintln!("IPA init failed: {}", e);
        return 1;
    }
    if let Err(e) = ctx.start() {
        eprintln!("IPA start failed: {}", e);
        return 1;
    }

    let w = config.image_width;
    let h = config.image_height;
    let input_size = (w * h * 2) as usize;
    let output_size = (w * h * 4) as usize;

    // Pre-allocate buffers
    let mut input_buf = MemBuffer::new(input_size);
    for (i, b) in input_buf.data.iter_mut().enumerate() {
        *b = (i % 256) as u8;
    }
    let mut output_buf = MemBuffer::new(output_size);

    let mut controls = ControlList::new();
    controls.set_f32("AnalogueGain", 2.0);
    controls.set_i32("ExposureTime", 2000);
    controls.set_f32_array("ColourGains", &[1.5, 1.2]);

    let mut latencies: Vec<u64> = Vec::new();
    let total_start = Instant::now();

    for frame in 1..=(warmup + count) {
        let frame_start = Instant::now();

        if let Err(e) = ctx.process_frame(
            frame, frame,
            &mut input_buf, &mut output_buf,
            0, 0, w, h, &controls,
        ) {
            eprintln!("process_frame(frame {}) failed: {}", frame, e);
            continue;
        }

        let frame_elapsed = frame_start.elapsed().as_micros() as u64;
        if frame > warmup {
            latencies.push(frame_elapsed);
        }

        if frame == warmup {
            println!("Warmup complete, starting timed run...");
        }
        if frame % 10 == 0 || frame <= warmup + 5 {
            println!("Frame {}: {} us", frame, frame_elapsed);
        }
    }

    let total_elapsed = total_start.elapsed();
    let timed_count = latencies.len() as u64;

    if timed_count > 0 {
        let avg_us: u64 = latencies.iter().sum::<u64>() / timed_count;
        let max_us = latencies.iter().max().copied().unwrap_or(0);
        let min_us = latencies.iter().min().copied().unwrap_or(0);
        let fps = (timed_count as f64) / (total_elapsed.as_secs_f64() - (warmup as f64 * avg_us as f64 / 1_000_000.0));

        println!("\n--- Bench results ---");
        println!("Timed frames: {}", timed_count);
        println!("Avg latency: {} us", avg_us);
        println!("Min latency: {} us", min_us);
        println!("Max latency: {} us", max_us);
        println!("Approx FPS: {:.2}", fps);
        println!("Resolution: {}x{}", w, h);
        println!("Profile: {:?}", config.profile);
    }

    ctx.stop();
    0
}

/// cmd_stats — collect per-frame stats for CMOS characterization
fn cmd_stats(config: &IpaConfig, count: u32, output: Option<PathBuf>) -> i32 {
    println!("\n--- Stats mode: {} frames ---", count);

    let mut ctx = match IpaContext::new(config.clone()) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create IPA context: {}", e);
            return 1;
        }
    };

    let sensor_info = IpaSensorInfo {
        model: "stats_sensor".to_string(),
        bits_per_pixel: 10,
        cfa_pattern: 0,
        active_area_size: Size { width: config.image_width, height: config.image_height },
        analog_crop: Rectangle { x: 0, y: 0, width: config.image_width, height: config.image_height },
        output_size: Size { width: config.image_width, height: config.image_height },
        pixel_rate: 0,
        min_line_length: 0,
        max_line_length: 0,
        min_frame_length: 0,
        max_frame_length: 0,
    };

    let settings = IpaSettings {
        config_file: None,
        sensor_model: "stats_sensor".to_string(),
    };
    if let Err(e) = ctx.init(&settings, &sensor_info) {
        eprintln!("IPA init failed: {}", e);
        return 1;
    }
    if let Err(e) = ctx.start() {
        eprintln!("IPA start failed: {}", e);
        return 1;
    }

    let w = config.image_width;
    let h = config.image_height;
    let input_size = (w * h * 2) as usize;

    let mut all_stats: Vec<serde_json::Value> = Vec::new();

    for frame in 1..=count {
        let mut input_data = vec![0u8; input_size];
        for (i, chunk) in input_data.chunks_mut(2).enumerate() {
            let val = (i as u16).wrapping_add((frame as u16).wrapping_mul(13));
            chunk[0] = val as u8;
            chunk[1] = (val >> 8) as u8;
        }

        let mut input_buf = MemBuffer::new(input_data.len());
        input_buf.data.copy_from_slice(&input_data);

        let mut output_buf = MemBuffer::new((w * h * 4) as usize);

        let mut controls = ControlList::new();
        controls.set_f32("AnalogueGain", 2.0);
        controls.set_i32("ExposureTime", 2000);
        controls.set_f32_array("ColourGains", &[1.5, 1.2]);

        if let Err(e) = ctx.queue_request(frame, &controls) {
            eprintln!("queue_request(frame {}) failed: {}", frame, e);
            continue;
        }
        if let Err(e) = ctx.compute_params(frame) {
            eprintln!("compute_params(frame {}) failed: {}", frame, e);
            continue;
        }

        let stats_buf = MemBuffer::new(((w / 4) * (h / 4) * 4) as usize);
        match ctx.process_stats(frame, frame, &stats_buf) {
            Ok(metadata) => {
                let stats_entry = serde_json::json!({
                    "frame": frame,
                    "width": w,
                    "height": h,
                    "metadata": metadata.data,
                });
                all_stats.push(stats_entry);
                if frame % 10 == 0 {
                    println!("Frame {} stats collected", frame);
                }
            }
            Err(e) => {
                eprintln!("process_stats(frame {}) failed: {}", frame, e);
            }
        }
    }

    ctx.stop();

    if let Some(ref path) = output {
        let json = serde_json::to_string_pretty(&all_stats).unwrap_or_default();
        if let Err(e) = std::fs::write(path, &json) {
            eprintln!("Failed to write stats to {}: {}", path.display(), e);
            return 1;
        }
        println!("Stats written to {} ({} frames)", path.display(), all_stats.len());
    } else {
        println!("Collected stats from {} frames", all_stats.len());
    }

    println!("Stats complete");
    0
}

/// cmd_zerocopy — test memfd/dmabuf zero-copy buffer path
fn cmd_zerocopy(size: usize, iterations: u32) -> i32 {
    println!("\n--- Zerocopy mode: {} iterations, {} bytes ---", iterations, size);

    #[cfg(not(target_os = "android"))]
    {
        use cam_ipa::buffer::create_memfd_dmabuf;
        use std::os::fd::AsRawFd;

        let (fd, mut buf) = match create_memfd_dmabuf(size) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("Failed to create memfd dmabuf: {}", e);
                return 1;
            }
        };
        println!("Created memfd dmabuf: fd={} size={} ptr={:?}", fd, buf.len(), buf.as_ptr());

        let mut write_sum: u8 = 0;
        let start = Instant::now();

        for i in 0..iterations {
            // Write pattern through mmap'd pointer
            let val = (i % 256) as u8;
            for chunk in buf.data.chunks_mut(8) {
                for b in chunk.iter_mut() {
                    *b = val;
                }
            }
            write_sum = write_sum.wrapping_add(val);

            // Read back
            let read_val = unsafe { buf.as_ptr().read() };
            if read_val != val {
                eprintln!("Zerocopy mismatch at iter {}: wrote {} read {}", i, val, read_val);
                return 1;
            }
        }

        let elapsed = start.elapsed();
        let per_iter = elapsed.as_nanos() / iterations as u128;

        println!("Zerocopy test: {} iterations in {:?}", iterations, elapsed);
        println!("Per iteration: {} ns", per_iter);
        println!("Write sum: {}", write_sum);
        println!("Memfd fd: {}", fd);
        println!("Zerocopy test PASSED");
        0
    }
    #[cfg(target_os = "android")]
    {
        eprintln!("Zerocopy memfd test not available on Android (use dmabuf path instead)");
        // Test MemBuffer as fallback
        let mut buf = MemBuffer::new(size);
        let start = Instant::now();
        for i in 0..iterations {
            let val = (i % 256) as u8;
            for b in buf.data.iter_mut() {
                *b = val;
            }
        }
        let elapsed = start.elapsed();
        println!("MemBuffer fallback: {} iterations in {:?}", iterations, elapsed);
        0
    }
}

/// cmd_probe — probe profile capabilities and engine info
fn cmd_probe() -> i32 {
    println!("\n--- Probe mode ---");

    println!("\nAvailable profiles:");
    for p in cam_isp::profile::PipelineProfile::ALL.iter() {
        println!("  {:?}: level={:?} input={} unpack={} fused_stages={:?}",
            p.label, p.level, p.input_type, p.use_unpack, p.fuse_stages);
    }

    println!("\nEngine backends:");
    println!("  MnnVulkan (priority 99)");
    println!("  Cpu (not yet implemented)");
    println!("  OnnxRuntime (not yet implemented)");

    println!("\nBuffer types:");
    println!("  DmaBufBuffer (Linux memfd/dmabuf mmap + Android SharedFD)");
    println!("  AhwBuffer (Android AHardwareBuffer, Vulkan input_ahb/output_ahb)");
    println!("  MemBuffer (host memory, testing)");

    println!("\ncam-ipa features:");
    println!("  neural_enabled: opt-in NeuralController (isp-rectifier)");
    println!("  zero-copy: memfd_create + mmap on Linux, SharedFD on Android");

    0
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