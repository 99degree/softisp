//! Camera → ISP Pipeline Integration Example
//!
//! Captures frames from a V4L2 camera (or generates test patterns),
//! processes them through the ISP pipeline, and saves output.
//!
//! Usage:
//!   # V4L2 camera with ISP
//!   cargo run --example camera_isp --features mnn -p cam-isp -- \
//!     --v4l2 /dev/video0 --width 1920 --height 1080 --frames 5
//!
//!   # Test pattern with ISP (no camera needed)
//!   cargo run --example camera_isp --features mnn -p cam-isp -- \
//!     --width 640 --height 480 --frames 3
//!
//!   # Use MNN Vulkan backend
//!   cargo run --example camera_isp --features mnn -p cam-isp -- \
//!     --engine mnn --width 1920 --height 1080 --frames 10

use std::sync::{Arc, Mutex};
use std::path::PathBuf;
use std::time::{Instant, Duration};

use clap::Parser;
use log::info;

use cam_isp::engine::{ProcessParams, IspEngine, select_engine, select_engine_by_name};
use cam_isp::pipeline::IspFrame;

#[derive(Parser, Debug)]
#[clap(about = "Camera → ISP pipeline integration")]
struct Args {
    /// V4L2 device path (omit for test pattern)
    #[clap(long)]
    v4l2: Option<String>,

    /// Frame width
    #[clap(long, default_value_t = 640)]
    width: u32,

    /// Frame height
    #[clap(long, default_value_t = 480)]
    height: u32,

    /// Number of frames to process
    #[clap(long, default_value_t = 5)]
    frames: usize,

    /// Output directory
    #[clap(long, default_value = "./isp_output")]
    out: String,

    /// ISP engine: cpu, mnn, auto
    #[clap(long, default_value = "auto")]
    engine: String,

    /// Sensor max value (1023 for 10-bit, 65535 for 16-bit)
    #[clap(long, default_value_t = 1023.0)]
    sensor_max: f32,

    /// Bayer pattern: 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR
    #[clap(long, default_value_t = 0)]
    bayer_pattern: i32,
}

/// Generate a test pattern frame (gradient + color bars).
fn generate_test_frame(width: u32, height: u32, frame_num: u32) -> Vec<u8> {
    let mut data = vec![0u8; (width * height * 2) as usize]; // uint16 raw Bayer

    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            // Create a gradient pattern that varies by frame
            let base = ((x * 1024 / width) + (y * 1024 / height) + frame_num * 100) as u16;
            let val = base.min(1023);
            // Pack as little-endian uint16
            data[idx * 2] = (val & 0xFF) as u8;
            data[idx * 2 + 1] = ((val >> 8) & 0xFF) as u8;
        }
    }
    data
}

/// Capture a single V4L2 frame.
#[cfg(feature = "v4l2")]
fn capture_v4l2_frame(device_path: &str, width: u32, height: u32) -> Result<Vec<u8>, String> {
    use cam_hal_linux::capture_single_v4l2_frame;
    let (_w, _h, data) = capture_single_v4l2_frame(device_path, width, height)?;
    Ok(data)
}

#[cfg(not(feature = "v4l2"))]
fn capture_v4l2_frame(_device_path: &str, _width: u32, _height: u32) -> Result<Vec<u8>, String> {
    Err("V4L2 feature not enabled".into())
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    info!("═══ Camera → ISP Pipeline ═══");
    info!("Resolution: {}x{}, Frames: {}", args.width, args.height, args.frames);
    info!("Engine: {}, Sensor: {}, Bayer: {}", args.engine, args.sensor_max, args.bayer_pattern);
    if let Some(ref dev) = args.v4l2 {
        info!("V4L2 device: {}", dev);
    }

    // Create output directory
    std::fs::create_dir_all(&args.out).ok();

    // Select ISP engine
    let mut engine: Box<dyn IspEngine> = if args.engine == "auto" {
        select_engine().expect("No ISP engine available")
    } else {
        select_engine_by_name(&args.engine)
            .unwrap_or_else(|| panic!("ISP engine '{}' not found", args.engine))
    };
    info!("Using engine: {}", engine.backend_name());

    // Build pipeline (placeholder — real usage would set up blocks)
    // For now, we use the engine's default configuration

    let mut total_latency = Duration::ZERO;
    let mut frame_latencies = Vec::new();

    for frame_num in 0..args.frames {
        let t_start = Instant::now();

        // Get frame data (V4L2 or test pattern)
        let raw_data = if let Some(ref dev) = args.v4l2 {
            match capture_v4l2_frame(dev, args.width, args.height) {
                Ok(data) => data,
                Err(e) => {
                    log::error!("V4L2 capture failed: {}, using test pattern", e);
                    generate_test_frame(args.width, args.height, frame_num as u32)
                }
            }
        } else {
            generate_test_frame(args.width, args.height, frame_num as u32)
        };

        let t_capture = Instant::now();

        // Process through ISP pipeline
        let mut params = ProcessParams::new(args.width, args.height, &raw_data);
        params.sensor_max = args.sensor_max;
        params.bayer_pattern = args.bayer_pattern;
        params.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);

        match engine.process(&params) {
            Ok(frame) => {
                let t_process = Instant::now();
                let latency = t_process.duration_since(t_start);
                let capture_ms = t_capture.duration_since(t_start).as_secs_f64() * 1000.0;
                let process_ms = t_process.duration_since(t_capture).as_secs_f64() * 1000.0;
                let total_ms = latency.as_secs_f64() * 1000.0;

                info!(
                    "Frame {}/{}: {}x{} → {}x{} ({} bytes) | capture={:.1}ms process={:.1}ms total={:.1}ms | engine={}",
                    frame_num + 1, args.frames,
                    args.width, args.height,
                    frame.width, frame.height, frame.data.len(),
                    capture_ms, process_ms, total_ms,
                    engine.backend_name()
                );

                // Save output frame as raw RGBA
                let out_path = PathBuf::from(&args.out).join(format!("frame_{:04}.raw", frame_num));
                if let Err(e) = std::fs::write(&out_path, &frame.data) {
                    log::error!("Failed to save {}: {}", out_path.display(), e);
                } else {
                    info!("  Saved: {} ({} bytes)", out_path.display(), frame.data.len());
                }

                total_latency += latency;
                frame_latencies.push(total_ms);
            }
            Err(e) => {
                log::error!("Frame {} ISP failed: {}", frame_num, e);
            }
        }
    }

    // Summary
    info!("═══ Summary ═══");
    info!("Frames processed: {}", frame_latencies.len());
    if !frame_latencies.is_empty() {
        let avg = frame_latencies.iter().sum::<f64>() / frame_latencies.len() as f64;
        let min = frame_latencies.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = frame_latencies.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let fps = 1000.0 / avg;
        info!("Latency: avg={:.1}ms min={:.1}ms max={:.1}ms", avg, min, max);
        info!("Throughput: {:.1} FPS", fps);
        info!("Output: {}", args.out);
    }
}
