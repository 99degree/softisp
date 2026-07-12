//! Continuous V4L2 → ISP Streaming Example
//!
//! Streams frames from a V4L2 camera, processes through ISP in real-time,
//! and displays performance stats. Runs until interrupted.
//!
//! Usage:
//!   # Stream from camera with ISP
//!   cargo run --example stream_isp --features mnn -p cam-isp -- \
//!     --v4l2 /dev/video0 --width 1920 --height 1080 --fps 30
//!
//!   # Test pattern streaming (no camera)
//!   cargo run --example stream_isp --features mnn -p cam-isp -- \
//!     --width 640 --height 480 --fps 30
//!
//!   # MNN Vulkan backend
//!   cargo run --example stream_isp --features mnn -p cam-isp -- \
//!     --engine mnn --v4l2 /dev/video0 --width 1920 --height 1080

use std::time::{Duration, Instant};

use clap::Parser;
use log::info;

use cam_isp::engine::{select_engine, select_engine_by_name, IspEngine, ProcessParams};

#[derive(Parser, Debug)]
#[clap(about = "Continuous V4L2 → ISP streaming")]
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

    /// Target FPS
    #[clap(long, default_value_t = 30)]
    fps: u32,

    /// ISP engine: cpu, mnn, auto
    #[clap(long, default_value = "auto")]
    engine: String,

    /// Sensor max value
    #[clap(long, default_value_t = 1023.0)]
    sensor_max: f32,

    /// Duration in seconds (0 = infinite)
    #[clap(long, default_value_t = 0)]
    duration: u64,

    /// Print every N frames
    #[clap(long, default_value_t = 30)]
    print_every: usize,
}

/// Generate test pattern frame (moving gradient).
fn generate_test_frame(width: u32, height: u32, frame_num: u32) -> Vec<u8> {
    let mut data = vec![0u8; (width * height * 2) as usize];
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            let base = ((x * 1024 / width) + (y * 1024 / height) + frame_num * 50) as u16;
            let val = base.min(1023);
            data[idx * 2] = (val & 0xFF) as u8;
            data[idx * 2 + 1] = ((val >> 8) & 0xFF) as u8;
        }
    }
    data
}

/// Capture V4L2 frame.
#[cfg(feature = "v4l2")]
fn capture_v4l2(device: &str, width: u32, height: u32) -> Option<Vec<u8>> {
    // For continuous streaming, we'd use the V4L2 adapter's streaming thread.
    // For this example, we do single-shot captures in a loop.
    match cam_hal_linux::capture_single_v4l2_frame(device, width, height) {
        Ok((_w, _h, data)) => Some(data),
        Err(e) => {
            log::error!("V4L2 capture error: {}", e);
            None
        }
    }
}

#[cfg(not(feature = "v4l2"))]
fn capture_v4l2(_device: &str, _width: u32, _height: u32) -> Option<Vec<u8>> {
    None
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    info!("═══ V4L2 → ISP Streaming ═══");
    info!(
        "Resolution: {}x{} @ {}fps",
        args.width, args.height, args.fps
    );
    info!("Engine: {}, Sensor: {}", args.engine, args.sensor_max);
    if let Some(ref dev) = args.v4l2 {
        info!("V4L2 device: {}", dev);
    } else {
        info!("Mode: test pattern (no camera)");
    }
    info!(
        "Duration: {}s (or Ctrl+C)",
        if args.duration > 0 {
            args.duration.to_string()
        } else {
            "infinite".to_string()
        }
    );

    // Select ISP engine
    let engine: Box<dyn IspEngine> = if args.engine == "auto" {
        select_engine().expect("No ISP engine available")
    } else {
        select_engine_by_name(&args.engine)
            .unwrap_or_else(|| panic!("ISP engine '{}' not found", args.engine))
    };
    info!("Using engine: {}", engine.backend_name());

    let start_time = Instant::now();
    let mut frame_count: u64 = 0;
    let mut total_latency_ns: u128 = 0;
    let mut min_latency_ns: u128 = u128::MAX;
    let mut max_latency_ns: u128 = 0;
    let mut last_print = Instant::now();
    let mut frames_since_print: u64 = 0;
    let mut fps_samples: Vec<f64> = Vec::new();

    while frame_count < 10000 {
        // Check duration limit
        if args.duration > 0 && start_time.elapsed() > Duration::from_secs(args.duration) {
            info!("Duration limit reached");
            break;
        }

        let t_start = Instant::now();

        // Get frame data
        let raw_data = if let Some(ref dev) = args.v4l2 {
            match capture_v4l2(dev, args.width, args.height) {
                Some(data) => data,
                None => {
                    std::thread::sleep(Duration::from_millis(10));
                    continue;
                }
            }
        } else {
            generate_test_frame(args.width, args.height, frame_count as u32)
        };

        // Process through ISP
        let mut params = ProcessParams::new(args.width, args.height, &raw_data);
        params.sensor_max = args.sensor_max;
        params.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);

        match engine.process(&params) {
            Ok(_frame) => {
                let t_end = Instant::now();
                let latency_ns = t_end.duration_since(t_start).as_nanos();
                total_latency_ns += latency_ns;
                min_latency_ns = min_latency_ns.min(latency_ns);
                max_latency_ns = max_latency_ns.max(latency_ns);
                frame_count += 1;
                frames_since_print += 1;

                // Print stats periodically
                if last_print.elapsed() > Duration::from_secs(1) {
                    let elapsed = last_print.elapsed().as_secs_f64();
                    let fps = frames_since_print as f64 / elapsed;
                    let avg_ms = (total_latency_ns as f64 / frame_count as f64) / 1_000_000.0;
                    let min_ms = min_latency_ns as f64 / 1_000_000.0;
                    let max_ms = max_latency_ns as f64 / 1_000_000.0;

                    fps_samples.push(fps);
                    if fps_samples.len() > 10 {
                        fps_samples.remove(0);
                    }
                    let avg_fps: f64 = fps_samples.iter().sum::<f64>() / fps_samples.len() as f64;

                    info!(
                        "[{:5.1}s] frame={} fps={:.1} (avg={:.1}) latency={:.1}/{:.1}/{:.1}ms",
                        start_time.elapsed().as_secs_f64(),
                        frame_count,
                        fps,
                        avg_fps,
                        min_ms,
                        avg_ms,
                        max_ms
                    );

                    frames_since_print = 0;
                    last_print = Instant::now();
                }
            }
            Err(e) => {
                log::error!("ISP error: {}", e);
                std::thread::sleep(Duration::from_millis(10));
            }
        }
    }

    // Final summary
    let total_time = start_time.elapsed();
    let total_secs = total_time.as_secs_f64();
    let avg_fps = frame_count as f64 / total_secs;
    let avg_ms = (total_latency_ns as f64 / frame_count as f64) / 1_000_000.0;

    info!("═══ Summary ═══");
    info!("Frames: {}", frame_count);
    info!("Duration: {:.1}s", total_secs);
    info!("FPS: avg={:.1}", avg_fps);
    info!(
        "Latency: avg={:.1}ms min={:.1}ms max={:.1}ms",
        avg_ms,
        min_latency_ns as f64 / 1_000_000.0,
        max_latency_ns as f64 / 1_000_000.0
    );
}
