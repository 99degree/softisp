//! Post-Processing Pipeline Demo
//!
//! Demonstrates EIS, GDC, and temporal denoise working together
//! on top of the default ISP pipeline.
//!
//! Usage:
//!   cargo run --example postprocess_demo -p cam-isp
//!   cargo run --example postprocess_demo -p cam-isp -- --eis --gdc --denoise

use std::time::Instant;
use clap::Parser;
use log::info;

use cam_isp::eis::GyroSample;
use cam_isp::pipeline::IspFrame;
use cam_isp::postprocess::{PostProcessConfig, PostProcessPipeline};
use cam_types::FrameFormat;

#[derive(Parser, Debug)]
#[clap(about = "Post-processing pipeline demo")]
struct Args {
    /// Enable EIS stabilization
    #[clap(long)]
    eis: bool,

    /// Enable GDC (lens distortion correction)
    #[clap(long)]
    gdc: bool,

    /// Enable temporal denoise
    #[clap(long)]
    denoise: bool,

    /// Number of frames to process
    #[clap(long, default_value_t = 10)]
    frames: usize,

    /// Frame width
    #[clap(long, default_value_t = 640)]
    width: u32,

    /// Frame height
    #[clap(long, default_value_t = 480)]
    height: u32,
}

/// Generate a test frame with moving content.
fn generate_test_frame(width: u32, height: u32, frame_num: u32) -> Vec<u8> {
    let mut data = vec![0u8; (width * height * 4) as usize];

    for y in 0..height {
        for x in 0..width {
            let idx = ((y * width + x) * 4) as usize;

            // Moving gradient pattern
            let shift = (frame_num as f32 * 5.0) as i32;
            let r = ((x as i32 + shift) % 256) as u8;
            let g = ((y as i32) % 256) as u8;
            let b = (((x + y) as i32 + shift) % 256) as u8;

            data[idx] = r;
            data[idx + 1] = g;
            data[idx + 2] = b;
            data[idx + 3] = 255;
        }
    }
    data
}

/// Simulate gyro data (gentle oscillation).
fn simulate_gyro(frame_num: u32, timestamp_ns: i64) -> GyroSample {
    let t = frame_num as f32 * 0.033; // ~30fps
    GyroSample {
        timestamp_ns,
        x: (t * 2.0).sin() * 0.05, // Pitch oscillation
        y: (t * 3.0).cos() * 0.03, // Yaw oscillation
        z: (t * 1.5).sin() * 0.02, // Roll oscillation
    }
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    info!("═══ Post-Processing Pipeline Demo ═══");
    info!("Resolution: {}x{}, Frames: {}", args.width, args.height, args.frames);
    info!("EIS: {}, GDC: {}, Denoise: {}", args.eis, args.gdc, args.denoise);

    // Build post-process config
    let config = PostProcessConfig {
        eis_enabled: args.eis,
        gdc_enabled: args.gdc,
        temporal_denoise_enabled: args.denoise,
        ..PostProcessConfig::default()
    };

    // Create pipeline with selected features
    let mut pipeline = PostProcessPipeline::new(config)
        .with_eis(0.10, 0.15) // 10% crop, 0.15 smoothing
        .with_gdc(-0.3, 0.1, 0.0, 0.0, 0.0, 1.0); // Typical lens distortion

    if args.denoise {
        // Reconfigure with denoise enabled
        pipeline = PostProcessPipeline::new(PostProcessConfig {
            eis_enabled: args.eis,
            gdc_enabled: args.gdc,
            temporal_denoise_enabled: true,
            temporal_denoise_blend: 0.3,
            ..Default::default()
        })
        .with_eis(0.10, 0.15)
        .with_gdc(-0.3, 0.1, 0.0, 0.0, 0.0, 1.0);
    }

    let mut total_latency = std::time::Duration::ZERO;
    let mut frame_latencies = Vec::new();

    for frame_num in 0..args.frames {
        let t_start = Instant::now();
        let frame_num_u32 = frame_num as u32;

        // Simulate gyro input for EIS
        if args.eis {
            let timestamp_ns = 1_000_000_000 + (frame_num as i64 * 33_333_333); // ~30fps
            let gyro = simulate_gyro(frame_num_u32, timestamp_ns);
            pipeline.push_gyro_sample(gyro);
        }

        // Create ISP output frame
        let data = generate_test_frame(args.width, args.height, frame_num_u32);
        let mut frame = IspFrame::new(args.width, args.height, FrameFormat::Rgba8888);
        frame.data = data;
        frame.timestamp_ns = 1_000_000_000 + (frame_num as u64 * 33_333_333);

        // Process through post-processing pipeline
        let output = pipeline.process(&frame).unwrap();

        let latency = t_start.elapsed();
        total_latency += latency;
        frame_latencies.push(latency.as_secs_f64() * 1000.0);

        if frame_num % 3 == 0 || frame_num == args.frames - 1 {
            info!(
                "Frame {}/{}: {}x{} → {}x{} ({:.2}ms)",
                frame_num + 1,
                args.frames,
                args.width,
                args.height,
                output.width,
                output.height,
                latency.as_secs_f64() * 1000.0
            );
        }
    }

    // Summary
    let avg = frame_latencies.iter().sum::<f64>() / frame_latencies.len() as f64;
    let min = frame_latencies.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = frame_latencies.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let fps = 1000.0 / avg;

    info!("═══ Summary ═══");
    info!("Frames: {}", args.frames);
    info!("Latency: avg={:.2}ms min={:.2}ms max={:.2}ms", avg, min, max);
    info!("Throughput: {:.1} FPS", fps);
    info!("Features: EIS={}, GDC={}, Denoise={}", args.eis, args.gdc, args.denoise);
}
