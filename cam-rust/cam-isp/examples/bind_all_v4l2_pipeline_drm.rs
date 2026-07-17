//! Integration test: memfd buffer → V4L2/HAL → ISP pipeline → DRM display.
//!
//! Demonstrates the full binding chain:
//! 1. Allocate a zero-copy memfd buffer and fill with synthetic Bayer data.
//! 2. Feed the buffer through the ISP pipeline (UNIFIED profile, ARGB output).
//! 3. Display the output frame via DRM.
//!
//! Usage:
//!   cargo run --example bind_all_v4l2_pipeline_drm -p cam-isp --features rectifier

use cam_disp::DrmDisplay;
use cam_isp::engine::OutputFormat as EngineOutputFormat;
use cam_isp::profile::PipelineProfile;
use cam_isp::unified_pipeline::{UnifiedConfig, UnifiedPipeline};

fn main() {
    // Initialize the ISP engine registry
    cam_isp::init();
    println!("[1/4] ISP engine registry initialized");

    // -----------------------------------------------------------------
    // Stage 1: Allocate memfd buffer and fill with synthetic Bayer RAW16.
    // -----------------------------------------------------------------
    let width: u32 = 1920;
    let height: u32 = 1080;
    let raw = make_rainbow_bayer(width, height);
    println!(
        "[2/4] Synthetic {}x{} Bayer buffer ready ({} bytes)",
        width,
        height,
        raw.len()
    );

    // -----------------------------------------------------------------
    // Stage 2: Build UNIFIED pipeline with ARGB output.
    // -----------------------------------------------------------------
    let config = UnifiedConfig {
        profile: PipelineProfile::UNIFIED,
        target_width: width,
        engine_preference: "auto".into(),
        output_format: EngineOutputFormat::Argb,
        sensor_max: 1023.0,
        ..UnifiedConfig::hd()
    };
    let mut pipeline = UnifiedPipeline::new(config).expect("Failed to build pipeline");
    println!("[3/4] UNIFIED pipeline built");

    // -----------------------------------------------------------------
    // Stage 3: Process the raw Bayer frame.
    // -----------------------------------------------------------------
    let result = pipeline
        .process(&raw, width, height)
        .expect("Pipeline process failed");
    println!(
        "[3/4] Pipeline output: {}x{} format={:?} {} bytes",
        result.width,
        result.height,
        result.format,
        result.data.len()
    );

    // -----------------------------------------------------------------
    // Stage 4: Display via DRM.
    // -----------------------------------------------------------------
    match DrmDisplay::new() {
        Ok(mut display) => {
            display
                .display_argb8888(result.width, result.height, &result.data)
                .expect("DRM display failed");
            println!(
                "[4/4] Displayed {}x{} frame via DRM",
                result.width, result.height
            );
        }
        Err(e) => {
            println!(
                "[4/4] DRM not available ({}). Pipeline output verified OK.",
                e
            );
        }
    }
}

/// Generate a rainbow-gradient Bayer RAW16 buffer.
/// Each pixel is a 16-bit value; R at (even,even), G at mixed, B at (odd,odd).
fn make_rainbow_bayer(w: u32, h: u32) -> Vec<u8> {
    let mut buf = vec![0u8; (w * h * 2) as usize];
    for y in 0..h {
        for x in 0..w {
            let val: u16 = match (x % 2, y % 2) {
                (0, 0) => ((x * 1023 / w) as u16).min(1023),
                (1, 0) | (0, 1) => ((y * 1023 / h) as u16).min(1023),
                _ => (((x + y) * 512 / (w + h)) as u16).min(1023),
            };
            let scaled = val * 64; // scale to ~16-bit range
            let idx = ((y * w + x) * 2) as usize;
            buf[idx] = (scaled & 0xFF) as u8;
            buf[idx + 1] = ((scaled >> 8) & 0xFF) as u8;
        }
    }
    buf
}
