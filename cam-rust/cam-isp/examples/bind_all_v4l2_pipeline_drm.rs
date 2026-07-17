//! Integration test: V4L2 → ISP pipeline (UNIFIED profile) → DRM display.
//! This example demonstrates the full pipeline:
//! 1. Capture a frame from V4L2 (or use a synthetic frame if V4L2 not available).
//! 2. Process the frame through the ISP pipeline with UNIFIED profile.
//! 3. Display the resulting ARGB8888 frame via DRM using the cam-disp crate.
//!
//! Note: This example requires the `v4l2` feature for cam-hal-linux and the `mnn` feature for cam-isp.
//! If V4L2 hardware is not available, a synthetic Bayer frame is generated.

use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use cam_isp::UnifiedConfig;
use cam_isp::UnifiedPipeline;
use cam_types::Frame;
use std::sync::Arc;

// Optional V4L2 support
#[cfg(feature = "v4l2")]
use cam_hal_linux::{capture_single_v4l2_frame, create_v4l2_adapter, list_v4l2_devices};

use cam_disp::DrmDisplay;

fn main() {
    // Initialize the ISP engine registry (required before creating pipelines)
    cam_isp::init();

    // -------------------------------------------------------------------------
    // Stage 1: Input frame acquisition (V4L2 or synthetic)
    // -------------------------------------------------------------------------
    let (width, height, mut frame) = {
        // Try to get a real frame from V4L2 if the feature is enabled and a device is found
        #[cfg(feature = "v4l2")]
        {
            let devices = list_v4l2_devices();
            if !devices.is_empty() {
                let device_path = &devices[0];
                println!("Using V4L2 device: {}", device_path);
                match capture_single_v4l2_frame(device_path, 3840, 2160) {
                    Ok((w, h, data)) => {
                        println!("Captured {}x{} frame from V4L2", w, h);
                        (w, h, Frame::from_rgba8888(w, h, data))
                    }
                    Err(e) => {
                        eprintln!(
                            "Failed to capture from V4L2: {}. Falling back to synthetic.",
                            e
                        );
                        fallback_synthetic_frame()
                    }
                }
            } else {
                println!("No V4L2 devices found. Using synthetic frame.");
                fallback_synthetic_frame()
            }
        }
        #[cfg(not(feature = "v4l2"))]
        {
            println!("V4L2 feature not enabled. Using synthetic frame.");
            fallback_synthetic_frame()
        }
    };

    // -------------------------------------------------------------------------
    // Stage 2: Build and configure the ISP pipeline (UNIFIED profile)
    // -------------------------------------------------------------------------
    let config = UnifiedConfig {
        profile: PipelineProfile::UNIFIED,
        target_width: width as u32,
        bayer_pattern: 0, // RGGB (will be overridden by the pipeline if needed)
        engine_preference: "auto".into(), // Let the system choose the best available engine (CPU/Vulkan/MNN)
        post_config: Default::default(),
        output_format: EngineOutputFormat::Argb8888, // We want ARGB8888 for display
        sensor_max: 1023.0,                          // Assuming 10-bit sensor
    };

    let mut pipeline = UnifiedPipeline::new(config).expect("Failed to create unified pipeline");

    // -------------------------------------------------------------------------
    // Stage 3: Process the frame through the pipeline
    // -------------------------------------------------------------------------
    let mut params = ProcessParams::default();
    // Note: In a real scenario, you would set AWB, AE, AF parameters here based on 3A results.
    // For this example, we use defaults.

    let output_frame = pipeline
        .process(&frame, &mut params)
        .expect("ISP processing failed");

    // -------------------------------------------------------------------------
    // Stage 4: Display the output frame via DRM
    // -------------------------------------------------------------------------
    // Ensure the output frame is in the expected format (ARGB8888)
    if output_frame.format() != OutputFormat::Argb8888 {
        panic!("Expected ARGB8888 output, got {:?}", output_frame.format());
    }

    let width = output_frame.width() as u32;
    let height = output_frame.height() as u32;
    let data = output_frame.data(); // This should be a slice of u8 (ARGB8888)

    println!("Displaying {}x{} ARGB8888 frame via DRM", width, height);

    // Create a DRM display instance and show the frame
    let mut display = DrmDisplay::new().expect("Failed to initialize DRM display");
    display
        .display_argb8888(width, height, data)
        .expect("Failed to display frame");

    println!("Frame displayed successfully!");
}

/// Generate a synthetic Bayer frame (RGGB pattern) for testing when V4L2 is not available.
/// This creates a simple gradient pattern to verify the pipeline.
fn fallback_synthetic_frame() -> (u32, u32, Frame) {
    let width = 3840;
    let height = 2160;
    // Allocate space for RAW16 (2 bytes per pixel)
    let mut raw = vec![0u8; (width * height * 2) as usize];

    // Generate a simple checkerboard-like pattern in the Bayer domain
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) * 2;
            let val = if (x % 2 == 0) == (y % 2 == 0) {
                // Red or Blue pixel (depending on position)
                if (x % 2 == 0) && (y % 2 == 0) {
                    // Red pixel
                    ((x as u16 * 65535 / width as u16) as u16)
                } else {
                    // Blue pixel
                    ((y as u16 * 65535 / height as u16) as u16)
                }
            } else {
                // Green pixel
                (((x + y) as u16 * 65535 / (width as u16 + height as u16)) as u16)
            };
            raw[idx] = (val & 0xFF) as u8;
            raw[idx + 1] = ((val >> 8) & 0xFF) as u8;
        }
    }

    (
        width as u32,
        height as u32,
        Frame::from_raw16(width as u32, height as u32, raw),
    )
}
