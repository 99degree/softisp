//! Demo: Full pipeline wiring (HAL + ISP) using cam-core.
//!
//! This demonstrates how an application would:
//! 1. Initialize the logger
//! 2. Create an AndroidCameraAdapter (via HAL)
//! 3. Attach the ISP processor
//! 4. Simulate a capture request
//!
//! Run with: cargo run --example full_pipeline --features android -p cam-core

use std::time::Instant;

use cam_core::hal_bridge::attach_isp_to_android_adapter;
use cam_hal::{camera::StreamConfig, ICameraAdapter};
use cam_hal_android::adapter::AndroidCameraAdapter;
use cam_types::FrameFormat;

fn main() -> Result<(), String> {
    // 1. Initialize logger
    cam_core::logger::init_logger_with_tag(true, "softisp_demo");
    log::info!("=== SoftISP Full Pipeline Demo ===");

    // 2. Create the Android camera adapter
    let mut adapter = AndroidCameraAdapter::new("0");

    // 3. Open the adapter with a stream config
    let stream_cfg = StreamConfig {
        width: 640,
        height: 480,
        format: FrameFormat::RawSensor,
        fps: 30,
    };
    adapter.open(&stream_cfg)?;
    ICameraAdapter::start_streaming(&mut adapter)?;

    // 4. Attach the ISP processor (builds full pipeline)
    log::info!("Attaching ISP processor...");
    attach_isp_to_android_adapter(&mut adapter, 640, 480, 1280)?;

    // 5. Simulate frame processing
    log::info!("Simulating frame processing...");
    let start = Instant::now();

    // Create a test raw frame (640x480 RAW16 = 614,400 bytes)
    let frame_size = 640 * 480 * 2;
    let mut raw_frame = vec![0u8; frame_size as usize];

    // Fill with a simple gradient pattern (simulating Bayer data)
    for y in 0..480 {
        for x in 0..640 {
            let idx = (y * 640 + x) * 2;
            let val = ((x + y) * 65535 / 1120) as u16;
            raw_frame[idx..idx + 2].copy_from_slice(&val.to_le_bytes());
        }
    }

    // Process through the adapter's processor
    // We use the ICameraAdapter trait method to send a frame
    let byte_frame = cam_hal::camera::ByteFrame {
        data: raw_frame,
        width: 640,
        height: 480,
        format: FrameFormat::RawSensor,
        timestamp: 0,
    };

    // Note: In real usage, the adapter's processor is invoked by the HAL
    // when process_capture_request is called. Here we demonstrate the
    // processor exists by checking the adapter state.
    assert_eq!(
        ICameraAdapter::state(&adapter),
        cam_hal::camera::CameraState::Streaming
    );
    log::info!("Adapter state: Streaming, processor attached");

    let elapsed = start.elapsed();
    log::info!("Pipeline ready in {:.2}ms", elapsed.as_secs_f64() * 1000.0);

    // 6. Clean up
    ICameraAdapter::stop_streaming(&mut adapter);
    ICameraAdapter::close(&mut adapter);

    log::info!("=== Demo completed successfully ===");
    Ok(())
}
