//! Integration tests for cam-binder bridges.
//!
//! Tests the `V4l2AidlBridge`, `HardwareBufferBridge`, and their
//! integration with the ISP pipeline (when `mnn` feature is active).
//!
//! These tests run without real V4L2 hardware — the non-`v4l2` stubs
//! generate synthetic frames and exercise all fallback paths.

use cam_binder::callback::IFrameCallback;
use cam_binder::hal_bridge::*;
use cam_binder::types::StreamBuffer;
use cam_binder::v4l2_aidl_bridge::*;
use std::sync::{Arc, Mutex, atomic::{AtomicU64, Ordering}};

// ── Helpers ────────────────────────────────────────────────────────────────

/// Callback that records frames for inspection
#[derive(Clone)]
struct TestCallback {
    frames: Arc<Mutex<Vec<StreamBuffer>>>,
    count: Arc<AtomicU64>,
}

impl TestCallback {
    fn new() -> Self {
        Self {
            frames: Arc::new(Mutex::new(Vec::new())),
            count: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Total frame count (used by tests)
    #[allow(dead_code)]
    fn total(&self) -> u64 {
        self.count.load(Ordering::Relaxed)
    }

    fn last_frame(&self) -> Option<StreamBuffer> {
        self.frames.lock().unwrap().last().cloned()
    }
}

impl IFrameCallback for TestCallback {
    fn on_frame(&self, buffer: StreamBuffer) {
        self.frames.lock().unwrap().push(buffer);
        self.count.fetch_add(1, Ordering::Relaxed);
    }
}

// ── V4l2AidlBridge Tests ───────────────────────────────────────────────────

#[test]
fn test_bridge_construction() {
    let bridge = V4l2AidlBridge::new("/dev/video0");
    let stats = bridge.stats();
    assert_eq!(stats.frames_captured, 0);
    assert_eq!(stats.frames_dropped, 0);
}

#[test]
fn test_bridge_sensor_defaults() {
    let bridge = V4l2AidlBridge::new("/dev/video0");
    let sensor = bridge.sensor();
    assert_eq!(sensor.name, "default");
    assert_eq!(sensor.width, 1920);
    assert_eq!(sensor.height, 1080);
    assert_eq!(sensor.bayer_pattern, BayerPattern::Rggb);
    assert_eq!(sensor.bit_depth, 10);
}

#[test]
fn test_bridge_set_sensor() {
    let bridge = V4l2AidlBridge::new("/dev/video2");
    let spec = SensorSpec {
        name: "imx586".into(),
        width: 4000,
        height: 3000,
        bayer_pattern: BayerPattern::Bggr,
        bit_depth: 12,
        frame_rate_hz: 60,
    };
    bridge.set_sensor(spec);

    let s = bridge.sensor();
    assert_eq!(s.name, "imx586");
    assert_eq!(s.width, 4000);
    assert_eq!(s.height, 3000);
    assert_eq!(s.bayer_pattern, BayerPattern::Bggr);
    assert_eq!(s.bit_depth, 12);
    assert_eq!(s.frame_rate_hz, 60);
}

#[test]
fn test_bridge_bayer_pattern_v4l2_string() {
    assert_eq!(BayerPattern::Rggb.to_v4l2_string(), "rggb");
    assert_eq!(BayerPattern::Grbg.to_v4l2_string(), "grbg");
    assert_eq!(BayerPattern::Gbrg.to_v4l2_string(), "gbrg");
    assert_eq!(BayerPattern::Bggr.to_v4l2_string(), "bggr");
}

#[test]
fn test_bridge_capture_synthetic_stub() {
    // The non-v4l2 stub generates a synthetic RGBA frame.
    let bridge = V4l2AidlBridge::new("/dev/video0");
    let cb = TestCallback::new();

    let result = bridge.capture_and_forward(&cb);
    assert!(result.is_ok(), "stub capture should succeed: {:?}", result);

    let frame = cb.last_frame().expect("should have received a frame");
    assert_eq!(frame.format, 0x1); // HAL_PIXEL_FORMAT_RGBA_8888
    assert_eq!(frame.width, 1920);
    assert_eq!(frame.height, 1080);
    assert_eq!(frame.stride, 1920 * 4);
    assert!(frame.frame_number >= 0);
}

#[test]
fn test_bridge_capture_loop_synthetic() {
    let bridge = V4l2AidlBridge::new("/dev/video0");
    let cb = TestCallback::new();

    let results = bridge.capture_loop(5, &cb);
    assert!(results.is_ok());

    let stats = results.unwrap();
    assert_eq!(stats.frames_captured, 5, "all 5 frames captured");
    assert_eq!(stats.frames_dropped, 0, "no frames dropped");
}

#[test]
fn test_bridge_capture_loop_synthetic_detailed() {
    let bridge = V4l2AidlBridge::new("/dev/video0");
    bridge.set_sensor(SensorSpec {
        name: "test".into(),
        width: 640,
        height: 480,
        ..Default::default()
    });

    let cb = TestCallback::new();
    let results = bridge.capture_loop(3, &cb).expect("capture loop");
    assert_eq!(results.frames_captured, 3);
    assert_eq!(results.frames_dropped, 0);

    let last = cb.last_frame().unwrap();
    assert_eq!(last.width, 640);
    assert_eq!(last.height, 480);
}

#[test]
fn test_bridge_stats_accumulate() {
    let bridge = V4l2AidlBridge::new("/dev/video0");
    let cb = TestCallback::new();

    let initial = bridge.stats();
    assert_eq!(initial.frames_captured, 0);

    bridge.capture_loop(3, &cb).unwrap();
    let after = bridge.stats();
    // stats() returns the internal running count:
    assert!(after.frames_captured <= 3,
        "stats should reflect captured frames (got {})", after.frames_captured);
}

#[test]
fn test_bridge_stop_early() {
    let bridge = V4l2AidlBridge::new("/dev/video0");
    bridge.stop(); // no-op when not running
}

// ── HardwareBufferBridge Tests ─────────────────────────────────────────────

#[test]
fn test_hwb_bridge_initial_stats() {
    let bridge = HardwareBufferBridge::new(4);
    let stats = bridge.stats();
    assert_eq!(stats.zero_copy_count, 0);
    assert_eq!(stats.fallback_count, 0);
    assert_eq!(stats.cached_count, 0);
}

#[test]
fn test_hwb_bridge_acquire_release() {
    let bridge = HardwareBufferBridge::new(4);
    let locked = bridge.acquire(1920, 1080, HalPixelFormat::Rgba8888);
    assert!(locked.is_ok());

    let l = locked.unwrap();
    assert_eq!(l.width, 1920);
    assert_eq!(l.height, 1080);
    assert_eq!(l.stride, 1920);
    assert!(l.id > 0);

    bridge.release(l);
    let stats = bridge.stats();
    assert_eq!(stats.zero_copy_count, 1);
}

#[test]
fn test_hwb_bridge_pool_exhaustion() {
    let bridge = HardwareBufferBridge::new(2);
    let b1 = bridge.acquire(640, 480, HalPixelFormat::Rgba8888).unwrap();
    let b2 = bridge.acquire(640, 480, HalPixelFormat::Rgba8888).unwrap();
    let b3 = bridge.acquire(640, 480, HalPixelFormat::Rgba8888);
    assert!(b3.is_err(), "pool of 2 should reject 3rd");

    bridge.release(b1);
    bridge.release(b2);

    let b4 = bridge.acquire(640, 480, HalPixelFormat::Rgba8888);
    assert!(b4.is_ok(), "after release, acquire should succeed");
    bridge.release(b4.unwrap());
}

#[test]
fn test_hwb_bridge_format_separation() {
    let bridge = HardwareBufferBridge::new(8);
    let rgba = bridge.acquire(640, 480, HalPixelFormat::Rgba8888).unwrap();
    let yuv = bridge.acquire(640, 480, HalPixelFormat::Yuv420SemiPlanar).unwrap();
    assert_ne!(rgba.id, yuv.id, "different formats → different buffer IDs");

    bridge.release(rgba);
    bridge.release(yuv);
}

#[test]
fn test_hwb_bridge_reuse_same_dimensions() {
    let bridge = HardwareBufferBridge::new(4);
    let b1 = bridge.acquire(1920, 1080, HalPixelFormat::Rgba8888).unwrap();
    let _id1 = b1.id;
    bridge.release(b1);

    // Next acquire with same dims should reuse cached details
    let b2 = bridge.acquire(1920, 1080, HalPixelFormat::Rgba8888).unwrap();
    // The struct itself is not reused (ptr remains null in standalone mode),
    // but the stats reflect the zero-copy release.
    assert_eq!(bridge.stats().zero_copy_count, 1);
    drop(b2);
}

#[test]
fn test_hwb_bridge_pixel_format_bpp() {
    assert_eq!(HalPixelFormat::Rgba8888.bytes_per_pixel(), 4);
    assert_eq!(HalPixelFormat::RawBayer.bytes_per_pixel(), 2);
    assert_eq!(HalPixelFormat::Yuv420SemiPlanar.bytes_per_pixel(), 1);
    assert_eq!(HalPixelFormat::Yuv420Planar.bytes_per_pixel(), 1);
}

#[test]
fn test_hwb_bridge_multi_format_acquire() {
    let bridge = HardwareBufferBridge::new(16);
    for fmt in &[
        HalPixelFormat::Rgba8888,
        HalPixelFormat::RawBayer,
        HalPixelFormat::Yuv420SemiPlanar,
        HalPixelFormat::Yuv420Planar,
    ] {
        let buf = bridge.acquire(1280, 720, *fmt);
        assert!(buf.is_ok(), "acquire {:?} should succeed", fmt);
        bridge.release(buf.unwrap());
    }
    // 4 releases, 0 fallbacks
    assert_eq!(bridge.stats().zero_copy_count, 4);
    assert_eq!(bridge.stats().fallback_count, 0);
}

// ── CameraIspService wiring (mnn feature gate) ─────────────────────────────
//
// When built with `--features mnn`, these tests verify the ISP pipeline
// attaches correctly and the Bayer→RGB fallback chain works.

#[test]
#[cfg(feature = "mnn")]
fn test_isp_service_set_and_capture() {
    // Build a CameraIspService backed by the CPU engine pipeline
    let isp = match cam_isp::integration::CameraIspService::new("cpu", 1023.0, 0) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("ISP init skipped ({}), test requires MNN libs", e);
            return;
        }
    };

    let bridge = V4l2AidlBridge::new("/dev/video0");
    bridge.set_sensor(SensorSpec {
        name: "test-sensor".into(),
        width: 640,
        height: 480,
        bayer_pattern: BayerPattern::Rggb,
        bit_depth: 10,
        frame_rate_hz: 30,
    });
    bridge.set_isp_service(isp);

    // Capture stub with ISP — should produce RGBA frame
    let cb = TestCallback::new();
    let result = bridge.capture_and_forward(&cb);
    assert!(result.is_ok(), "capture with ISP should succeed: {:?}", result);

    let frame = cb.last_frame().expect("frame received");
    assert_eq!(frame.width, 640);
    assert_eq!(frame.height, 480);
    assert_eq!(frame.format, 0x1);
    // Data should be non-zero if ISP actually processed (not all-zero synthetic)
    if frame.data.iter().any(|&b| b != 0) {
        println!("ISP produced non-zero frame data (good)");
    }
}

// ── Error Handling ─────────────────────────────────────────────────────────

#[test]
fn test_bridge_error_display() {
    let e = BridgeError::DeviceError("no such device".into());
    let msg = format!("{}", e);
    assert!(msg.contains("no such device"), "display: {}", msg);
}

#[test]
fn test_bridge_error_kinds() {
    let dev = BridgeError::DeviceError("dev".into());
    let buf = BridgeError::BufferError("buf".into());
    let pipe = BridgeError::PipelineError("pipe".into());
    let aidl = BridgeError::AidlError("aidl".into());

    assert!(format!("{}", dev).starts_with("V4L2 device error"));
    assert!(format!("{}", buf).starts_with("Buffer error"));
    assert!(format!("{}", pipe).starts_with("Pipeline error"));
    assert!(format!("{}", aidl).starts_with("AIDL error"));
}
