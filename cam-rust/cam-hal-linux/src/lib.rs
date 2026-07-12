//! Linux V4L2 Camera HAL using rscam crate.
//!
//! Provides a pure-Rust implementation of the camera HAL interface
//! using the rscam V4L2 wrapper.
//!
//! # Features
//! - `v4l2` - Enable V4L2 backend (requires rscam crate)
//!
//! # Usage
//!
//! ```rust,ignore
//! use cam_hal_linux::create_v4l2_adapter;
//!
//! let adapter = create_v4l2_adapter("/dev/video0")
//!     .expect("Failed to create V4L2 adapter");
//! ```

#[cfg(feature = "v4l2")]
pub mod v4l2;

pub mod v4l2_compliance;

#[cfg(feature = "v4l2")]
use crate::v4l2::V4l2CameraAdapter;

use cam_hal::camera::ICameraAdapter;
use log::warn;

/// Create a Linux V4L2 camera adapter.
/// Create a Linux V4L2 camera adapter.
///
/// Returns None if V4L2 feature is not enabled or no camera found.
///
/// # Arguments
/// * `device_path` - Path to V4L2 device (e.g., "/dev/video0")
///
/// # Returns
/// Some(adapter) if successful, None otherwise.
pub fn create_v4l2_adapter(_device_path: &str) -> Option<Box<dyn ICameraAdapter>> {
    #[cfg(feature = "v4l2")]
    {
        V4l2CameraAdapter::new(_device_path)
            .ok()
            .map(|a| Box::new(a) as Box<dyn ICameraAdapter>)
    }
    #[cfg(not(feature = "v4l2"))]
    {
        warn!("V4L2 feature not enabled, cannot create V4L2 camera adapter");
        None
    }
}

/// List available V4L2 camera devices.
///
/// Scans /dev/video* for available camera devices.
///
/// # Returns
/// List of device paths (e.g., ["/dev/video0", "/dev/video1"])
pub fn list_v4l2_devices() -> Vec<String> {
    #[cfg(feature = "v4l2")]
    {
        crate::v4l2::list_devices()
    }
    #[cfg(not(feature = "v4l2"))]
    {
        warn!("V4L2 feature not enabled");
        vec![]
    }
}

/// Capture a single frame from a V4L2 device.
///
/// Opens the device, configures it for RGBA at the given resolution,
/// captures one frame, stops streaming, and returns the raw pixel data
/// as a `Vec<u8>`. Useful for single-shot captures in the HAL binder.
///
/// Returns `(width, height, data)` on success.
#[cfg(feature = "v4l2")]
pub fn capture_single_v4l2_frame(
    device_path: &str,
    width: u32,
    height: u32,
) -> Result<(u32, u32, Vec<u8>), String> {
    let mut cam = rscam::Camera::new(device_path)
        .map_err(|e| format!("Failed to open {}: {}", device_path, e))?;

    // Use ARGB32 (RGBA 8888) fourcc = 0x47503241 "RGBA"
    let mut cfg = rscam::Config::new();
    cfg.resolution(width, height)
        .format(0x47503241) // "RGBA" — V4L2_PIX_FMT_RGBA32
        .frame_rate(30, 1);

    cam.configure(&cfg)
        .map_err(|e| format!("Failed to configure V4L2 {}x{}: {}", width, height, e))?;

    cam.start_streaming(1)
        .map_err(|e| format!("Failed to start V4L2 streaming: {}", e))?;

    let buf = cam
        .read_frame()
        .map_err(|e| format!("Failed to read V4L2 frame: {}", e))?;

    let _ = cam.stop_streaming();

    // Get actual resolution from format
    let actual_fmt = cam
        .get_format()
        .map_err(|_| "Failed to get format".to_string())?;
    let actual_w = actual_fmt.width;
    let actual_h = actual_fmt.height;

    Ok((actual_w, actual_h, buf.data.to_vec()))
}

/// Non-V4L2 stub for `capture_single_v4l2_frame`.
#[cfg(not(feature = "v4l2"))]
pub fn capture_single_v4l2_frame(
    _device_path: &str,
    _width: u32,
    _height: u32,
) -> Result<(u32, u32, Vec<u8>), String> {
    Err("V4L2 feature not enabled".to_string())
}
