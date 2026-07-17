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
#[cfg(not(feature = "v4l2"))]
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

    // Try raw Bayer formats first, fall back to RGBA
    let raw_formats: &[&[u8; 4]] = &[b"RG10", b"RG12", b"RG16", b"RGGB", b"BA81"];
    let mut config = None;

    for fourcc in raw_formats {
        let cfg = rscam::Config {
            interval: (30, 1),
            resolution: (width, height),
            format: *fourcc,
            field: 0,
            nbuffers: 2,
        };
        if cam.start(&cfg).is_ok() {
            let _ = cam.stop();
            config = Some(cfg);
            log::info!("V4L2 capture using raw format {:?}", std::str::from_utf8(*fourcc));
            break;
        }
    }

    // Fall back to RGBA
    let cfg = config.unwrap_or(rscam::Config {
        interval: (30, 1),
        resolution: (width, height),
        format: b"RGBA",
        field: 0,
        nbuffers: 2,
    });

    if config.is_none() {
        log::warn!("V4L2: no raw format supported, falling back to RGBA");
    }

    cam.start(&cfg)
        .map_err(|e| format!("Failed to configure V4L2 {}x{}: {}", width, height, e))?;

    let frame = cam
        .capture()
        .map_err(|e| format!("Failed to read V4L2 frame: {}", e))?;

    let _ = cam.stop();

    let (actual_w, actual_h) = frame.resolution;

    Ok((actual_w, actual_h, frame.to_vec()))
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
