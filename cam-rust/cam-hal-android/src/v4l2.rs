//! V4L2 device enumeration for Android HAL.
//!
//! Provides camera device detection using rscam when the `v4l2` feature is enabled.

#[cfg(feature = "v4l2")]
use rscam::{Camera, Config};

#[cfg(feature = "v4l2")]
use std::fs;

/// List available V4L2 camera devices.
#[cfg(feature = "v4l2")]
pub fn list_devices() -> Vec<String> {
    let mut devices = Vec::new();

    // Scan /dev/video* devices
    if let Ok(entries) = fs::read_dir("/dev") {
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with("video") {
                let path = format!("/dev/{}", name);
                // Try to open the device - if successful, it's a valid camera
                if Camera::new(&path).is_ok() {
                    log::info!("Found V4L2 device: {}", path);
                    devices.push(path);
                }
            }
        }
    }

    devices
}

/// Get device info for a V4L2 device path.
#[cfg(feature = "v4l2")]
pub fn get_device_info(device_path: &str) -> Option<DeviceInfo> {
    let cam = Camera::new(device_path).ok()?;

    Some(DeviceInfo {
        path: device_path.to_string(),
        driver: String::new(),
        card: String::new(),
        bus_info: String::new(),
        version: 0,
        capabilities: 0,
        device_caps: 0,
    })

    Some(DeviceInfo {
        path: device_path.to_string(),
        driver: String::from_utf8_lossy(&caps.driver)
            .trim_end_matches('\0')
            .to_string(),
        card: String::from_utf8_lossy(&caps.card)
            .trim_end_matches('\0')
            .to_string(),
        bus_info: String::from_utf8_lossy(&caps.bus_info)
            .trim_end_matches('\0')
            .to_string(),
        version: caps.version,
        capabilities: caps.capabilities,
        device_caps: caps.device_caps,
    })
}

#[cfg(feature = "v4l2")]
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    pub path: String,
    pub driver: String,
    pub card: String,
    pub bus_info: String,
    pub version: u32,
    pub capabilities: u32,
    pub device_caps: u32,
}

// Stub implementations when v4l2 feature is disabled
#[cfg(not(feature = "v4l2"))]
pub fn list_devices() -> Vec<String> {
    vec![]
}

#[cfg(not(feature = "v4l2"))]
pub fn get_device_info(_device_path: &str) -> Option<()> {
    None
}
