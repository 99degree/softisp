//! V4L2 device enumeration.

#[cfg(feature = "v4l2")]
use rscam::Camera;

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
                // Try to open the device
                if Camera::new(&path).is_ok() {
                    info!("Found V4L2 device: {}", path);
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
    let _cam = Camera::new(device_path).ok()?;

    Some(DeviceInfo {
        path: device_path.to_string(),
        driver: String::new(),
        card: device_path.to_string(),
        bus_info: String::new(),
        version: 0,
        capabilities: 0,
        device_caps: 0,
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

#[cfg(not(feature = "v4l2"))]
pub fn list_devices() -> Vec<String> {
    vec![]
}

#[cfg(not(feature = "v4l2"))]
pub fn get_device_info(_device_path: &str) -> Option<()> {
    None
}
