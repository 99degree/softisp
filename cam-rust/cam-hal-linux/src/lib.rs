//! Linux V4L2 Camera HAL using rscam crate.
//!
//! Provides a pure-Rust implementation of the camera HAL interface
//! using the rscam V4L2 wrapper.
//!
//! ## Features
//! - `v4l2` - Enable V4L2 backend (requires rscam crate)

#[cfg(feature = "v4l2")]
pub mod v4l2;

#[cfg(feature = "v4l2")]
use crate::v4l2::V4l2CameraAdapter;

use log::warn;
use cam_hal::camera::ICameraAdapter;

/// Create a Linux V4L2 camera adapter.
/// Returns None if V4L2 feature is not enabled or no camera found.
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
