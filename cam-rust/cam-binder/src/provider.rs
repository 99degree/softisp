//! Camera HAL provider service — implements ICameraProvider.
//!
//! Registers with the Android binder service manager as "media.camera".
//! Manages available camera devices and delegates to the ISP pipeline.

use log::{info, warn, error};
use std::sync::{Arc, Mutex};
use cam_types::CameraSourceType;

use crate::device::CameraDeviceService;

/// Configuration for a single camera device.
#[derive(Clone, Debug)]
pub struct CameraDeviceConfig {
    pub camera_id: String,
    pub source_type: CameraSourceType,
    pub facing: String, // "back" or "front"
    pub sensor_orientation: i32,
}

/// Camera provider service — the entry point for the camera HAL.
pub struct CameraProviderService {
    /// Registered camera devices.
    pub devices: Vec<CameraDeviceConfig>,
    /// Running state.
    running: bool,
}

impl CameraProviderService {
    pub fn new() -> Self {
        Self {
            devices: Self::default_devices(),
            running: false,
        }
    }

    /// Create default camera device list (simulated).
    fn default_devices() -> Vec<CameraDeviceConfig> {
        vec![
            CameraDeviceConfig {
                camera_id: "0".to_string(),
                source_type: CameraSourceType::RawCamera2,
                facing: "back".to_string(),
                sensor_orientation: 90,
            },
            CameraDeviceConfig {
                camera_id: "1".to_string(),
                source_type: CameraSourceType::Camera2,
                facing: "front".to_string(),
                sensor_orientation: 270,
            },
        ]
    }

    /// Register with the Android service manager.
    /// In production, this uses the `binder` crate to register with servicemanager.
    #[cfg(feature = "android")]
    pub fn register(service: Arc<Self>) -> Result<(), String> {
        // In production, use rsbinder to register with servicemanager:
        // rsbinder::ServiceManager::add_service(CAMERA_HAL_SERVICE_NAME, service)
        info!("Camera HAL service '{}' registered (android feature)", crate::CAMERA_HAL_SERVICE_NAME);
        Ok(())
    }

    #[cfg(not(feature = "binder"))]
    pub fn register(_service: Arc<Self>) -> Result<(), String> {
        info!("Camera HAL service simulated (binder feature not enabled)");
        Ok(())
    }

    /// Get the list of available camera device IDs.
    pub fn get_camera_id_list(&self) -> Vec<String> {
        self.devices.iter().map(|d| d.camera_id.clone()).collect()
    }

    /// Create a camera device service for the given ID.
    pub fn get_camera_device(&self, camera_id: &str) -> Option<CameraDeviceService> {
        let config = self.devices.iter().find(|d| d.camera_id == camera_id)?;
        Some(CameraDeviceService::new(config.clone()))
    }

    /// Set the provider callback (for status notifications).
    pub fn set_callback(&self) {
        info!("Provider callback registered");
    }

    /// Notify device state change (folding, covers, etc.)
    pub fn notify_device_state_change(&self, state: i64) {
        info!("Device state change: {}", state);
    }

    /// Start the provider service.
    pub fn start(&mut self) {
        self.running = true;
        info!("Camera provider started with {} devices", self.devices.len());
    }

    /// Stop the provider service.
    pub fn stop(&mut self) {
        self.running = false;
        info!("Camera provider stopped");
    }
}
