//! ICameraProvider -- AIDL Camera Provider implementation.
//!
//! Matches `android.hardware.camera.provider.ICameraProvider`.
//!
//! Transaction codes (per AIDL spec):
//!   1 = setCallback
//!   2 = getVendorTags
//!   3 = getCameraIdList
//!   4 = getCameraDeviceInterface
//!   5 = notifyDeviceStateChange
//!   6 = getConcurrentCameraIds
//!   7 = isConcurrentStreamCombinationSupported

use std::sync::{Arc, Mutex};

use log::info;

use crate::callback::ICameraProviderCallback;
use crate::device::CameraDevice;
use crate::types::*;

/// ICameraProvider implementation.
///
/// Manages camera enumeration and device access.
pub struct CameraProvider {
    /// Known camera devices (camera_id -> CameraDevice).
    devices: Vec<Arc<Mutex<CameraDevice>>>,
    /// Provider callback (framework registers this).
    callback: Arc<Mutex<Option<Arc<dyn ICameraProviderCallback>>>>,
    /// Current device state bitmask.
    device_state: Arc<Mutex<i64>>,
}

impl CameraProvider {
    pub fn new() -> Self {
        let mut provider = Self {
            devices: Vec::new(),
            callback: Arc::new(Mutex::new(None)),
            device_state: Arc::new(Mutex::new(0)),
        };
        provider.enumerate_cameras();
        provider
    }

    /// Enumerate available cameras.
    ///
    /// On Android: use ACameraManager.
    /// On Linux host: use V4L2 enumeration.
    fn enumerate_cameras(&mut self) {
        #[cfg(feature = "v4l2")]
        {
            let v4l2_devices = cam_hal_linux::list_v4l2_devices();
            for (i, path) in v4l2_devices.iter().enumerate() {
                let id = i.to_string();
                let dev_path = path.clone();
                let info = CameraInfo {
                    camera_id: id.clone(),
                    facing: if i == 0 { 1 } else { 2 },
                    orientation: 90,
                    supported_formats: vec![0x23, 0x20, 0x100],
                    max_resolution: (1920, 1080),
                    hardware_level: 0,
                };
                let device = CameraDevice::new(id.clone(), dev_path, info);
                self.devices.push(Arc::new(Mutex::new(device)));
                info!("CameraProvider: found camera {} at {}", id, path);
            }
        }

        // Always add at least a stub camera for testing
        #[cfg(not(feature = "v4l2"))]
        {
            let info = CameraInfo {
                camera_id: "0".to_string(),
                facing: 1,
                orientation: 90,
                supported_formats: vec![0x23, 0x100],
                max_resolution: (1920, 1080),
                hardware_level: 0,
            };
            let device = CameraDevice::new("0".to_string(), "/dev/video0".to_string(), info);
            self.devices.push(Arc::new(Mutex::new(device)));
            info!("CameraProvider: stub camera 0 added");
        }

        if self.devices.is_empty() {
            info!("CameraProvider: no cameras found");
        }
    }

    // -- ICameraProvider methods --

    /// Set the callback for camera status changes.
    pub fn set_callback(&self, callback: Arc<dyn ICameraProviderCallback>) {
        *self.callback.lock().unwrap() = Some(callback);
        info!("CameraProvider: callback set");
    }

    /// Get vendor tags.
    pub fn get_vendor_tags(&self) -> Vec<VendorTagSection> {
        Vec::new()
    }

    /// Get list of camera IDs.
    pub fn get_camera_id_list(&self) -> Vec<String> {
        self.devices
            .iter()
            .map(|d| d.lock().unwrap().camera_id().to_string())
            .collect()
    }

    /// Get a camera device interface by ID.
    pub fn get_camera_device(&self, camera_id: &str) -> Option<Arc<Mutex<CameraDevice>>> {
        self.devices
            .iter()
            .find(|d| d.lock().unwrap().camera_id() == camera_id)
            .cloned()
    }

    /// Get camera info for a device.
    pub fn get_camera_info(&self, camera_id: &str) -> Option<CameraInfo> {
        self.devices
            .iter()
            .find(|d| d.lock().unwrap().camera_id() == camera_id)
            .map(|d| d.lock().unwrap().info().clone())
    }

    /// Notify device state change (folded, covered, etc.)
    pub fn notify_device_state_change(&self, state: i64) {
        let old = *self.device_state.lock().unwrap();
        *self.device_state.lock().unwrap() = state;
        info!("CameraProvider: device state changed {} -> {}", old, state);

        // Notify callback if camera availability may have changed
        if let Some(cb) = self.callback.lock().unwrap().as_ref() {
            let changed_bits = old ^ state;
            if changed_bits
                & (DEVICE_STATE_BACK_COVERED | DEVICE_STATE_FRONT_COVERED | DEVICE_STATE_FOLDED)
                != 0
            {
                for dev in &self.devices {
                    let id = dev.lock().unwrap().camera_id().to_string();
                    cb.on_camera_device_status_change(&id, CameraDeviceStatus::Present);
                }
            }
        }
    }

    /// Get concurrent camera ID combinations.
    pub fn get_concurrent_camera_ids(&self) -> Vec<ConcurrentCameraIdCombination> {
        Vec::new()
    }

    /// Check if a concurrent stream combination is supported.
    pub fn is_concurrent_stream_combination_supported(
        &self,
        _configs: &[CameraIdAndStreamCombination],
    ) -> bool {
        false
    }

    /// Number of cameras.
    pub fn camera_count(&self) -> usize {
        self.devices.len()
    }
}

impl Default for CameraProvider {
    fn default() -> Self {
        Self::new()
    }
}
