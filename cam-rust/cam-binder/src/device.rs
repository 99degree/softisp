//! ICameraDevice -- AIDL Camera Device implementation.
//!
//! Matches `android.hardware.camera.device.ICameraDevice`.
//!
//! Flow:
//! 1. open(callback) -> onOpened callback with session
//! 2. session.configureStreams(configs)
//! 3. session.processCaptureRequest(request) -> fills buffers
//! 4. session.flush() / session.close()

use std::sync::{Arc, Mutex};

use log::info;

use crate::types::*;
use crate::callback::ICameraDeviceCallback;
use crate::session::CameraDeviceSession;

/// ICameraDevice implementation.
pub struct CameraDevice {
    camera_id: String,
    device_path: String,
    info: CameraInfo,
    opened: Arc<Mutex<bool>>,
    session: Arc<Mutex<Option<Arc<Mutex<CameraDeviceSession>>>>>,
    callback: Arc<Mutex<Option<Arc<dyn ICameraDeviceCallback>>>>,
}

impl CameraDevice {
    pub fn new(camera_id: String, device_path: String, info: CameraInfo) -> Self {
        Self {
            camera_id,
            device_path,
            info,
            opened: Arc::new(Mutex::new(false)),
            session: Arc::new(Mutex::new(None)),
            callback: Arc::new(Mutex::new(None)),
        }
    }

    pub fn camera_id(&self) -> &str { &self.camera_id }
    pub fn info(&self) -> &CameraInfo { &self.info }
    pub fn device_path(&self) -> &str { &self.device_path }
    pub fn is_opened(&self) -> bool { *self.opened.lock().unwrap() }

    /// Open the camera device.
    ///
    /// Creates a capture session and delivers it via the callback's onOpened().
    /// If V4L2 is available, auto-configures the camera.
    pub fn open(&self, callback: Arc<dyn ICameraDeviceCallback>) -> Result<Arc<Mutex<CameraDeviceSession>>, String> {
        if *self.opened.lock().unwrap() {
            return Err(format!("Camera {} already opened", self.camera_id));
        }

        // Clone before moving into self.callback
        let callback_for_session = callback.clone();
        *self.callback.lock().unwrap() = Some(callback);

        // Create a new session for this device
        let session = Arc::new(Mutex::new(CameraDeviceSession::new(
            self.camera_id.clone(),
            self.device_path.clone(),
            self.info.clone(),
        )));
        *self.session.lock().unwrap() = Some(session.clone());
        *self.opened.lock().unwrap() = true;

        // Give the session a reference to the same callback
        session.lock().unwrap().set_callback(callback_for_session);

        // Auto-configure V4L2 if device_path looks like a V4L2 device
        if self.device_path.starts_with("/dev/video") {
            if let Err(e) = session.lock().unwrap().configure_v4l2(&self.device_path) {
                log::warn!("CameraDevice({}): V4L2 config failed: {}", self.camera_id, e);
            }
        }

        info!("CameraDevice({}): opened", self.camera_id);

        // Notify callback
        if let Some(cb) = self.callback.lock().unwrap().as_ref() {
            cb.on_opened(&self.camera_id);
        }

        Ok(session)
    }

    /// Close the camera and all sessions.
    pub fn close(&self) {
        if !*self.opened.lock().unwrap() {
            return;
        }

        // Close the session
        if let Some(session) = self.session.lock().unwrap().take() {
            session.lock().unwrap().close();
        }

        *self.opened.lock().unwrap() = false;
        info!("CameraDevice({}): closed", self.camera_id);
    }

    /// Get camera characteristics.
    pub fn get_camera_characteristics(&self) -> CameraCharacteristics {
        CameraCharacteristics {
            camera_id: self.camera_id.clone(),
            info: self.info.clone(),
            raw_metadata: Vec::new(),
        }
    }

    /// Get the current session (if any).
    pub fn get_session(&self) -> Option<Arc<Mutex<CameraDeviceSession>>> {
        self.session.lock().unwrap().clone()
    }

    /// Set torch mode.
    pub fn set_torch_mode(&self, _on: bool) {
        info!("CameraDevice({}): torch not implemented", self.camera_id);
    }

    pub fn is_torch_supported(&self) -> bool { false }

    /// Flush all pending requests.
    pub fn flush(&self) {
        if let Some(session) = self.session.lock().unwrap().as_ref() {
            session.lock().unwrap().flush();
        }
    }

    /// AIDL: openSession — create and configure a capture session.
    pub fn open_session(&self, callback: Arc<dyn ICameraDeviceCallback>) -> Result<Arc<Mutex<CameraDeviceSession>>, String> {
        if *self.opened.lock().unwrap() {
            return Err("device already opened".into());
        }
        let session = Arc::new(Mutex::new(CameraDeviceSession::new(
            self.camera_id.clone(),
            self.device_path.clone(),
            self.info.clone(),
        )));
        session.lock().unwrap().set_callback(callback);
        *self.session.lock().unwrap() = Some(session.clone());
        *self.opened.lock().unwrap() = true;
        Ok(session)
    }

    /// AIDL: getResourceCost — get the resource cost for this device.
    pub fn get_resource_cost(&self) -> CameraResourceCost {
        CameraResourceCost {
            cost: 100,
            conflict_devices: vec![],
        }
    }

    /// AIDL: getCameraInfo — get the CameraInfo for this device.
    pub fn get_camera_info(&self) -> &CameraInfo {
        &self.info
    }
}
