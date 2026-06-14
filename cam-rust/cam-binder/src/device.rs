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
    pub fn open(&self, callback: Arc<dyn ICameraDeviceCallback>) -> Result<Arc<Mutex<CameraDeviceSession>>, String> {
        // Check if already opened — use try_lock to avoid deadlock
        let already_opened = self.opened.try_lock()
            .map(|g| *g)
            .unwrap_or(false);
        if already_opened {
            return Err(format!("Camera {} already opened", self.camera_id));
        }

        // Clone before moving into self.callback
        let callback_for_session = callback.clone();
        if let Ok(mut cb) = self.callback.try_lock() {
            *cb = Some(callback);
        }

        // Create a new session for this device
        let session = Arc::new(Mutex::new(CameraDeviceSession::new(
            self.camera_id.clone(),
            self.device_path.clone(),
            self.info.clone(),
        )));
        if let Ok(mut s) = self.session.try_lock() {
            *s = Some(session.clone());
        }
        if let Ok(mut op) = self.opened.try_lock() {
            *op = true;
        }

        // Give the session a reference to the same callback
        session.lock().unwrap().set_callback(callback_for_session);

        info!("CameraDevice({}): opened", self.camera_id);

        // Notify callback
        if let Some(cb) = self.callback.lock().unwrap().as_ref() {
            cb.on_opened(&self.camera_id);
        }

        Ok(session)
    }

    /// Close the camera and all sessions.
    pub fn close(&self) {
        // Use try_lock to check opened state
        let is_open = self.opened.try_lock()
            .map(|g| *g)
            .unwrap_or(false);
        if !is_open {
            return;
        }

        // Close the session
        if let Some(session) = self.session.lock().unwrap().take() {
            session.lock().unwrap().close();
        }

        if let Ok(mut op) = self.opened.try_lock() {
            *op = false;
        }
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
}
