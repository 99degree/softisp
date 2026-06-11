//! Camera device service — implements ICameraDevice.
//!
//! Each camera device is a binder service that manages:
//! - Camera characteristics (static metadata)
//! - Session lifecycle (open → configure → capture → close)
//! - ISP pipeline configuration

use log::{info, warn, error};
use std::sync::{Arc, Mutex};
use cam_types::{Frame, FrameFormat, CameraSourceType};
use cam_core::pipeline::{PipelineManager, PipelineConfig};

use crate::provider::CameraDeviceConfig;
use crate::session::CameraDeviceSession;

/// Camera device service — one per physical camera.
pub struct CameraDeviceService {
    /// Device configuration.
    pub config: CameraDeviceConfig,
    /// Whether the device is opened.
    opened: bool,
    /// Active session (if any).
    session: Option<CameraDeviceSession>,
}

impl CameraDeviceService {
    pub fn new(config: CameraDeviceConfig) -> Self {
        Self {
            config,
            opened: false,
            session: None,
        }
    }

    /// Open the camera device.
    /// Returns the device session for capture operations.
    pub fn open(&mut self) -> Result<&CameraDeviceSession, String> {
        if self.opened {
            return Err("Camera already opened".to_string());
        }
        self.opened = true;

        // Create a pipeline manager for this device
        let pipeline_config = PipelineConfig {
            target_width: 1280,
            opset_version: 16,
            bayer_pattern: 2, // GBRG
        };
        let mut pipeline = PipelineManager::new(pipeline_config);
        pipeline.build_default_chain()?;

        let session = CameraDeviceSession::new(
            self.config.camera_id.clone(),
            pipeline,
        );
        info!("Camera device {} opened", self.config.camera_id);
        self.session = Some(session);
        Ok(self.session.as_ref().unwrap())
    }

    /// Get camera characteristics as a key-value map.
    pub fn get_characteristics(&self) -> std::collections::HashMap<String, String> {
        let mut chars = std::collections::HashMap::new();
        chars.insert("camera_id".to_string(), self.config.camera_id.clone());
        chars.insert("facing".to_string(), self.config.facing.clone());
        chars.insert("orientation".to_string(), self.config.sensor_orientation.to_string());
        chars.insert("source".to_string(), format!("{:?}", self.config.source_type));
        chars
    }

    /// Close the camera device.
    pub fn close(&mut self) {
        self.session = None;
        self.opened = false;
        info!("Camera device {} closed", self.config.camera_id);
    }
}
