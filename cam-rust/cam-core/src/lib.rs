//! Core ISP pipeline manager.

pub mod pipeline;
pub mod debug;
pub mod logger;
pub mod hal_bridge;

use std::sync::{Arc, Mutex};
use log::{info, error};
use cam_isp::engine::{IspEngine, ProcessParams, select_engine};
use cam_isp::pipeline::IspFrame;
use cam_hal::ICameraAdapter;

/// Central application holder.
pub struct ApplicationHolder {
    pub isp_pipeline: Arc<Mutex<Option<Box<dyn IspEngine>>>>,
    pub camera_adapter: Arc<Mutex<Option<Box<dyn ICameraAdapter>>>>,
}

impl ApplicationHolder {
    pub fn new() -> Self {
        Self {
            isp_pipeline: Arc::new(Mutex::new(None)),
            camera_adapter: Arc::new(Mutex::new(None)),
        }
    }
}

impl Default for ApplicationHolder {
    fn default() -> Self {
        Self::new()
    }
}

impl ApplicationHolder {
    /// Initialize ISP pipeline only.
    pub fn init_pipeline(&self) -> Result<(), String> {
        info!("Initializing ISP pipeline...");
        let mut pipeline = self.isp_pipeline.lock().unwrap();
        if pipeline.is_some() {
            info!("Pipeline already initialized");
            return Ok(());
        }
        match select_engine() {
            Some(engine) => {
                info!("Selected engine: {}", engine.backend_name());
                *pipeline = Some(engine);
                Ok(())
            }
            None => {
                error!("No ISP engine available");
                Err("No ISP engine".into())
            }
        }
    }

    /// Set the camera adapter (must be called before use).
    pub fn set_camera_adapter(&self, adapter: Box<dyn ICameraAdapter>) {
        *self.camera_adapter.lock().unwrap() = Some(adapter);
        info!("Camera adapter set");
    }

    /// Process a frame through ISP pipeline.
    pub fn process_frame(
        &self,
        width: u32,
        height: u32,
        stride: u32,
        buf: &[u8],
        sensor_max: f32,
        target_width: u32,
    ) -> Option<IspFrame> {
        let pipeline = self.isp_pipeline.lock().unwrap();
        match pipeline.as_ref() {
            Some(engine) => {
                let mut params = ProcessParams::new(width, height, buf);
                params.stride_width = stride;
                params.sensor_max = sensor_max;
                params.target_width = target_width;
                engine.process(&params).ok()
            }
            None => {
                error!("ISP pipeline not initialized");
                None
            }
        }
    }

}
