//! Core ISP pipeline manager.
//! Ported from com.camcore

pub mod pipeline;
pub mod debug;
pub mod logger;

use std::sync::{Arc, Mutex};
use log::{info, error};
use cam_isp::engine::{IspEngine, select_engine};
use cam_isp::pipeline::IspFrame;
use cam_types::{Frame, FrameFormat, CameraSourceType};
use cam_hal::ICameraAdapter;

/// Application holder for the camera ISP pipeline.
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

    /// Initialize the ISP pipeline.
    pub fn init_pipeline(&self) -> Result<(), String> {
        let mut pipeline = self.isp_pipeline.lock().unwrap();
        if pipeline.is_some() {
            info!("Pipeline already initialized");
            return Ok(());
        }

        // Select best available engine
        match select_engine() {
            Some(engine) => {
                info!("Selected engine: {}", engine.backend_name());
                *pipeline = Some(engine);
                Ok(())
            }
            None => {
                let err = "No suitable ISP engine available".to_string();
                error!("{}", err);
                Err(err)
            }
        }
    }

    /// Process a frame through the ISP pipeline.
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
                // Use default tone params
                let tone_params = cam_types::ToneParams::default();
                match engine.process(
                    width, height, stride, buf, sensor_max, target_width,
                    None, &tone_params, None, None, 1.0, 0.0, None, None, None,
                ) {
                    Ok(frame) => Some(frame),
                    Err(e) => {
                        error!("Pipeline processing failed: {}", e);
                        None
                    }
                }
            }
            None => {
                error!("Pipeline not initialized");
                None
            }
        }
    }

    /// Register camera adapter.
    pub fn set_camera_adapter(&self, adapter: Box<dyn ICameraAdapter>) {
        *self.camera_adapter.lock().unwrap() = Some(adapter);
    }
}
