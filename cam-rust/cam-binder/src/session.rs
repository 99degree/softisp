//! Camera device session — processes capture requests through the ISP pipeline.
//!
//! Each session owns an IspEngine and processes raw frames through the
//! 9-block pipeline: RawInput → Normalize → CFA → BLC → WB → Demosaic → CCM → Tone → Display.
//!
//! The session receives raw camera frames (as ByteBuffer or AHardwareBuffer),
//! feeds them through the ISP pipeline, and returns processed frames.

use log::{info, warn, error};
use std::sync::{Arc, Mutex};
use cam_types::{Frame, FrameFormat, ToneParams};
use cam_core::pipeline::PipelineManager;
use cam_isp::engine::{IspEngine, select_engine};
use cam_isp::pipeline::IspFrame;

/// A single capture session for a camera device.
pub struct CameraDeviceSession {
    /// Camera ID this session belongs to.
    camera_id: String,
    /// Pipeline manager (builds the block chain).
    pipeline: PipelineManager,
    /// Selected ISP engine.
    engine: Option<Box<dyn IspEngine>>,
    /// Session running state.
    running: bool,
    /// Frame counter.
    frame_count: u64,
}

impl CameraDeviceSession {
    pub fn new(camera_id: String, pipeline: PipelineManager) -> Self {
        Self {
            camera_id,
            pipeline,
            engine: None,
            running: false,
            frame_count: 0,
        }
    }

    /// Initialize the session — select an engine and build the pipeline.
    pub fn initialize(&mut self) -> Result<(), String> {
        // Register engines (ideally done once globally)
        // select_engine() returns the highest-priority engine from the global registry

        if self.engine.is_some() {
            return Ok(());
        }

        // Get pipeline head blocks
        let blocks = self.pipeline.get_blocks();
        if blocks.is_empty() {
            return Err("No pipeline blocks available".to_string());
        }

        // Select an engine
        match select_engine() {
            Some(mut engine) => {
                let head = blocks.into_iter().next()
                    .ok_or("No pipeline head")?;
                engine.build(head, vec![], None, 16)?;
                info!("Session {} initialized with engine {}", self.camera_id, engine.backend_name());
                self.engine = Some(engine);
                Ok(())
            }
            None => Err("No ISP engine available".to_string()),
        }
    }

    /// Process a raw frame through the ISP pipeline.
    pub fn process_frame(
        &mut self,
        width: u32,
        height: u32,
        stride: u32,
        data: &[u8],
        sensor_max: f32,
        target_width: u32,
    ) -> Result<IspFrame, String> {
        let engine = self.engine.as_ref().ok_or("Session not initialized")?;

        let tone_params = ToneParams::default();
        let frame = engine.process(
            width, height, stride, data, sensor_max, target_width,
            None, &tone_params, None, None, 1.0, 0.0, None, None, None,
        )?;

        self.frame_count += 1;
        info!("Session {} processed frame #{}: {}x{} → {}x{}",
            self.camera_id, self.frame_count, width, height,
            target_width, frame.height);

        Ok(frame)
    }

    /// Flush any pending operations.
    pub fn flush(&self) {
        info!("Session {} flushed", self.camera_id);
    }

    /// Close the session.
    pub fn close(&mut self) {
        self.running = false;
        self.engine = None;
        info!("Session {} closed ({} frames processed)", self.camera_id, self.frame_count);
    }
}
