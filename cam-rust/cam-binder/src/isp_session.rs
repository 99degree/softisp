//! Camera HAL + ISP Pipeline Integration
//!
//! Bridges the binder camera API with the ISP pipeline.
//! When a frame is captured from the camera, it's automatically
//! processed through the ISP before being returned to the client.
//!
//! # Usage
//!
//! ```rust,ignore
//! use cam_binder::isp_session::IspCameraSession;
//!
//! let session = IspCameraSession::new("0", 1920, 1080, "auto")?;
//! let frame = session.capture_frame()?;
//! // frame.output is the ISP-processed RGBA data
//! ```

use std::sync::{Arc, Mutex};
use log::{info, error};

use crate::types::*;
use crate::callback::*;

/// ISP-processed frame result.
#[derive(Debug, Clone)]
pub struct IspFrameResult {
    /// Raw camera frame (before ISP).
    pub raw: StreamBuffer,
    /// ISP-processed output (RGBA).
    pub output: Vec<u8>,
    /// Output width after ISP.
    pub width: i32,
    /// Output height after ISP.
    pub height: i32,
    /// ISP processing latency in microseconds.
    pub isp_latency_us: u64,
    /// Total capture + ISP latency in microseconds.
    pub total_latency_us: u64,
}

/// Camera session with integrated ISP processing.
#[allow(dead_code)]
pub struct IspCameraSession {
    camera_id: String,
    width: i32,
    height: i32,
    engine_name: String,
    /// ISP engine (if available).
    engine: Arc<Mutex<Option<Box<dyn cam_isp::engine::IspEngine>>>>,
    /// Frame counter.
    frame_counter: Mutex<u64>,
}

impl IspCameraSession {
    /// Create a new ISP-integrated camera session.
    ///
    /// - `camera_id`: Camera to open (e.g., "0")
    /// - `width`, `height`: Capture resolution
    /// - `engine_name`: ISP engine ("cpu", "mnn", "auto")
    pub fn new(camera_id: &str, width: i32, height: i32, engine_name: &str) -> Result<Self, String> {
        // Initialize ISP engine
        let engine = if engine_name == "none" {
            None
        } else {
            let eng = if engine_name == "auto" {
                cam_isp::engine::select_engine()
            } else {
                cam_isp::engine::select_engine_by_name(engine_name)
            };
            eng
        };

        if let Some(ref e) = engine {
            info!("ISP engine initialized: {}", e.backend_name());
        } else {
            info!("No ISP engine — raw camera output only");
        }

        Ok(Self {
            camera_id: camera_id.to_string(),
            width,
            height,
            engine_name: engine_name.to_string(),
            engine: Arc::new(Mutex::new(engine)),
            frame_counter: Mutex::new(0),
        })
    }

    /// Capture a single frame with ISP processing.
    ///
    /// Uses the binder camera API internally:
    /// 1. Opens camera device
    /// 2. Configures streams
    /// 3. Captures raw frame
    /// 4. Processes through ISP
    /// 5. Returns ISP-processed output
    pub fn capture_frame(&self) -> Result<IspFrameResult, String> {
        let t_start = std::time::Instant::now();

        // Create a no-op callback for synchronous capture
        struct SyncCallback;
        impl ICameraDeviceCallback for SyncCallback {
            fn on_opened(&self, _camera_id: &str) {}
            fn on_error(&self, _code: i32, _msg: &str) {}
            fn on_idle(&self) {}
            fn on_capture_result(&self, _result: CaptureResult) {}
            fn on_request_queue_empty(&self) {}
        }

        // Create provider and open camera
        let provider = crate::provider::CameraProvider::new();
        let device = provider.get_camera_device(&self.camera_id)
            .ok_or_else(|| format!("Camera {} not found", self.camera_id))?;

        let session = device.lock().unwrap().open(Arc::new(SyncCallback))?;

        // Configure stream
        let stream_config = StreamConfig::new(0, self.width, self.height, 0x1);
        session.lock().unwrap().configure_streams(&[stream_config]);

        // Capture raw frame
        let frame_number = {
            let mut fc = self.frame_counter.lock().unwrap();
            *fc += 1;
            *fc
        };
        let request = CaptureRequest::preview(frame_number as i64, 0);
        let buffers = session.lock().unwrap().process_capture_request(&request);

        let raw_buffer = buffers.into_iter().next()
            .ok_or("No buffers returned")?;

        if raw_buffer.status != 0 {
            return Err(format!("Capture failed with status {}", raw_buffer.status));
        }

        let t_capture = std::time::Instant::now();

        // Process through ISP
        let output = {
            let engine_guard = self.engine.lock().unwrap();
            if let Some(ref engine) = *engine_guard {
                let mut params = cam_isp::engine::ProcessParams::new(
                    raw_buffer.width as u32,
                    raw_buffer.height as u32,
                    &raw_buffer.data,
                );
                params.sensor_max = 1023.0;
                params.timestamp_ns = raw_buffer.timestamp_ns as u64;

                match engine.process(&params) {
                    Ok(frame) => {
                        info!("ISP: {}x{} → {}x{} ({} bytes)",
                            raw_buffer.width, raw_buffer.height,
                            frame.width, frame.height, frame.data.len());
                        frame.data
                    }
                    Err(e) => {
                        error!("ISP processing failed: {}, using raw", e);
                        raw_buffer.data.clone()
                    }
                }
            } else {
                raw_buffer.data.clone()
            }
        };

        let t_end = std::time::Instant::now();
        let total_us = t_end.duration_since(t_start).as_micros() as u64;
        let isp_us = t_end.duration_since(t_capture).as_micros() as u64;

        // Close camera
        session.lock().unwrap().close();

        Ok(IspFrameResult {
            raw: raw_buffer.clone(),
            output,
            width: raw_buffer.width,
            height: raw_buffer.height,
            isp_latency_us: isp_us,
            total_latency_us: total_us,
        })
    }

    /// Capture multiple frames with ISP processing.
    pub fn capture_frames(&self, count: usize) -> Result<Vec<IspFrameResult>, String> {
        let mut frames = Vec::with_capacity(count);
        for i in 0..count {
            info!("Capturing frame {}/{}", i + 1, count);
            let frame = self.capture_frame()?;
            frames.push(frame);
        }
        Ok(frames)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_isp_session_creation() {
        let session = IspCameraSession::new("0", 640, 480, "none").unwrap();
        assert_eq!(session.camera_id, "0");
    }

    #[test]
    fn test_isp_session_capture() {
        let session = IspCameraSession::new("0", 320, 240, "none").unwrap();
        let frame = session.capture_frame().unwrap();
        assert_eq!(frame.raw.status, 0);
        assert!(frame.width > 0);
        assert!(frame.height > 0);
        assert!(!frame.output.is_empty());
    }

    #[test]
    fn test_isp_session_multi_capture() {
        let session = IspCameraSession::new("0", 160, 120, "none").unwrap();
        let frames = session.capture_frames(3).unwrap();
        assert_eq!(frames.len(), 3);
        for frame in &frames {
            assert_eq!(frame.raw.status, 0);
        }
    }
}
