//! # ICameraDeviceSession — AIDL Camera Device Session
//!
//! Implements `android.hardware.camera.device.ICameraDeviceSession` for the
//! camera HAL service. Integrates V4L2 camera capture with ISP pipeline
//! processing.
//!
//! ## AIDL Transaction Codes
//!
//! | Code | Method | Description |
//! |------|--------|-------------|
//! | 1 | configureStreams | Set up output streams + ISP pipeline |
//! | 2 | processCaptureRequest | Capture frame → ISP → output |
//! | 3 | getStreamBuffer | Get stream buffer |
//! | 4 | returnStreamBuffer | Return stream buffer |
//! | 5 | returnInputBuffer | Return input buffer |
//! | 6 | returnResultMetadata | Return result metadata |
//! | 7 | notify | Notify shutter/error |
//! | 8 | flush | Flush pending requests |
//! | 9 | close | Close session |
//! | 10 | signalStreamFlush | Signal stream flush |
//! | 11 | setRepeatingRequest | Set repeating capture request |
//! | 12 | getRequestList | Get pending request list |
//!
//! ## Flow
//!
//! ```text
//! configure_streams(configs)
//!   ├── Parse stream configurations
//!   ├── Build ISP pipeline (PipelineBuilder)
//!   └── Return configured stream IDs
//!
//! process_capture_request(request)
//!   ├── Capture frame (V4L2 or test pattern)
//!   ├── Process through ISP pipeline
//!   └── Return processed RGBA buffer
//!
//! flush() / close()
//!   └── Release ISP pipeline resources
//! ```
//!
//! ## V4L2 Integration
//!
//! When V4L2 is enabled (`--features v4l2`):
//! - `auto_configure_v4l2()` — auto-detect camera
//! - `configure_v4l2(path)` — configure specific device
//! - `capture_frame()` — real camera capture
//!
//! Without V4L2:
//! - Falls back to SMPTE color bar test patterns
//!
//! ## ISP Pipeline
//!
//! The session builds an ISP pipeline on `configure_streams()`:
//!
//! ```text
//! Unpack → Demosaic → Display (RGBA)
//! ```
//!
//! The pipeline is stored in `IspPipelineState` and executed
//! on each `process_capture_request()`.

use std::sync::{Arc, Mutex};
use log::info;

use crate::types::*;
use crate::callback::ICameraDeviceCallback;

/// Attempt to capture a single frame from V4L2 device.
/// Returns raw RGBA pixel data.
#[cfg(feature = "v4l2")]
fn capture_v4l2_frame(device_path: &str, width: u32, height: u32) -> crate::error::BinderResult<Vec<u8>> {
    let (_w, _h, data) = cam_hal_linux::capture_single_v4l2_frame(device_path, width, height)
        .map_err(|e| crate::error::BinderError::V4L2(e))?;
    Ok(data)
}

/// Non-V4L2 stub.
#[cfg(not(feature = "v4l2"))]
fn capture_v4l2_frame(_device_path: &str, _width: u32, _height: u32) -> crate::error::BinderResult<Vec<u8>> {
    Err(crate::error::BinderError::NotImplemented("V4L2 feature not enabled".into()))
}

/// List available V4L2 cameras.
pub fn list_v4l2_cameras() -> Vec<String> {
    #[cfg(feature = "v4l2")]
    {
        cam_hal_linux::list_v4l2_devices()
    }
    #[cfg(not(feature = "v4l2"))]
    {
        vec![]
    }
}

/// Session state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SessionState {
    Idle,
    Configured,
    _Streaming,
    Flushing,
    Closed,
}

/// Configured stream.
#[derive(Debug, Clone)]
struct ActiveStream {
    config: StreamConfig,
    _frame_count: u64,
}

/// ICameraDeviceSession implementation.
/// Integrates with ISP pipeline for real-time frame processing.
pub struct CameraDeviceSession {
    camera_id: String,
    _device_path: String,
    _camera_info: CameraInfo,
    state: Mutex<SessionState>,
    streams: Mutex<Vec<ActiveStream>>,
    callback: Arc<Mutex<Option<Arc<dyn ICameraDeviceCallback>>>>,
    frame_counter: Mutex<u64>,
    /// Buffer pool: stream_id -> list of pre-allocated buffers.
    _buffer_pool: Mutex<std::collections::HashMap<i32, Vec<Vec<u8>>>>,
    /// Optional V4L2 device path for real frame capture.
    v4l2_device: Mutex<Option<String>>,
    /// ISP pipeline for frame processing.
    isp_pipeline: Mutex<Option<IspPipelineState>>,
}

/// ISP pipeline state for a session.
pub struct IspPipelineState {
    /// ONNX model bytes (generated from pipeline config).
    pub onnx_bytes: Vec<u8>,
    /// MNN model bytes (converted from ONNX).
    pub mnn_bytes: Option<Vec<u8>>,
    /// Pipeline width.
    pub width: u32,
    /// Pipeline height.
    pub height: u32,
    /// ISP engine type ("cpu", "vulkan", "auto").
    pub engine_type: String,
    /// Pipeline block IDs for debugging.
    pub block_ids: Vec<String>,
}

impl IspPipelineState {
    /// Create a new ISP pipeline state.
    pub fn new(width: u32, height: u32, engine_type: &str) -> Self {
        Self {
            onnx_bytes: Vec::new(),
            mnn_bytes: None,
            width,
            height,
            engine_type: engine_type.to_string(),
            block_ids: Vec::new(),
        }
    }
}

impl CameraDeviceSession {
    pub fn new(camera_id: String, device_path: String, camera_info: CameraInfo) -> Self {
        Self {
            camera_id,
            _device_path: device_path,
            _camera_info: camera_info,
            state: Mutex::new(SessionState::Idle),
            streams: Mutex::new(Vec::new()),
            callback: Arc::new(Mutex::new(None)),
            frame_counter: Mutex::new(0),
            _buffer_pool: Mutex::new(std::collections::HashMap::new()),
            v4l2_device: Mutex::new(None),
            isp_pipeline: Mutex::new(None),
        }
    }

    /// Auto-detect and configure V4L2 camera.
    ///
    /// Scans /dev/video* and configures the first available camera.
    pub fn auto_configure_v4l2(&self) -> Result<(), String> {
        let cameras = list_v4l2_cameras();
        if cameras.is_empty() {
            return Err("No V4L2 cameras found".into());
        }
        let device_path = &cameras[0];
        info!("CameraDeviceSession({}): auto-configured V4L2: {}", self.camera_id, device_path);
        *self.v4l2_device.lock().unwrap() = Some(device_path.to_string());
        Ok(())
    }

    /// Configure V4L2 with a specific device path.
    pub fn configure_v4l2(&self, device_path: &str) -> Result<(), String> {
        // Verify device exists
        if !std::path::Path::new(device_path).exists() {
            return Err(format!("V4L2 device not found: {}", device_path));
        }
        info!("CameraDeviceSession({}): V4L2 configured: {}", self.camera_id, device_path);
        *self.v4l2_device.lock().unwrap() = Some(device_path.to_string());
        Ok(())
    }

    /// Get the V4L2 device path (if configured).
    pub fn v4l2_device_path(&self) -> Option<String> {
        self.v4l2_device.lock().unwrap().clone()
    }

    /// List available V4L2 cameras.
    pub fn list_cameras() -> Vec<String> {
        list_v4l2_cameras()
    }

    pub fn is_valid(&self) -> bool {
        *self.state.lock().unwrap() != SessionState::Closed
    }

    /// Configure output streams.
    ///
    /// Must be called before processCaptureRequest.
    /// Returns configured stream IDs.
    /// Also initializes the ISP pipeline for the configured resolution.
    pub fn configure_streams(&self, configs: &[StreamConfig]) -> Vec<i32> {
        let mut streams = self.streams.lock().unwrap();
        streams.clear();
        self._buffer_pool.lock().unwrap().clear();

        // Find the primary output stream for ISP pipeline
        let mut primary_width: u32 = 0;
        let mut primary_height: u32 = 0;

        for cfg in configs {
            let stream_id = cfg.stream_id;
            streams.push(ActiveStream {
                config: cfg.clone(),
                _frame_count: 0,
            });

            // Use first stream as primary for ISP
            if primary_width == 0 {
                primary_width = cfg.width as u32;
                primary_height = cfg.height as u32;
            }

            info!("CameraDeviceSession({}): configured stream {} ({}x{})",
                self.camera_id, stream_id, cfg.width, cfg.height);
        }

        // Build ISP pipeline for the primary stream
        if primary_width > 0 && primary_height > 0 {
            match self.build_isp_pipeline(primary_width, primary_height) {
                Ok(isp_state) => {
                    info!("CameraDeviceSession({}): ISP pipeline ready ({} blocks, {} bytes ONNX)",
                        self.camera_id, isp_state.block_ids.len(), isp_state.onnx_bytes.len());
                    *self.isp_pipeline.lock().unwrap() = Some(isp_state);
                }
                Err(e) => {
                    log::warn!("CameraDeviceSession({}): ISP pipeline build failed: {}",
                        self.camera_id, e);
                }
            }
        }

        *self.state.lock().unwrap() = SessionState::Configured;
        streams.iter().map(|s| s.config.stream_id).collect()
    }

    /// Build the ISP pipeline for the given resolution.
    fn build_isp_pipeline(&self, width: u32, height: u32) -> Result<IspPipelineState, String> {
        use cam_isp::pipeline_builder::PipelineBuilder;

        // Build the standard ISP pipeline:
        // Unpack → Demosaic → Display (minimal for GPU execution)
        let onnx_bytes = PipelineBuilder::new(width, height)
            .unpack()
            .demosaic_bilinear()
            .display_rgba()
            .compose()?;

        let mut state = IspPipelineState::new(width, height, "auto");
        state.onnx_bytes = onnx_bytes;
        state.block_ids = vec!["unpack".into(), "demosaic".into(), "display".into()];

        Ok(state)
    }

    /// Process a single capture request.
    ///
    /// Fills buffers with camera frame data (or test pattern).
    /// Returns filled StreamBuffers.
    pub fn process_capture_request(&self, request: &CaptureRequest) -> Vec<StreamBuffer> {
        let state = *self.state.lock().unwrap();
        if state == SessionState::Closed {
            return request.buffer_requests.iter()
                .map(|br| StreamBuffer::error(br.stream_id, -1))
                .collect();
        }

        let frame_number = {
            let mut fc = self.frame_counter.lock().unwrap();
            *fc += 1;
            *fc
        };

        let mut results = Vec::new();

        for br in &request.buffer_requests {
            let buffer = self.capture_frame(br.stream_id, frame_number);
            results.push(buffer);
        }

        // Notify callback
        if let Some(cb) = self.callback.lock().unwrap().as_ref() {
            let result = CaptureResult {
                frame_number: frame_number as i64,
                buffers: results.clone(),
                exposure_time_ns: 0,
                sensitivity: 100,
                focus_distance: 0.0,
                timestamp_ns: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as i64,
                status: 0,
            };
            cb.on_capture_result(result);
        }

        results
    }

    /// Capture a single frame for a stream.
    ///
    /// If a V4L2 device is configured, captures a real frame.
    /// Otherwise generates a test pattern (color bars).
    /// Then processes through the ISP pipeline if available.
    fn capture_frame(&self, stream_id: i32, frame_number: u64) -> StreamBuffer {
        let streams = self.streams.lock().unwrap();
        let stream = match streams.iter().find(|s| s.config.stream_id == stream_id) {
            Some(s) => s,
            None => return StreamBuffer::error(stream_id, -2),
        };

        let width = stream.config.width;
        let height = stream.config.height;

        // Try real V4L2 capture first
        let raw_data = if let Some(dev_path) = self.v4l2_device.lock().unwrap().as_ref() {
            match capture_v4l2_frame(dev_path, width as u32, height as u32) {
                Ok(data) => data,
                Err(e) => {
                    info!("V4L2 capture failed ({}), falling back to test pattern", e);
                    self.generate_test_pattern(width as u32, height as u32, frame_number)
                }
            }
        } else {
            // Fallback: generate test pattern
            self.generate_test_pattern(width as u32, height as u32, frame_number)
        };

        // Process through ISP pipeline if available
        let output_data = {
            let isp_guard = self.isp_pipeline.lock().unwrap();
            if let Some(ref isp_state) = *isp_guard {
                match self.process_through_isp(&raw_data, isp_state) {
                    Ok(processed) => {
                        info!("ISP: {}x{} → {}x{} ({} bytes)",
                            width, height, width, height, processed.len());
                        processed
                    }
                    Err(e) => {
                        log::warn!("ISP processing failed: {}, using raw", e);
                        raw_data
                    }
                }
            } else {
                raw_data
            }
        };

        StreamBuffer::ok(stream_id, width, height, output_data)
    }

    /// Process raw camera data through the ISP pipeline.
    fn process_through_isp(&self, raw_data: &[u8], isp_state: &IspPipelineState) -> Result<Vec<u8>, cam_isp::error::IspError> {
        use cam_isp::engine::{IspEngine, ProcessParams, select_engine_by_name};

        // Select engine based on pipeline config
        let engine = select_engine_by_name(&isp_state.engine_type)
            .ok_or_else(|| cam_isp::error::IspError::Config(format!("ISP engine '{}' not available", isp_state.engine_type)))?;

        // Create processing params
        let mut params = ProcessParams::new(
            isp_state.width,
            isp_state.height,
            raw_data,
        );
        params.sensor_max = 1023.0; // 10-bit sensor

        // Process through ISP
        let output = engine.process(&params)?;
        Ok(output.data)
    }

    /// Set a V4L2 device path for real frame capture.
    pub fn set_v4l2_device(&self, device_path: &str) {
        *self.v4l2_device.lock().unwrap() = Some(device_path.to_string());
        info!("CameraDeviceSession({}): V4L2 device set to {}", self.camera_id, device_path);
    }

    /// Clear the V4L2 device (use test patterns).
    pub fn clear_v4l2_device(&self) {
        *self.v4l2_device.lock().unwrap() = None;
    }

    /// Generate SMPTE color bar test pattern.
    fn generate_test_pattern(&self, width: u32, height: u32, frame_number: u64) -> Vec<u8> {
        let mut data = vec![0u8; (width * height * 4) as usize];

        // Color bar colors (RGBA)
        let bars: [[u8; 4]; 8] = [
            [192, 192, 192, 255], // 75% white
            [192, 192, 0, 255],   // 75% yellow
            [0, 192, 192, 255],   // 75% cyan
            [0, 192, 0, 255],     // 75% green
            [192, 0, 192, 255],   // 75% magenta
            [192, 0, 0, 255],     // 75% red
            [0, 0, 192, 255],     // 75% blue
            [0, 0, 0, 255],       // 75% black
        ];

        let bar_width = width / 8;
        let tick = (frame_number % 60) as u32;

        for y in 0..height {
            for x in 0..width {
                let bar_idx = (x / bar_width).min(7) as usize;
                let color = bars[bar_idx];

                // Moving scan line for animation
                let scan_line = ((y + tick * 2) % height) as usize;
                let alpha = if scan_line < height as usize / 2 { 255 } else { 200 };

                let offset = ((y * width + x) * 4) as usize;
                if offset + 3 < data.len() {
                    data[offset] = color[0];
                    data[offset + 1] = color[1];
                    data[offset + 2] = color[2];
                    data[offset + 3] = alpha;
                }
            }
        }

        data
    }

    /// Flush all pending capture requests.
    pub fn flush(&self) {
        *self.state.lock().unwrap() = SessionState::Flushing;
        info!("CameraDeviceSession({}): flush", self.camera_id);
        // ISP pipeline continues running (no cleanup needed)
        *self.state.lock().unwrap() = SessionState::Configured;
    }

    /// Close the session and release ISP resources.
    pub fn close(&self) {
        *self.state.lock().unwrap() = SessionState::Closed;
        self.streams.lock().unwrap().clear();
        self._buffer_pool.lock().unwrap().clear();
        // Release ISP pipeline
        *self.isp_pipeline.lock().unwrap() = None;
        info!("CameraDeviceSession({}): closed, ISP pipeline released", self.camera_id);
    }

    /// Pause frame production.
    pub fn pause(&self) {
        info!("CameraDeviceSession({}): pause", self.camera_id);
    }

    /// Resume frame production.
    pub fn resume(&self) {
        info!("CameraDeviceSession({}): resume", self.camera_id);
    }

    /// Set the device callback on the session.
    pub fn set_callback(&self, callback: Arc<dyn ICameraDeviceCallback>) {
        *self.callback.lock().unwrap() = Some(callback);
    }

    /// AIDL: signalStreamFlush — signal that a stream's buffers are ready to be flushed.
    pub fn signal_stream_flush(&self, _stream_id: i32, _frame_number: i64) {
        info!("CameraDeviceSession({}): signalStreamFlush", self.camera_id);
    }

    /// AIDL: setRepeatingRequest — set a repeating capture request.
    pub fn set_repeating_request(&self, request: &CaptureRequest) -> Result<(), String> {
        info!("CameraDeviceSession({}): setRepeatingRequest frame={}", self.camera_id, request.frame_number);
        Ok(())
    }

    /// AIDL: getRequestList — get the list of pending capture requests.
    pub fn get_request_list(&self) -> Vec<CaptureRequest> {
        Vec::new()
    }

    /// AIDL: getActiveStreamConfigurations — get currently active streams.
    pub fn get_active_stream_configurations(&self) -> Vec<StreamConfig> {
        self.streams.lock().unwrap().iter().map(|s| s.config.clone()).collect()
    }

    /// AIDL: getActiveStreamBufferCount — get buffer count for a stream.
    pub fn get_active_stream_buffer_count(&self, stream_id: i32) -> i32 {
        self.streams.lock().unwrap()
            .iter()
            .find(|s| s.config.stream_id == stream_id)
            .map(|s| s.config.buffer_count)
            .unwrap_or(0)
    }
}
