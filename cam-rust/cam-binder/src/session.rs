//! ICameraDeviceSession -- AIDL Camera Device Session implementation.
//!
//! Matches `android.hardware.camera.device.ICameraDeviceSession`.
//!
//! Flow:
//! 1. configureStreams(configs) -> set up output streams
//! 2. processCaptureRequest(request) -> fill buffers with camera data
//! 3. flush() / close()

use std::sync::{Arc, Mutex};

use log::info;

use crate::types::*;
use crate::callback::ICameraDeviceCallback;

/// Attempt to capture a single frame from V4L2 device.
/// Returns raw RGBA pixel data.
#[cfg(feature = "v4l2")]
fn capture_v4l2_frame(device_path: &str, width: u32, height: u32) -> Result<Vec<u8>, String> {
    let (_w, _h, data) = cam_hal_linux::capture_single_v4l2_frame(device_path, width, height)?;
    Ok(data)
}

/// Non-V4L2 stub.
#[cfg(not(feature = "v4l2"))]
fn capture_v4l2_frame(_device_path: &str, _width: u32, _height: u32) -> Result<Vec<u8>, String> {
    Err("V4L2 feature not enabled".to_string())
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
        }
    }

    pub fn is_valid(&self) -> bool {
        *self.state.lock().unwrap() != SessionState::Closed
    }

    /// Configure output streams.
    ///
    /// Must be called before processCaptureRequest.
    /// Returns configured stream IDs.
    pub fn configure_streams(&self, configs: &[StreamConfig]) -> Vec<i32> {
        let mut streams = self.streams.lock().unwrap();
        streams.clear();
        self._buffer_pool.lock().unwrap().clear();

        for cfg in configs {
            let stream_id = cfg.stream_id;

            streams.push(ActiveStream {
                config: cfg.clone(),
                _frame_count: 0,
            });

            info!("CameraDeviceSession({}): configured stream {} ({}x{})",
                self.camera_id, stream_id, cfg.width, cfg.height);
        }

        *self.state.lock().unwrap() = SessionState::Configured;
        streams.iter().map(|s| s.config.stream_id).collect()
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
    fn capture_frame(&self, stream_id: i32, frame_number: u64) -> StreamBuffer {
        let streams = self.streams.lock().unwrap();
        let stream = match streams.iter().find(|s| s.config.stream_id == stream_id) {
            Some(s) => s,
            None => return StreamBuffer::error(stream_id, -2),
        };

        let width = stream.config.width;
        let height = stream.config.height;

        // Try real V4L2 capture first
        if let Some(dev_path) = self.v4l2_device.lock().unwrap().as_ref() {
            match capture_v4l2_frame(dev_path, width as u32, height as u32) {
                Ok(data) => {
                    return StreamBuffer::ok(stream_id, width, height, data);
                }
                Err(e) => {
                    info!("V4L2 capture failed ({}), falling back to test pattern", e);
                }
            }
        }

        // Fallback: generate test pattern
        let data = self.generate_test_pattern(width as u32, height as u32, frame_number);
        StreamBuffer::ok(stream_id, width, height, data)
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
        *self.state.lock().unwrap() = SessionState::Configured;
    }

    /// Close the session.
    pub fn close(&self) {
        *self.state.lock().unwrap() = SessionState::Closed;
        self.streams.lock().unwrap().clear();
        self._buffer_pool.lock().unwrap().clear();
        info!("CameraDeviceSession({}): closed", self.camera_id);
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
