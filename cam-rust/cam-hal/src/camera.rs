//! Camera device abstraction.
//!
//! Trait for camera adapters (V4L2, Android NDK, stub, etc.).

use std::sync::Mutex;
use cam_types::{FrameFormat, CameraSourceType};

/// A byte frame sent from the camera adapter to the ISP pipeline.
#[derive(Debug, Clone)]
pub struct ByteFrame {
    pub data: Vec<u8>,
    pub width: u32,
    pub height: u32,
    pub format: FrameFormat,
    pub timestamp: u64,
}

impl ByteFrame {
    /// Create an empty frame.
    pub fn empty() -> Self {
        Self { data: Vec::new(), width: 0, height: 0, format: FrameFormat::Rgba8888, timestamp: 0 }
    }

    /// Frame size in bytes.
    pub fn len(&self) -> usize { self.data.len() }
    pub fn is_empty(&self) -> bool { self.data.is_empty() }
}

/// Frame callback type.
pub type FrameCallback = Box<dyn Fn(ByteFrame) + Send + Sync>;

/// Camera state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraState {
    Closed,
    Open,
    Streaming,
    Error,
}

/// Stream configuration for a camera device.
#[derive(Debug, Clone)]
pub struct StreamConfig {
    pub width: u32,
    pub height: u32,
    pub format: FrameFormat,
    pub fps: u32,
}

impl StreamConfig {
    pub fn new(width: u32, height: u32, format: FrameFormat) -> Self {
        Self { width, height, format, fps: 30 }
    }
}

/// Trait for a camera adapter.
pub trait ICameraAdapter: Send + Sync {
    /// Source type (raw camera, USB, etc.)
    fn source_type(&self) -> CameraSourceType;

    /// Open the camera device with the given config.
    fn open(&mut self, config: &StreamConfig) -> Result<(), String>;

    /// Close the camera device.
    fn close(&mut self);

    /// Start streaming frames. Callback will be invoked on each frame.
    fn start_streaming(&mut self) -> Result<(), String>;

    /// Stop streaming frames.
    fn stop_streaming(&mut self);

    /// Set the frame callback.
    fn set_frame_callback(&mut self, callback: FrameCallback);

    /// Get current camera state.
    fn state(&self) -> CameraState;

    /// Get the device path/name.
    fn device_name(&self) -> &str;

    /// Send a frame to the adapter for processing.
    fn send_frame(&self, frame: ByteFrame) -> Result<(), String>;
}

/// Base camera adapter providing shared state.
pub struct BaseCameraAdapter {
    pub source_type: CameraSourceType,
    frame_callback: Mutex<Option<FrameCallback>>,
    state: Mutex<CameraState>,
}

impl BaseCameraAdapter {
    pub fn new(source_type: CameraSourceType) -> Self {
        Self {
            source_type,
            frame_callback: Mutex::new(None),
            state: Mutex::new(CameraState::Closed),
        }
    }

    pub fn set_frame_callback(&self, callback: FrameCallback) {
        *self.frame_callback.lock().unwrap() = Some(callback);
    }

    pub fn invoke_frame_callback(&self, frame: ByteFrame) {
        if let Some(cb) = self.frame_callback.lock().unwrap().as_ref() {
            cb(frame);
        }
    }

    pub fn has_frame_callback(&self) -> bool {
        self.frame_callback.lock().unwrap().is_some()
    }

    pub fn set_state(&self, state: CameraState) {
        *self.state.lock().unwrap() = state;
    }

    pub fn get_state(&self) -> CameraState {
        *self.state.lock().unwrap()
    }
}

/// Buffer manager for camera frames.
#[allow(dead_code)]
pub struct BufferManager {
    allocated_buffers: Mutex<Vec<Vec<u8>>>,
}

impl BufferManager {
    pub fn new() -> Self {
        Self { allocated_buffers: Mutex::new(Vec::new()) }
    }

    pub fn allocate_buffer(&self, size: usize) -> Vec<u8> {
        vec![0u8; size]
    }

    pub fn release_buffer(&self, _buf: Vec<u8>) {
        // Return to pool
    }
}
