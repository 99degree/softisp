//! Camera device abstraction.
//!
//! Trait for camera adapters (V4L2, Android NDK, stub, etc.).

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
