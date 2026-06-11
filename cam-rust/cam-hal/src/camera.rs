//! Camera device abstraction.
//! Ported from com.camhal.camera

use std::sync::Mutex;
use log::info;
use cam_types::{FrameFormat, CameraSourceType};

/// Type for frame callback.
pub type FrameCallback = Box<dyn Fn(ByteFrame) + Send + Sync>;

/// A byte frame sent from the camera adapter to the ISP pipeline.
#[derive(Debug, Clone)]
pub struct ByteFrame {
    pub data: Vec<u8>,
    pub width: u32,
    pub height: u32,
    pub format: String,
    pub timestamp: u64,
}

/// Trait for a camera adapter.
pub trait ICameraAdapter: Send + Sync {
    fn source_type(&self) -> CameraSourceType;
    fn open(&self);
    fn close(&self);
    fn set_preview_surface(&self, surface: *const std::ffi::c_void);
    fn set_frame_callback(&self, callback: FrameCallback);
    fn initialize(&self, on_ready: Box<dyn FnOnce() + Send>);
}

/// Base camera adapter providing shared state.
pub struct BaseCameraAdapter {
    pub source_type: CameraSourceType,
    preview_surface: Mutex<Option<*const std::ffi::c_void>>,
    frame_callback: Mutex<Option<FrameCallback>>,
}

impl BaseCameraAdapter {
    pub fn new(source_type: CameraSourceType) -> Self {
        Self {
            source_type,
            preview_surface: Mutex::new(None),
            frame_callback: Mutex::new(None),
        }
    }

    pub fn set_preview_surface(&self, surface: *const std::ffi::c_void) {
        *self.preview_surface.lock().unwrap() = Some(surface);
    }

    pub fn set_frame_callback(&self, callback: FrameCallback) {
        *self.frame_callback.lock().unwrap() = Some(callback);
    }

    pub fn get_frame_callback(&self) -> Option<FrameCallback> {
        self.frame_callback.lock().unwrap().take()
    }

    pub fn initialize(&self, on_ready: Box<dyn FnOnce() + Send>) {
        on_ready();
    }
}

/// Stream configuration.
#[derive(Debug, Clone)]
pub struct StreamConfig {
    pub width: u32,
    pub height: u32,
    pub fmt: FrameFormat,
    pub direction: i32,
    pub usage: i32,
    pub id: i32,
}

/// Buffer manager for camera frames.
pub struct BufferManager {
    allocated_buffers: Mutex<Vec<Vec<u8>>>,
}

impl BufferManager {
    pub fn new() -> Self {
        Self {
            allocated_buffers: Mutex::new(Vec::new()),
        }
    }

    pub fn allocate_buffer(&self, size: usize) -> Vec<u8> {
        vec![0u8; size]
    }

    pub fn release_buffer(&self, _buf: Vec<u8>) {
        // Return to pool
    }
}
