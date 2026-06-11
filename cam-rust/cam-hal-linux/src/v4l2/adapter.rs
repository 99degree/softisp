//! V4L2 Camera Adapter - implements ICameraAdapter trait.

use log::{info, warn};
use std::sync::Mutex;
use cam_types::{CameraSourceType, FrameFormat};
use cam_hal::camera::{ICameraAdapter, BaseCameraAdapter, StreamConfig, FrameCallback, ByteFrame};
#[cfg(feature = "v4l2")]
use rscam::{Camera, Config};

use super::{configure_stream, buffer_to_byte_frame};

#[cfg(feature = "v4l2")]
#[derive(Default)]
struct InnerState {
    camera: Option<Camera>,
    running: bool,
    stream_config: Option<StreamConfig>,
}

/// V4L2 Camera Adapter implementing ICameraAdapter trait.
pub struct V4l2CameraAdapter {
    base: BaseCameraAdapter,
    device_path: String,
    inner: Mutex<InnerState>,
}

#[cfg(feature = "v4l2")]
impl V4l2CameraAdapter {
    pub fn new(device_path: &str) -> Result<Self, String> {
        let cam = Camera::new(device_path)
            .map_err(|e| format!("Failed to open V4L2 device: {}", e))?;
        let info = cam.query_capability()
            .map_err(|e| format!("Failed to query capabilities: {}", e))?;
        info!("V4L2 device: driver={}, card={}", 
            String::from_utf8_lossy(&info.driver),
            String::from_utf8_lossy(&info.card));

        Ok(Self {
            base: BaseCameraAdapter::new(CameraSourceType::RawCamera2),
            device_path: device_path.to_string(),
            inner: Mutex::new(InnerState {
                camera: Some(cam),
                running: false,
                stream_config: None,
            }),
        })
    }

    fn with_camera<F, R>(&self, f: F) -> Result<R, String>
    where
        F: FnOnce(&mut Camera) -> R,
    {
        let mut inner = self.inner.lock().map_err(|e| format!("Lock error: {}", e))?;
        if let Some(ref mut cam) = inner.camera {
            Ok(f(cam))
        } else {
            Err("Camera not initialized".to_string())
        }
    }
}

impl ICameraAdapter for V4l2CameraAdapter {
    fn source_type(&self) -> CameraSourceType {
        self.base.source_type
    }

    fn open(&self) {
        info!("V4L2 adapter open: {}", self.device_path);
        // Actual open happens during start_streaming
    }

    fn close(&self) {
        info!("V4L2 adapter close: {}", self.device_path);
        let mut inner = self.inner.lock().unwrap();
        if let Some(mut cam) = inner.camera.take() {
            let _ = cam.stop_streaming();
        }
        inner.running = false;
    }

    fn set_preview_surface(&self, _surface: *const std::ffi::c_void) {
        warn!("Preview surface not used in V4L2 adapter");
    }

    fn set_frame_callback(&self, callback: FrameCallback) {
        self.base.set_frame_callback(callback);
    }

    fn initialize(&self, on_ready: Box<dyn FnOnce() + Send>) {
        on_ready();
        info!("V4L2 adapter initialized");
    }
}

#[cfg(feature = "v4l2")]
impl V4l2CameraAdapter {
    /// Configure the stream (must be called before streaming)
    pub fn configure_stream(
        &self,
        width: u32,
        height: u32,
        format: FrameFormat,
        fps: u32,
    ) -> Result<(), String> {
        let mut inner = self.inner.lock().map_err(|e| format!("Lock error: {}", e))?;
        let cam = inner.camera.as_mut().ok_or("Camera not initialized")?;

        configure_stream(cam, width, height, format, fps)?;

        let config = StreamConfig {
            width,
            height,
            fmt: format,
            direction: 0,
            usage: 0,
            id: 0,
        };
        inner.stream_config = Some(config);
        Ok(())
    }

    /// Start streaming frames
    pub fn start_streaming(&self) -> Result<(), String> {
        let mut inner = self.inner.lock().map_err(|e| format!("Lock error: {}", e))?;
        let cam = inner.camera.as_mut().ok_or("Camera not initialized")?;

        // Use configured stream or default
        if inner.stream_config.is_none() {
            configure_stream(cam, 1280, 720, cam_types::FrameFormat::Rgba8888, 30)?;
            inner.stream_config = Some(StreamConfig {
                width: 1280,
                height: 720,
                fmt: cam_types::FrameFormat::Rgba8888,
                direction: 0,
                usage: 0,
                id: 0,
            });
        }

        cam.start_streaming(4)
            .map_err(|e| format!("Failed to start streaming: {:?}", e))?;

        inner.running = true;
        info!("V4L2 streaming started");
        Ok(())
    }

    /// Stop streaming
    pub fn stop_streaming(&self) -> Result<(), String> {
        let mut inner = self.inner.lock().map_err(|e| format!("Lock error: {}", e))?;
        if let Some(mut cam) = inner.camera.take() {
            let _ = cam.stop_streaming();
        }
        inner.running = false;
        info!("V4L2 streaming stopped");
        Ok(())
    }

    /// Capture a single frame (blocking)
    pub fn capture_frame(&self) -> Result<ByteFrame, String> {
        let mut inner = self.inner.lock().map_err(|e| format!("Lock error: {}", e))?;
        let cam = inner.camera.as_mut().ok_or("Camera not initialized")?;
        let config = inner.stream_config.as_ref().ok_or("Stream not configured")?;

        let buf = cam.read_frame()
            .map_err(|e| format!("Failed to read frame: {:?}", e))?;

        let frame = buffer_to_byte_frame(
            &buf,
            config.width,
            config.height,
            config.fmt,
        );

        // Invoke callback if set
        if let Some(callback) = self.base.get_frame_callback() {
            callback(frame.clone());
        }

        Ok(frame)
    }

    /// Get current stream configuration
    pub fn get_stream_config(&self) -> Option<StreamConfig> {
        let inner = self.inner.lock().unwrap();
        inner.stream_config.clone()
    }
}
