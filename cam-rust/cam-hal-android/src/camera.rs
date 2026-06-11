//! Android Camera2 adapter implementation.
//! Ported from com.camhal.camera.Camera2RawAdapter

use std::sync::{Arc, Mutex, atomic::{AtomicBool, AtomicU32, Ordering}};
// unused


use log::{info, warn, error};
use cam_types::CameraSourceType;
use cam_hal::{ICameraAdapter, FrameCallback};

/// Android Camera2 raw sensor adapter.
///
/// Uses the Android NDK camera API (ACameraManager, ACameraDevice) to capture raw frames.
/// Supports RAW_SENSOR, RAW10, and RAW12 formats.
pub struct Camera2RawAdapter {
    /// Camera source type
    source_type: CameraSourceType,
    /// Camera ID
    camera_id: String,
    /// Whether the adapter is running
    running: AtomicBool,
    /// Frame callback
    frame_callback: Mutex<Option<FrameCallback>>,
    /// Raw frame dimensions
    raw_width: AtomicU32,
    raw_height: AtomicU32,
    /// Bayer pattern (RGGB=0, GRBG=1, GBRG=2, BGGR=3)
    bayer_pattern: i32,
    /// Sensor bits per sample
    bits_per_sample: u32,
}

impl Camera2RawAdapter {
    /// Create a new Camera2RawAdapter.
    pub fn new(preferred_camera_id: Option<&str>) -> Self {
        Self {
            source_type: CameraSourceType::RawCamera2,
            camera_id: preferred_camera_id.unwrap_or("0").to_string(),
            running: AtomicBool::new(false),
            frame_callback: Mutex::new(None),
            raw_width: AtomicU32::new(0),
            raw_height: AtomicU32::new(0),
            bayer_pattern: 0,
            bits_per_sample: 10,
        }
    }

    /// Open the camera device.
    fn open_device(&self) -> Result<(), String> {
        info!("Opening camera device: {}", self.camera_id);

        // TODO: Use ACameraManager_openCamera() from ndk-sys.
        // For now, simulate successful open.
        
        self.raw_width.store(640, Ordering::Relaxed);
        self.raw_height.store(480, Ordering::Relaxed);
        
        info!("Camera {} opened: {}x{}", self.camera_id, self.raw_width.load(Ordering::Relaxed), self.raw_height.load(Ordering::Relaxed));
        Ok(())
    }

    /// Close the camera device.
    fn close_device(&self) {
        info!("Closing camera device: {}", self.camera_id);
        // TODO: ACameraDevice_close()
    }

    /// Start the frame capture loop.
    fn start_capture_loop(&self) {
        let running = Arc::new(AtomicBool::new(true));
        let cb_lock = Arc::new(Mutex::new(()));
        
        let width = self.raw_width.load(Ordering::Relaxed);
        let height = self.raw_height.load(Ordering::Relaxed);
        let callback_holder: Arc<Mutex<Option<FrameCallback>>> = self.frame_callback.lock().unwrap().take().map(|cb| {
            Arc::new(Mutex::new(Some(cb)))
        }).unwrap_or(Arc::new(Mutex::new(None)));

        // TODO: Start a real capture session using ACameraCaptureSession.
        // For now, simulate frame capture with a timer.
    }
}

impl ICameraAdapter for Camera2RawAdapter {
    fn source_type(&self) -> CameraSourceType {
        self.source_type
    }

    fn open(&self) {
        if self.running.load(Ordering::SeqCst) {
            warn!("Camera already running");
            return;
        }
        match self.open_device() {
            Ok(_) => {
                self.running.store(true, Ordering::SeqCst);
                self.start_capture_loop();
            }
            Err(e) => {
                error!("Failed to open camera: {}", e);
            }
        }
    }

    fn close(&self) {
        self.running.store(false, Ordering::SeqCst);
        self.close_device();
    }

    fn set_preview_surface(&self, _surface: *const std::ffi::c_void) {
        // TODO: Implement surface-based preview.
        info!("Preview surface set (not yet implemented)");
    }

    fn set_frame_callback(&self, callback: FrameCallback) {
        *self.frame_callback.lock().unwrap() = Some(callback);
    }

    fn initialize(&self, on_ready: Box<dyn FnOnce() + Send>) {
        info!("Initializing Camera2RawAdapter");
        on_ready();
    }
}

/// Frame rate controller for limiting capture rate.
pub struct FrameRateController {
    target_fps: f64,
    last_frame_time: u64,
}

impl FrameRateController {
    pub fn new(target_fps: f64) -> Self {
        Self {
            target_fps,
            last_frame_time: 0,
        }
    }

    /// Check if we should drop this frame based on frame rate.
    pub fn should_drop(&mut self, timestamp: u64) -> bool {
        let interval_ns = (1_000_000_000.0 / self.target_fps) as u64;
        if timestamp - self.last_frame_time < interval_ns {
            return true;
        }
        self.last_frame_time = timestamp;
        false
    }
}
