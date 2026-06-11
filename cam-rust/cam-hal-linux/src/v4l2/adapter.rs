//! V4L2 Camera Adapter implementing ICameraAdapter with callback streaming.

use log::{info, warn, error};
use std::sync::{Arc, Mutex};
use std::thread;
use cam_types::{CameraSourceType, FrameFormat};
use cam_hal::camera::{ICameraAdapter, BaseCameraAdapter, ByteFrame, FrameCallback};

#[cfg(feature = "v4l2")]
use rscam::{Camera, Config};

use super::{configure_stream, get_stream_config, buffer_to_byte_frame};

/// V4L2 Camera Adapter that streams frames via callback.
pub struct V4l2CameraAdapter {
    base: BaseCameraAdapter,
    device_path: String,
    width: u32,
    height: u32,
    format: FrameFormat,
    running: Arc<Mutex<bool>>,
}

impl V4l2CameraAdapter {
    pub fn new(device_path: &str) -> Result<Self, String> {
        #[cfg(feature = "v4l2")]
        {
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
                width: 1280,
                height: 720,
                format: FrameFormat::Rgba8888,
                running: Arc::new(Mutex::new(false)),
            })
        }
        #[cfg(not(feature = "v4l2"))]
        Err("V4L2 feature not enabled".into())
    }

    /// Start streaming in a background thread.
    fn start_streaming_thread(self: &Arc<Self>) -> Result<(), String> {
        let device_path = self.device_path.clone();
        let width = self.width;
        let height = self.height;
        let format = self.format;
        let running = Arc::clone(&self.running);
        let callback_opt = self.base.get_frame_callback();

        let callback = callback_opt.ok_or("No frame callback set")?;

        thread::spawn(move || {
            #[cfg(feature = "v4l2")]
            {
                let mut cam = match Camera::new(&device_path) {
                    Ok(c) => c,
                    Err(e) => {
                        error!("V4L2 open failed: {:?}", e);
                        return;
                    }
                };

                // Configure stream
                if let Err(e) = configure_stream(&mut cam, width, height, format, 30) {
                    error!("V4L2 configure failed: {}", e);
                    return;
                }

                // Start streaming
                if let Err(e) = cam.start_streaming(4) {
                    error!("V4L2 start_streaming failed: {:?}", e);
                    return;
                }

                *running.lock().unwrap() = true;
                info!("V4L2 streaming thread started");

                while *running.lock().unwrap() {
                    match cam.read_frame() {
                        Ok(buf) => {
                            let frame = buffer_to_byte_frame(
                                &buf,
                                width,
                                height,
                                format,
                            );
                            callback(frame);
                        }
                        Err(e) => {
                            error!("V4L2 read frame error: {:?}", e);
                            break;
                        }
                    }
                }

                let _ = cam.stop_streaming();
                info!("V4L2 streaming thread stopped");
            }
            #[cfg(not(feature = "v4l2"))]
            {
                *running.lock().unwrap() = false;
                warn!("V4L2 feature not enabled, streaming thread exiting");
            }
        });

        Ok(())
    }
}

impl ICameraAdapter for V4l2CameraAdapter {
    fn source_type(&self) -> CameraSourceType {
        self.base.source_type
    }

    fn open(&self) {
        // Opening is done lazily when streaming starts
        info!("V4L2 adapter open (path={})", self.device_path);
    }

    fn close(&self) {
        *self.running.lock().unwrap() = false;
        info!("V4L2 adapter close requested");
    }

    fn set_preview_surface(&self, _surface: *const std::ffi::c_void) {
        warn!("Preview surface not used in V4L2 adapter");
    }

    fn set_frame_callback(&self, callback: FrameCallback) {
        self.base.set_frame_callback(callback);
    }

    fn initialize(&self, on_ready: Box<dyn FnOnce() + Send>) {
        // Start streaming thread with current callback
        if let Err(e) = self.start_streaming_thread() {
            error!("Failed to start streaming thread: {}", e);
        } else {
            on_ready();
        }
    }
}
