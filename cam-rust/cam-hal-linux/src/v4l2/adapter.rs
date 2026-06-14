//! V4L2 Camera Adapter implementing ICameraAdapter with callback streaming.

use log::{info, warn, error};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::Duration;

use cam_types::FrameFormat;
use cam_hal::camera::{ByteFrame, CameraState, FrameCallback, ICameraAdapter, StreamConfig};

/// V4L2 Camera Adapter that streams frames via callback.
pub struct V4l2CameraAdapter {
    device_path: String,
    config: Option<StreamConfig>,
    callback: Option<FrameCallback>,
    running: Arc<Mutex<bool>>,
    state: Arc<Mutex<CameraState>>,
    thread_handle: Option<thread::JoinHandle<()>>,
}

impl V4l2CameraAdapter {
    pub fn new(device_path: &str) -> Result<Self, String> {
        #[cfg(feature = "v4l2")]
        {
            let cam = rscam::Camera::new(device_path)
                .map_err(|e| format!("Failed to open V4L2 device {}: {}", device_path, e))?;
            let info = cam.query_capability()
                .map_err(|e| format!("Failed to query capabilities: {}", e))?;
            info!("V4L2 device: driver={}, card={}",
                String::from_utf8_lossy(&info.driver).trim_end_matches('\0'),
                String::from_utf8_lossy(&info.card).trim_end_matches('\0'));
            drop(cam);

            Ok(Self {
                device_path: device_path.to_string(),
                config: None,
                callback: None,
                running: Arc::new(Mutex::new(false)),
                state: Arc::new(Mutex::new(CameraState::Closed)),
                thread_handle: None,
            })
        }
        #[cfg(not(feature = "v4l2"))]
        { Err("V4L2 feature not enabled".into()) }
    }
}

impl ICameraAdapter for V4l2CameraAdapter {
    fn source_type(&self) -> cam_types::CameraSourceType {
        cam_types::CameraSourceType::V4l2
    }

    fn open(&mut self, config: &StreamConfig) -> Result<(), String> {
        self.config = Some(config.clone());
        *self.state.lock().unwrap() = CameraState::Open;
        info!("V4L2 adapter opened (path={}, {}x{} @ {}fps)",
            self.device_path, config.width, config.height, config.fps);
        Ok(())
    }

    fn close(&mut self) {
        self.stop_streaming();
        *self.state.lock().unwrap() = CameraState::Closed;
        info!("V4L2 adapter closed");
    }

    fn set_frame_callback(&mut self, callback: FrameCallback) {
        self.callback = Some(callback);
    }

    fn start_streaming(&mut self) -> Result<(), String> {
        if self.callback.is_none() {
            return Err("No frame callback set".into());
        }

        let config = self.config.clone()
            .ok_or("Camera not opened (no StreamConfig)")?;

        *self.running.lock().unwrap() = true;
        *self.state.lock().unwrap() = CameraState::Streaming;

        let device_path = self.device_path.clone();
        let running = Arc::clone(&self.running);
        let state = Arc::clone(&self.state);

        // Extract callback and wrap for thread safety
        // We use an Arc<Mutex<Option<FrameCallback>>> for the thread
        let cb = self.callback.take().unwrap();
        let shared_cb: Arc<Mutex<Option<FrameCallback>>> = Arc::new(Mutex::new(Some(cb)));
        let shared_cb_clone = Arc::clone(&shared_cb);

        // Put the callback back via a wrapper that stores into shared_cb
        let shared_cb2 = Arc::clone(&shared_cb);
        self.callback = Some(Box::new(move |frame: ByteFrame| {
            if let Some(ref cb) = *shared_cb2.lock().unwrap() {
                cb(frame);
            }
        }));

        let width = config.width;
        let height = config.height;
        let format = config.format;
        let fps = config.fps;

        let handle = thread::spawn(move || {
            #[cfg(feature = "v4l2")]
            {
                let mut cam = match rscam::Camera::new(&device_path) {
                    Ok(c) => c,
                    Err(e) => {
                        error!("V4L2 thread: open failed: {:?}", e);
                        *state.lock().unwrap() = CameraState::Error;
                        return;
                    }
                };

                let fourcc = crate::v4l2::format::frame_format_to_fourcc(format);
                if fourcc == 0 {
                    error!("V4L2 thread: unsupported format {:?}", format);
                    *state.lock().unwrap() = CameraState::Error;
                    return;
                }

                let mut cfg = rscam::Config::new();
                cfg.resolution(width, height)
                   .format(fourcc)
                   .frame_rate(fps, 1);

                if let Err(e) = cam.configure(&cfg) {
                    error!("V4L2 thread: configure failed: {:?}", e);
                    *state.lock().unwrap() = CameraState::Error;
                    return;
                }

                if let Err(e) = cam.start_streaming(4) {
                    error!("V4L2 thread: start_streaming failed: {:?}", e);
                    *state.lock().unwrap() = CameraState::Error;
                    return;
                }

                info!("V4L2 streaming: {}x{} {}fps", width, height, fps);

                while *running.lock().unwrap() {
                    match cam.read_frame() {
                        Ok(buf) => {
                            let frame = ByteFrame {
                                data: buf.data.to_vec(),
                                width,
                                height,
                                format,
                                timestamp: 0,
                            };
                            if let Some(ref cb) = *shared_cb_clone.lock().unwrap() {
                                cb(frame);
                            }
                        }
                        Err(e) => {
                            error!("V4L2 read error: {:?}", e);
                            break;
                        }
                    }
                }

                let _ = cam.stop_streaming();
                info!("V4L2 streaming stopped");
            }
            #[cfg(not(feature = "v4l2"))]
            {
                warn!("V4L2 feature not enabled");
            }
        });

        self.thread_handle = Some(handle);
        Ok(())
    }

    fn stop_streaming(&mut self) {
        *self.running.lock().unwrap() = false;
        if let Some(handle) = self.thread_handle.take() {
            let _ = handle.join();
        }
        *self.state.lock().unwrap() = CameraState::Open;
        info!("V4L2 streaming stopped");
    }

    fn state(&self) -> CameraState {
        *self.state.lock().unwrap()
    }

    fn device_name(&self) -> &str {
        &self.device_path
    }

    fn send_frame(&self, _frame: ByteFrame) -> Result<(), String> {
        Err("V4L2 adapter does not support send_frame".into())
    }
}

impl Drop for V4l2CameraAdapter {
    fn drop(&mut self) {
        self.close();
    }
}
