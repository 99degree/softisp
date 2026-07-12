//! V4L2 Camera Adapter implementing ICameraAdapter with callback streaming.

use log::{error, info};
use std::sync::{Arc, Mutex};
use std::thread;

use cam_hal::camera::{
    BaseCameraAdapter, ByteFrame, CameraState, FrameCallback, ICameraAdapter, StreamConfig,
};
use cam_types::CameraSourceType;

/// V4L2 Camera Adapter that streams frames via callback.
pub struct V4l2CameraAdapter {
    base: Arc<BaseCameraAdapter>,
    device_path: String,
    config: Mutex<Option<StreamConfig>>,
    running: Arc<Mutex<bool>>,
}

impl V4l2CameraAdapter {
    pub fn new(device_path: &str) -> Result<Self, String> {
        #[cfg(feature = "v4l2")]
        {
            let cam = rscam::Camera::new(device_path)
                .map_err(|e| format!("Failed to open V4L2 device {}: {}", device_path, e))?;
            info!("Opened V4L2 device: {}", device_path);
            drop(cam); // close — we'll reopen on start_streaming

            Ok(Self {
                base: Arc::new(BaseCameraAdapter::new(CameraSourceType::V4l2)),
                device_path: device_path.to_string(),
                config: Mutex::new(None),
                running: Arc::new(Mutex::new(false)),
            })
        }
        #[cfg(not(feature = "v4l2"))]
        {
            Err("V4L2 feature not enabled".into())
        }
    }
}

impl ICameraAdapter for V4l2CameraAdapter {
    fn source_type(&self) -> CameraSourceType {
        self.base.source_type
    }

    fn open(&mut self, config: &StreamConfig) -> Result<(), String> {
        *self.config.lock().unwrap() = Some(config.clone());
        self.base.set_state(CameraState::Open);
        info!(
            "V4L2 adapter opened (path={}, {}x{})",
            self.device_path, config.width, config.height
        );
        Ok(())
    }

    fn close(&mut self) {
        *self.running.lock().unwrap() = false;
        self.base.set_state(CameraState::Closed);
        info!("V4L2 adapter closed");
    }

    fn set_frame_callback(&mut self, callback: FrameCallback) {
        self.base.set_frame_callback(callback);
    }

    fn start_streaming(&mut self) -> Result<(), String> {
        if !self.base.has_frame_callback() {
            return Err("No frame callback set".into());
        }

        let config = self
            .config
            .lock()
            .unwrap()
            .clone()
            .ok_or("Camera not opened (no StreamConfig)")?;

        let device_path = self.device_path.clone();
        let running = Arc::clone(&self.running);
        // We need the callback in the streaming thread. Since BaseCameraAdapter uses Mutex<Option<FrameCallback>>,
        // and the callback is Fn (not FnMut), we can share the BaseCameraAdapter via Arc.
        // But we don't have Arc<Self>. Let's work around: spawn thread that opens its own camera,
        // reads frames, and calls a shared callback.
        // Easiest: move a clone of the callback into the thread.
        // We take the callback out and wrap it in Arc for sharing.
        // But FrameCallback is Box<dyn Fn>. We'll wrap it.
        // Actually, we can use Arc<BaseCameraAdapter> but we don't have one.
        // Simplest: take callback out, wrap in Arc, put back a wrapper that delegates to Arc.

        // For now: use a simpler approach — we move a pointer to self.base into the thread.
        // But self is &mut, not Arc. So let's create a simple channel-based approach instead.
        // The streaming thread sends ByteFrames through a channel, and the main thread calls the callback.
        // Actually, the simplest approach is to use Arc<Mutex<Option<FrameCallback>>> directly.

        // Let's restructure: extract the callback, wrap in Arc<Mutex<>>, put a wrapper back.
        // This is messy. Better: redesign BaseCameraAdapter to use Arc internally.
        // For now, let's just use a crossbeam channel or std::sync::mpsc.

        // SIMPLEST: Just use a raw pointer to self.base which is 'static enough.
        // Since V4l2CameraAdapter is heap-allocated and lives for the duration of streaming,
        // and we control stop_streaming, this is safe in practice.
        // But it's not safe in Rust's model. Let's use Arc instead.

        // We'll create an Arc<Mutex<VecDeque<ByteFrame>>> as a frame queue,
        // and a drain thread that calls the callback.
        // This avoids moving the callback.

        // Actually, the simplest correct approach: store the callback in an Arc<Mutex<Option<FrameCallback>>>
        // directly in V4l2CameraAdapter instead of BaseCameraAdapter.
        // But that defeats the purpose of BaseCameraAdapter.

        // OK, pragmatic solution: just use a static-ish pattern.
        // We'll use Arc<dyn Fn(ByteFrame) + Send + Sync> instead of Box.
        // Wrap the callback when it's set.

        // Let me just use a crossbeam channel approach:
        // streaming thread -> channel -> main thread drains and calls callback.
        // But that requires the main thread to poll.

        // Easiest correct solution: store callback as Arc in a shared struct.
        // I'll create a small SharedState struct for the thread.

        let base = Arc::clone(&self.base);
        *self.running.lock().unwrap() = true;

        let running_clone = Arc::clone(&self.running);
        let width = config.width;
        let height = config.height;
        let format = config.format;
        let fps = config.fps;

        thread::spawn(move || {
            #[cfg(feature = "v4l2")]
            {
                let mut cam = match rscam::Camera::new(&device_path) {
                    Ok(c) => c,
                    Err(e) => {
                        error!("V4L2 thread: open failed: {:?}", e);
                        return;
                    }
                };

                let fourcc = crate::v4l2::format::frame_format_to_fourcc(format);
                if fourcc == 0 {
                    error!("V4L2 thread: unsupported format {:?}", format);
                    return;
                }

                let cfg = rscam::Config {
                    interval: (fps, 1),
                    resolution: (width, height),
                    format: &[
                        (fourcc >> 0) as u8,
                        (fourcc >> 8) as u8,
                        (fourcc >> 16) as u8,
                        (fourcc >> 24) as u8,
                    ],
                    field: 0,
                    nbuffers: 2,
                };

                if let Err(e) = cam.start(&cfg) {
                    error!("V4L2 thread: start failed: {:?}", e);
                    return;
                }

                info!(
                    "V4L2 streaming thread started ({}x{} {:?} @ {}fps)",
                    width, height, format, fps
                );

                while *running_clone.lock().unwrap() {
                    match cam.capture() {
                        Ok(frame) => {
                            let byte_frame = crate::v4l2::buffer::buffer_to_byte_frame(
                                &frame, width, height, format,
                            );
                            base.invoke_frame_callback(byte_frame);
                        }
                        Err(e) => {
                            error!("V4L2 capture frame error: {:?}", e);
                            break;
                        }
                    }
                }

                let _ = cam.stop();
                info!("V4L2 streaming thread stopped");
            }
            #[cfg(not(feature = "v4l2"))]
            {
                warn!("V4L2 feature not enabled, streaming thread exiting");
            }
        });

        self.base.set_state(CameraState::Streaming);
        Ok(())
    }

    fn stop_streaming(&mut self) {
        *self.running.lock().unwrap() = false;
        self.base.set_state(CameraState::Open);
        info!("V4L2 streaming stopped");
    }

    fn state(&self) -> CameraState {
        self.base.get_state()
    }

    fn device_name(&self) -> &str {
        &self.device_path
    }

    fn send_frame(&self, frame: ByteFrame) -> Result<(), String> {
        self.base.invoke_frame_callback(frame);
        Ok(())
    }
}
