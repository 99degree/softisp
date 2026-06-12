//! Camera HAL Service.
//!
//! Entry point that creates the CameraProvider and serves binder requests.
//! On Android, registers as `android.hardware.camera.provider.ICameraProvider`
//! with the service manager.
//!
//! On Linux host, runs as a standalone local service that the demo app
//! connects to directly (no binder IPC needed).

use std::sync::Arc;

use log::info;

use crate::provider::CameraProvider;
use crate::session::CameraDeviceSession;
use crate::types::*;
use crate::callback::*;

/// Camera HAL Service.
///
/// Owns the `CameraProvider` and manages the service lifecycle.
pub struct CameraHalService {
    provider: Arc<CameraProvider>,
}

impl CameraHalService {
    pub fn new() -> Self {
        let provider = CameraProvider::new();
        Self {
            provider: Arc::new(provider),
        }
    }

    /// Get a reference to the provider.
    pub fn provider(&self) -> &Arc<CameraProvider> {
        &self.provider
    }

    /// Convenience: get camera ID list.
    pub fn get_camera_id_list(&self) -> Vec<String> {
        self.provider.get_camera_id_list()
    }

    /// Convenience: open a camera and return the session.
    pub fn open_camera(
        &self,
        camera_id: &str,
        callback: Arc<dyn ICameraDeviceCallback>,
    ) -> Result<Arc<std::sync::Mutex<CameraDeviceSession>>, String> {
        let device = self.provider.get_camera_device(camera_id)
            .ok_or_else(|| format!("Camera {} not found", camera_id))?;

        let session = device.lock().unwrap().open(callback)?;
        Ok(session)
    }

    /// Convenience: close a camera.
    pub fn close_camera(&self, camera_id: &str) {
        if let Some(device) = self.provider.get_camera_device(camera_id) {
            device.lock().unwrap().close();
        }
    }

    /// Run a capture session: configure streams, process requests.
    ///
    /// This is a high-level API for the demo app.
    pub fn capture(
        &self,
        camera_id: &str,
        width: i32,
        height: i32,
        num_frames: usize,
        frame_callback: Arc<dyn IFrameCallback>,
    ) -> Result<Vec<StreamBuffer>, String> {
        let callback = Arc::new(CaptureCallbackAdapter {
            frame_callback: frame_callback.clone(),
        });

        let session = self.open_camera(camera_id, callback)?;

        // Configure a single output stream
        let stream_config = StreamConfig::new(0, width, height, 0x1 /* RGBA_8888 */);
        let stream_ids = session.lock().unwrap().configure_streams(&[stream_config]);
        if stream_ids.is_empty() {
            return Err("Failed to configure streams".to_string());
        }

        // Process capture requests
        let mut all_buffers = Vec::new();
        for i in 0..num_frames {
            let request = CaptureRequest::preview(i as i64, 0);
            let buffers = session.lock().unwrap().process_capture_request(&request);
            for buf in &buffers {
                if buf.status == 0 {
                    frame_callback.on_frame(buf.clone());
                }
            }
            all_buffers.extend(buffers);
        }

        // Close session
        session.lock().unwrap().close();
        self.close_camera(camera_id);

        Ok(all_buffers)
    }

    /// Register as an Android binder service.
    #[cfg(feature = "android")]
    pub fn register_service(&self) -> Result<(), String> {
        // TODO: Use binder::BpServiceManager to register
        info!("CameraHalService: registering as media.camera (Android)");
        Ok(())
    }

    #[cfg(not(feature = "android"))]
    pub fn register_service(&self) -> Result<(), String> {
        info!("CameraHalService: local mode (no binder registration)");
        Ok(())
    }
}

impl Default for CameraHalService {
    fn default() -> Self {
        Self::new()
    }
}

/// Adapter: ICameraDeviceCallback -> IFrameCallback
struct CaptureCallbackAdapter {
    frame_callback: Arc<dyn IFrameCallback>,
}

impl ICameraDeviceCallback for CaptureCallbackAdapter {
    fn on_opened(&self, camera_id: &str) {
        info!("CaptureCallbackAdapter: camera {} opened", camera_id);
    }
    fn on_error(&self, error_code: i32, message: &str) {
        info!("CaptureCallbackAdapter: error {} - {}", error_code, message);
    }
    fn on_idle(&self) {}
    fn on_capture_result(&self, result: CaptureResult) {
        for buf in &result.buffers {
            if buf.status == 0 {
                self.frame_callback.on_frame(buf.clone());
            }
        }
    }
    fn on_request_queue_empty(&self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_service_creation() {
        let service = CameraHalService::new();
        let cameras = service.get_camera_id_list();
        assert!(!cameras.is_empty(), "Should have at least one camera (stub)");
    }

    #[test]
    fn test_capture_test_pattern() {
        struct TestCallback;
        impl IFrameCallback for TestCallback {
            fn on_frame(&self, buffer: StreamBuffer) {
                assert_eq!(buffer.status, 0);
                assert!(buffer.width > 0);
                assert!(buffer.height > 0);
                assert!(!buffer.data.is_empty());
            }
        }

        let service = CameraHalService::new();
        let result = service.capture("0", 320, 240, 3, Arc::new(TestCallback));
        assert!(result.is_ok());
        let buffers = result.unwrap();
        assert_eq!(buffers.len(), 3);
    }

    #[test]
    fn test_provider_api() {
        let service = CameraHalService::new();
        let ids = service.get_camera_id_list();
        assert!(ids.contains(&"0".to_string()));

        let info = service.provider().get_camera_info("0");
        assert!(info.is_some());
        let info = info.unwrap();
        assert_eq!(info.camera_id, "0");
    }

    #[test]
    fn test_open_close() {
        let service = CameraHalService::new();
        let callback = Arc::new(crate::callback::CameraDeviceCallbackNoop);
        let session = service.open_camera("0", callback);
        assert!(session.is_ok());
        service.close_camera("0");
    }
}
