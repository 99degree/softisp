//! Callback interfaces for Camera HAL.
//!
//! - ICameraProviderCallback
//! - ICameraDeviceCallback
//! - IFrameCallback

use crate::types::{CameraDeviceStatus, TorchModeStatus, CaptureResult};

/// Callback from camera provider to framework.
///
/// Matches `android.hardware.camera.provider.ICameraProviderCallback`.
pub trait ICameraProviderCallback: Send + Sync {
    fn on_camera_device_status_change(&self, camera_id: &str, new_status: CameraDeviceStatus);
    fn on_torch_mode_status_change(&self, camera_id: &str, new_status: TorchModeStatus);
    fn on_physical_camera_device_status_change(
        &self,
        camera_id: &str,
        physical_camera_id: &str,
        new_status: CameraDeviceStatus,
    );
}

/// Default no-op implementation.
pub struct CameraProviderCallbackNoop;

impl ICameraProviderCallback for CameraProviderCallbackNoop {
    fn on_camera_device_status_change(&self, _camera_id: &str, _new_status: CameraDeviceStatus) {}
    fn on_torch_mode_status_change(&self, _camera_id: &str, _new_status: TorchModeStatus) {}
    fn on_physical_camera_device_status_change(
        &self,
        _camera_id: &str,
        _physical_camera_id: &str,
        _new_status: CameraDeviceStatus,
    ) {}
}

/// Callback from camera device to client.
///
/// Matches `android.hardware.camera.device.ICameraDeviceCallback`.
pub trait ICameraDeviceCallback: Send + Sync {
    fn on_opened(&self, camera_id: &str);
    fn on_error(&self, error_code: i32, message: &str);
    fn on_idle(&self);
    fn on_capture_result(&self, result: CaptureResult);
    fn on_request_queue_empty(&self);
}

/// Default no-op implementation.
pub struct CameraDeviceCallbackNoop;

impl ICameraDeviceCallback for CameraDeviceCallbackNoop {
    fn on_opened(&self, _camera_id: &str) {}
    fn on_error(&self, _error_code: i32, _message: &str) {}
    fn on_idle(&self) {}
    fn on_capture_result(&self, _result: CaptureResult) {}
    fn on_request_queue_empty(&self) {}
}

/// Frame callback for the demo app.
pub trait IFrameCallback: Send + Sync {
    fn on_frame(&self, buffer: crate::types::StreamBuffer);
}
