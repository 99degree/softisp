//! Android-specific camera HAL components.
//! Currently provides only stub adapter for compatibility.

use cam_hal::camera::{ICameraAdapter, FrameCallback};
use cam_types::CameraSourceType;

/// Stub camera adapter (for testing / build without real camera).
pub struct StubAdapter;

impl ICameraAdapter for StubAdapter {
    fn source_type(&self) -> cam_types::CameraSourceType {
        CameraSourceType::RawCamera2
    }
    fn open(&self) {}
    fn close(&self) {}
    fn set_preview_surface(&self, _: *const std::ffi::c_void) {}
    fn set_frame_callback(&self, _: FrameCallback) {}
    fn initialize(&self, on_ready: Box<dyn FnOnce() + Send>) {
        on_ready();
    }
}

/// Create a stub camera adapter.
pub fn create_stub_adapter() -> Box<dyn ICameraAdapter> {
    Box::new(StubAdapter)
}
