//! Android-specific camera HAL components.
//! Currently provides only stub adapter for compatibility.

use cam_hal::camera::{ICameraAdapter, FrameCallback, CameraState, StreamConfig};
use cam_types::CameraSourceType;

/// Stub camera adapter (for testing / build without real camera).
pub struct StubAdapter {
    state: CameraState,
}

impl StubAdapter {
    pub fn new() -> Self {
        Self { state: CameraState::Closed }
    }
}

impl ICameraAdapter for StubAdapter {
    fn source_type(&self) -> CameraSourceType { CameraSourceType::RawCamera2 }

    fn open(&mut self, _config: &StreamConfig) -> Result<(), String> {
        self.state = CameraState::Open;
        Ok(())
    }

    fn close(&mut self) {
        self.state = CameraState::Closed;
    }

    fn start_streaming(&mut self) -> Result<(), String> {
        self.state = CameraState::Streaming;
        Ok(())
    }

    fn stop_streaming(&mut self) {
        self.state = CameraState::Open;
    }

    fn set_frame_callback(&mut self, _callback: FrameCallback) {}

    fn state(&self) -> CameraState { self.state }

    fn device_name(&self) -> &str { "stub" }
}

/// Create a stub camera adapter.
pub fn create_stub_adapter() -> Box<dyn ICameraAdapter> {
    Box::new(StubAdapter::new())
}
