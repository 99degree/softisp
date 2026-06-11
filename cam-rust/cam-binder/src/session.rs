//! Camera Device Session service.

use log::{info, warn, error};
use rsbinder::{Binder, Interface, Parcel, Status, StatusCode};

const PROCESS_CAPTURE_REQUEST: u32 = 1;
const FLUSH: u32 = 2;
const CLOSE: u32 = 3;

pub struct CameraDeviceSession {
    camera_id: String,
}

impl CameraDeviceSession {
    pub fn new(camera_id: String) -> Self {
        Self { camera_id }
    }
}

impl Interface for CameraDeviceSession {
    fn on_transact(&mut self, code: u32, msg: &mut Parcel, reply: &mut Parcel) -> Status<()> {
        match code {
            PROCESS_CAPTURE_REQUEST => {
                info!("CameraDeviceSession({}): processCaptureRequest", self.camera_id);
                // TODO: Parse CaptureRequest (stub)
                // For now, just return success.
                Status::success(())
            }
            FLUSH => {
                info!("CameraDeviceSession({}): flush", self.camera_id);
                Status::success(())
            }
            CLOSE => {
                info!("CameraDeviceSession({}): close", self.camera_id);
                Status::success(())
            }
            _ => {
                warn!("CameraDeviceSession: unknown transaction code: {}", code);
                Status::from_error(StatusCode::UNKNOWN_TRANSACTION)
            }
        }
    }
}
