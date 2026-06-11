//! Camera Device service.

use log::{info, warn};
use rsbinder::{Binder, Interface, Parcel, Status, StatusCode};
use crate::session;

const OPEN: u32 = 1;
const GET_CHARACTERISTICS: u32 = 2;
const CLOSE: u32 = 3;

pub struct CameraDeviceService {
    camera_id: String,
}

impl CameraDeviceService {
    pub fn new(camera_id: String) -> Self {
        Self { camera_id }
    }
}

impl Interface for CameraDeviceService {
    fn on_transact(&mut self, code: u32, msg: &mut Parcel, reply: &mut Parcel) -> Status<()> {
        match code {
            OPEN => {
                info!("CameraDevice({}): open", self.camera_id);
                // Return a session binder
                let session = session::CameraDeviceSession::new(self.camera_id.clone());
                let binder = Binder::new(session);
                reply.write_binder(binder).map_err(|_| StatusCode::UNKNOWN_ERROR)?;
                Status::success(())
            }
            GET_CHARACTERISTICS => {
                info!("CameraDevice({}): getCharacteristics", self.camera_id);
                // Return a stub CameraMetadata as raw bytes (empty)
                // In real impl, would be a Parcelable CameraMetadata
                reply.write_i32(0); // empty byte array length
                Status::success(())
            }
            CLOSE => {
                info!("CameraDevice({}): close", self.camera_id);
                // nothing to clean
                Status::success(())
            }
            _ => {
                warn!("CameraDevice: unknown transaction code: {}", code);
                Status::from_error(StatusCode::UNKNOWN_TRANSACTION)
            }
        }
    }
}
