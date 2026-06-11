//! Camera Provider service using rsbinder.

use log::{info, warn, error};
use rsbinder::{Binder, Interface, Parcel, Status, StatusCode};
use crate::device;
use crate::session;

const GET_CAMERA_ID_LIST: u32 = 1;
const GET_CAMERA_DEVICE: u32 = 2;
const SET_CALLBACK: u32 = 3;
const GET_VENDOR_TAGS: u32 = 4;
const NOTIFY_DEVICE_STATE: u32 = 5;

pub struct CameraProviderService {
    cameras: Vec<String>,
}

impl CameraProviderService {
    pub fn new() -> Self {
        Self {
            cameras: vec!["0".to_string(), "1".to_string()],
        }
    }
}

impl Interface for CameraProviderService {
    fn on_transact(&mut self, code: u32, msg: &mut Parcel, reply: &mut Parcel) -> Status<()> {
        match code {
            GET_CAMERA_ID_LIST => {
                info!("CameraProvider: getCameraIdList");
                // Write string list as: count (i32), then each string as length (i32) + bytes
                let count = self.cameras.len() as i32;
                if reply.write_i32(count).is_err() {
                    return Status::from_error(StatusCode::UNKNOWN_ERROR);
                }
                for cam in &self.cameras {
                    if reply.write_string(cam).is_err() {
                        return Status::from_error(StatusCode::UNKNOWN_ERROR);
                    }
                }
                Status::success(())
            }
            GET_CAMERA_DEVICE => {
                let camera_id: String = match msg.read_string() {
                    Ok(s) => s,
                    Err(_) => {
                        error!("Failed to read cameraId from request");
                        return Status::from_error(StatusCode::BAD_VALUE);
                    }
                };
                info!("CameraProvider: getCameraDevice({})", camera_id);
                // For now, always return device for camera 0
                if camera_id != "0" && camera_id != "1" {
                    error!("Invalid camera ID: {}", camera_id);
                    return Status::from_error(StatusCode::BAD_VALUE);
                }
                let device = device::CameraDeviceService::new(camera_id);
                let binder = Binder::new(device);
                reply.write_binder(binder).map_err(|_| StatusCode::UNKNOWN_ERROR)?;
                Status::success(())
            }
            SET_CALLBACK => {
                // callback: ICameraProviderCallback
                let _callback: Option<Binder<dyn ICameraProviderCallback>> = match msg.read_binder() {
                    Ok(b) => b,
                    Err(e) => {
                        error!("Failed to read callback: {:?}", e);
                        return Status::from_error(StatusCode::BAD_VALUE);
                    }
                };
                info!("CameraProvider: setCallback (ignored)");
                // store if needed
                Status::success(())
            }
            GET_VENDOR_TAGS => {
                warn!("CameraProvider: getVendorTags not implemented");
                reply.write_i32(0).map_err(|_| StatusCode::UNKNOWN_ERROR)?;
                Status::success(())
            }
            NOTIFY_DEVICE_STATE => {
                let _state: i64 = match msg.read_i64() {
                    Ok(s) => s,
                    Err(_) => {
                        error!("Failed to read device state");
                        return Status::from_error(StatusCode::BAD_VALUE);
                    }
                };
                info!("CameraProvider: notifyDeviceStateChange({})", _state);
                Status::success(())
            }
            _ => {
                warn!("Unknown transaction code: {}", code);
                Status::from_error(StatusCode::UNKNOWN_TRANSACTION)
            }
        }
    }
}

// Stub callback trait
pub trait ICameraProviderCallback: Interface {}
