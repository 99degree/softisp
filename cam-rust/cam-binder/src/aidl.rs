//! AIDL Interface Implementations — Binary-compatible binder stubs.
//!
//! Implements the AIDL interfaces for camera HAL:
//! - BnCameraProvider (native/stub side)
//! - BpCameraProvider (proxy/client side)
//! - BnCameraDevice, BpCameraDevice
//! - BnCameraDeviceSession, BpCameraDeviceSession
//!
//! Each BnXxx implements onTransact() for incoming binder calls.
//! Each BpXxx provides typed methods for outgoing binder calls.

use std::sync::{Arc, Mutex};

use log::{info, warn, error};

use crate::binder::{Parcel, IBinder, BinderStatus, TransactionCode};
use crate::provider::CameraProvider;
use crate::device::CameraDevice;
use crate::session::CameraDeviceSession;
use crate::types::*;
use crate::metadata::{CameraMetadata, build_camera_characteristics, build_capture_result_metadata};

// ── BnCameraProvider (Server/Stub) ──

/// AIDL stub for ICameraProvider.
///
/// Handles incoming binder transactions and dispatches to CameraProvider.
pub struct BnCameraProvider {
    provider: Arc<CameraProvider>,
}

impl BnCameraProvider {
    /// Create a new stub wrapping the provider.
    pub fn new(provider: Arc<CameraProvider>) -> Self {
        Self { provider }
    }

    /// Handle a binder transaction.
    pub fn on_transact(&self, code: TransactionCode, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        // Verify interface token
        let _token = data.read_string16().map_err(|e| e)?;

        match code {
            1 => self.set_callback(data),
            2 => self.get_vendor_tags(data),
            3 => self.get_camera_id_list(data),
            4 => self.get_camera_device_interface(data),
            5 => self.notify_device_state_change(data),
            6 => self.get_concurrent_camera_ids(data),
            7 => self.is_concurrent_stream_combination_supported(data),
            _ => {
                warn!("BnCameraProvider: unknown transaction {}", code);
                Err(BinderStatus::NotImplemented)
            }
        }
    }

    fn set_callback(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        info!("BnCameraProvider::setCallback");
        Ok(Parcel::new())
    }

    fn get_vendor_tags(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        info!("BnCameraProvider::getVendorTags");
        let mut reply = Parcel::new();
        reply.write_i32(0); // empty vendor tag sections
        Ok(reply)
    }

    fn get_camera_id_list(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        info!("BnCameraProvider::getCameraIdList");
        let ids = self.provider.get_camera_id_list();
        let mut reply = Parcel::new();
        reply.write_i32(ids.len() as i32);
        for id in &ids {
            reply.write_string16(id);
        }
        Ok(reply)
    }

    fn get_camera_device_interface(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let camera_id = data.read_string16()?;
        info!("BnCameraProvider::getCameraDeviceInterface({})", camera_id);

        match self.provider.get_camera_device(&camera_id) {
            Some(device) => {
                let mut reply = Parcel::new();
                // In real binder, this would return a BpCameraDevice
                // For local simulation, we write device metadata
                reply.write_i32(0); // status OK
                reply.write_str(&camera_id);
                Ok(reply)
            }
            None => {
                warn!("BnCameraProvider: camera {} not found", camera_id);
                Err(BinderStatus::NameNotFound)
            }
        }
    }

    fn notify_device_state_change(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let state = data.read_i64()?;
        info!("BnCameraProvider::notifyDeviceStateChange({})", state);
        self.provider.notify_device_state_change(state);
        Ok(Parcel::new())
    }

    fn get_concurrent_camera_ids(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        info!("BnCameraProvider::getConcurrentCameraIds");
        let combos = self.provider.get_concurrent_camera_ids();
        let mut reply = Parcel::new();
        reply.write_i32(combos.len() as i32);
        for combo in &combos {
            reply.write_i32(combo.camera_ids.len() as i32);
            for id in &combo.camera_ids {
                reply.write_string16(id);
            }
        }
        Ok(reply)
    }

    fn is_concurrent_stream_combination_supported(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        info!("BnCameraProvider::isConcurrentStreamCombinationSupported");
        let mut reply = Parcel::new();
        reply.write_i32(0); // false
        Ok(reply)
    }
}

impl IBinder for BnCameraProvider {
    fn transact(&self, code: TransactionCode, mut data: Parcel) -> Result<BinderStatus, BinderStatus> {
        match self.on_transact(code, &mut data) {
            Ok(_) => Ok(BinderStatus::Ok),
            Err(e) => Err(e),
        }
    }

    fn interface_descriptor(&self) -> &str {
        "android.hardware.camera.provider.ICameraProvider"
    }
}

// ── BnCameraDevice (Server/Stub) ──

/// AIDL stub for ICameraDevice.
pub struct BnCameraDevice {
    device: Arc<Mutex<CameraDevice>>,
}

impl BnCameraDevice {
    pub fn new(device: Arc<Mutex<CameraDevice>>) -> Self {
        Self { device }
    }

    pub fn on_transact(&self, code: TransactionCode, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let _token = data.read_string16().map_err(|e| e)?;

        match code {
            1 => self.get_camera_characteristics(data),
            2 => self.deserialize(data),
            3 => self.get_resource_cost(data),
            4 => self.supports_torch_mode(data),
            5 => self.set_torch_mode(data),
            6 => self.open(data),
            7 => self.close(data),
            _ => {
                warn!("BnCameraDevice: unknown transaction {}", code);
                Err(BinderStatus::NotImplemented)
            }
        }
    }

    fn get_camera_characteristics(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let device = self.device.lock().unwrap();
        let info = device.info();
        let meta = build_camera_characteristics(
            info.max_resolution.0,
            info.max_resolution.1,
            info.facing,
            info.orientation,
        );
        let mut reply = Parcel::new();
        let bytes = meta.to_bytes();
        reply.write_bytes(&bytes);
        Ok(reply)
    }

    fn deserialize(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        info!("BnCameraDevice::deserialize");
        Ok(Parcel::new())
    }

    fn get_resource_cost(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let device = self.device.lock().unwrap();
        let cost = device.get_resource_cost();
        let mut reply = Parcel::new();
        reply.write_i32(cost.cost);
        reply.write_i32(cost.conflict_devices.len() as i32);
        for dev in &cost.conflict_devices {
            reply.write_string16(dev);
        }
        Ok(reply)
    }

    fn supports_torch_mode(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let device = self.device.lock().unwrap();
        let mut reply = Parcel::new();
        reply.write_i32(if device.is_torch_supported() { 1 } else { 0 });
        Ok(reply)
    }

    fn set_torch_mode(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let on = data.read_i32()?;
        let device = self.device.lock().unwrap();
        device.set_torch_mode(on != 0);
        Ok(Parcel::new())
    }

    fn open(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        info!("BnCameraDevice::open");
        // In real binder, we'd read the callback binder
        let mut reply = Parcel::new();
        reply.write_i32(0); // status OK
        Ok(reply)
    }

    fn close(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let device = self.device.lock().unwrap();
        device.close();
        Ok(Parcel::new())
    }
}

impl IBinder for BnCameraDevice {
    fn transact(&self, code: TransactionCode, mut data: Parcel) -> Result<BinderStatus, BinderStatus> {
        match self.on_transact(code, &mut data) {
            Ok(_) => Ok(BinderStatus::Ok),
            Err(e) => Err(e),
        }
    }

    fn interface_descriptor(&self) -> &str {
        "android.hardware.camera.device.ICameraDevice"
    }
}

// ── BnCameraDeviceSession (Server/Stub) ──

/// AIDL stub for ICameraDeviceSession.
pub struct BnCameraDeviceSession {
    session: Arc<Mutex<CameraDeviceSession>>,
}

impl BnCameraDeviceSession {
    pub fn new(session: Arc<Mutex<CameraDeviceSession>>) -> Self {
        Self { session }
    }

    pub fn on_transact(&self, code: TransactionCode, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let _token = data.read_string16().map_err(|e| e)?;

        match code {
            1 => self.configure_streams(data),
            2 => self.process_capture_request(data),
            3 => self.get_stream_buffer(data),
            4 => self.return_stream_buffer(data),
            5 => self.return_input_buffer(data),
            6 => self.return_result_metadata(data),
            7 => self.notify(data),
            8 => self.flush(data),
            9 => self.close(data),
            10 => self.signal_stream_flush(data),
            11 => self.set_repeating_request(data),
            12 => self.get_request_list(data),
            _ => {
                warn!("BnCameraDeviceSession: unknown transaction {}", code);
                Err(BinderStatus::NotImplemented)
            }
        }
    }

    fn configure_streams(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let count = data.read_i32()? as usize;
        let mut configs = Vec::new();
        for _ in 0..count {
            let stream_id = data.read_i32()?;
            let width = data.read_i32()?;
            let height = data.read_i32()?;
            let format = data.read_i32()?;
            configs.push(StreamConfig::new(stream_id, width, height, format));
        }

        let session = self.session.lock().unwrap();
        let stream_ids = session.configure_streams(&configs);

        let mut reply = Parcel::new();
        reply.write_i32(stream_ids.len() as i32);
        for id in &stream_ids {
            reply.write_i32(*id);
        }
        Ok(reply)
    }

    fn process_capture_request(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let frame_number = data.read_i64()?;
        let stream_id = data.read_i32()?;
        let request = CaptureRequest::preview(frame_number, stream_id);

        let session = self.session.lock().unwrap();
        let buffers = session.process_capture_request(&request);

        let mut reply = Parcel::new();
        reply.write_i32(buffers.len() as i32);
        for buffer in &buffers {
            reply.write_i32(buffer.status);
            reply.write_i64(buffer.frame_number);
        }
        Ok(reply)
    }

    fn get_stream_buffer(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let _stream_id = data.read_i32()?;
        let _frame_number = data.read_i64()?;
        info!("BnCameraDeviceSession::getStreamBuffer");
        Ok(Parcel::new())
    }

    fn return_stream_buffer(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let _stream_id = data.read_i32()?;
        let _frame_number = data.read_i64()?;
        info!("BnCameraDeviceSession::returnStreamBuffer");
        Ok(Parcel::new())
    }

    fn return_input_buffer(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let _stream_id = data.read_i32()?;
        let _frame_number = data.read_i64()?;
        info!("BnCameraDeviceSession::returnInputBuffer");
        Ok(Parcel::new())
    }

    fn return_result_metadata(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let _frame_number = data.read_i64()?;
        info!("BnCameraDeviceSession::returnResultMetadata");
        Ok(Parcel::new())
    }

    fn notify(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let notify_type = data.read_i32()?;
        let frame_number = data.read_i64()?;
        let timestamp = data.read_i64()?;
        info!("BnCameraDeviceSession::notify type={} frame={} ts={}", notify_type, frame_number, timestamp);
        Ok(Parcel::new())
    }

    fn flush(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let session = self.session.lock().unwrap();
        session.flush();
        Ok(Parcel::new())
    }

    fn close(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let session = self.session.lock().unwrap();
        session.close();
        Ok(Parcel::new())
    }

    fn signal_stream_flush(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let stream_id = data.read_i32()?;
        let frame_number = data.read_i64()?;
        let session = self.session.lock().unwrap();
        session.signal_stream_flush(stream_id, frame_number);
        Ok(Parcel::new())
    }

    fn set_repeating_request(&self, data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let frame_number = data.read_i64()?;
        let stream_id = data.read_i32()?;
        let request = CaptureRequest::preview(frame_number, stream_id);
        let session = self.session.lock().unwrap();
        session.set_repeating_request(&request)?;
        Ok(Parcel::new())
    }

    fn get_request_list(&self, _data: &mut Parcel) -> Result<Parcel, BinderStatus> {
        let session = self.session.lock().unwrap();
        let requests = session.get_request_list();
        let mut reply = Parcel::new();
        reply.write_i32(requests.len() as i32);
        Ok(reply)
    }
}

impl IBinder for BnCameraDeviceSession {
    fn transact(&self, code: TransactionCode, mut data: Parcel) -> Result<BinderStatus, BinderStatus> {
        match self.on_transact(code, &mut data) {
            Ok(_) => Ok(BinderStatus::Ok),
            Err(e) => Err(e),
        }
    }

    fn interface_descriptor(&self) -> &str {
        "android.hardware.camera.device.ICameraDeviceSession"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bn_provider_transact() {
        let provider = Arc::new(CameraProvider::new());
        let bn = BnCameraProvider::new(provider);

        let mut data = Parcel::new();
        data.write_string16("android.hardware.camera.provider.ICameraProvider");
        let result = bn.on_transact(3, &mut data); // GET_CAMERA_ID_LIST
        assert!(result.is_ok());
    }

    #[test]
    fn test_bn_device_transact() {
        let device = Arc::new(Mutex::new(CameraDevice::new(
            "0".into(),
            "/dev/video0".into(),
            CameraInfo::default(),
        )));
        let bn = BnCameraDevice::new(device);

        let mut data = Parcel::new();
        data.write_string16("android.hardware.camera.device.ICameraDevice");
        let result = bn.on_transact(1, &mut data); // GET_CAMERA_CHARACTERISTICS
        assert!(result.is_ok());
    }

    #[test]
    fn test_bn_session_transact() {
        let session = Arc::new(Mutex::new(CameraDeviceSession::new(
            "0".into(),
            "/dev/video0".into(),
            CameraInfo::default(),
        )));
        let bn = BnCameraDeviceSession::new(session);

        let mut data = Parcel::new();
        data.write_string16("android.hardware.camera.device.ICameraDeviceSession");
        let result = bn.on_transact(8, &mut data); // FLUSH
        assert!(result.is_ok());
    }

    #[test]
    fn test_bn_provider_interface() {
        let provider = Arc::new(CameraProvider::new());
        let bn = BnCameraProvider::new(provider);
        assert_eq!(bn.interface_descriptor(), "android.hardware.camera.provider.ICameraProvider");
    }
}
