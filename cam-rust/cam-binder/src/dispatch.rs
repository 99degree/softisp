//! AIDL Transaction Dispatch.
//!
//! Handles AIDL binder transaction codes and dispatches to the appropriate
//! Camera HAL methods. This is the core binder IPC handler that enables
//! Android framework integration.
//!
//! Transaction codes match the AIDL interface specification:
//! - ICameraProvider: 1-7
//! - ICameraDevice: 1-7
//! - ICameraDeviceSession: 1-12

use log::{info, warn};

use crate::device::CameraDevice;
use crate::metadata::{build_camera_characteristics, build_capture_result_metadata};
use crate::provider::CameraProvider;
use crate::session::CameraDeviceSession;
use crate::types::*;

/// AIDL transaction dispatch for ICameraProvider.
pub fn dispatch_provider_transaction(
    provider: &CameraProvider,
    code: i32,
    data: &[u8],
) -> Result<Vec<u8>, String> {
    match code {
        provider_transaction::SET_CALLBACK => {
            info!("Provider: SET_CALLBACK");
            Ok(Vec::new())
        }
        provider_transaction::GET_VENDOR_TAGS => {
            info!("Provider: GET_VENDOR_TAGS");
            // Return empty vendor tag sections
            Ok(Vec::new())
        }
        provider_transaction::GET_CAMERA_ID_LIST => {
            info!("Provider: GET_CAMERA_ID_LIST");
            let ids = provider.get_camera_id_list();
            // Serialize camera ID list
            let mut buf = Vec::new();
            buf.extend_from_slice(&(ids.len() as i32).to_ne_bytes());
            for id in &ids {
                buf.extend_from_slice(&(id.len() as i32).to_ne_bytes());
                buf.extend_from_slice(id.as_bytes());
            }
            Ok(buf)
        }
        provider_transaction::GET_CAMERA_DEVICE_INTERFACE => {
            info!("Provider: GET_CAMERA_DEVICE_INTERFACE");
            // Camera ID is in data
            if data.len() < 4 {
                return Err("missing camera ID".into());
            }
            let id_len = i32::from_ne_bytes(data[0..4].try_into().unwrap()) as usize;
            if data.len() < 4 + id_len {
                return Err("truncated camera ID".into());
            }
            let camera_id = String::from_utf8_lossy(&data[4..4 + id_len]).to_string();
            match provider.get_camera_device(&camera_id) {
                Some(_device) => Ok(Vec::new()),
                None => Err(format!("camera {} not found", camera_id)),
            }
        }
        provider_transaction::NOTIFY_DEVICE_STATE_CHANGE => {
            info!("Provider: NOTIFY_DEVICE_STATE_CHANGE");
            if data.len() >= 8 {
                let state = i64::from_ne_bytes(data[0..8].try_into().unwrap());
                provider.notify_device_state_change(state);
            }
            Ok(Vec::new())
        }
        provider_transaction::GET_CONCURRENT_CAMERA_IDS => {
            info!("Provider: GET_CONCURRENT_CAMERA_IDS");
            let combos = provider.get_concurrent_camera_ids();
            let mut buf = Vec::new();
            buf.extend_from_slice(&(combos.len() as i32).to_ne_bytes());
            for combo in &combos {
                buf.extend_from_slice(&(combo.camera_ids.len() as i32).to_ne_bytes());
                for id in &combo.camera_ids {
                    buf.extend_from_slice(&(id.len() as i32).to_ne_bytes());
                    buf.extend_from_slice(id.as_bytes());
                }
            }
            Ok(buf)
        }
        provider_transaction::IS_CONCURRENT_STREAM_COMBINATION_SUPPORTED => {
            info!("Provider: IS_CONCURRENT_STREAM_COMBINATION_SUPPORTED");
            // Return false by default
            Ok(vec![0u8])
        }
        _ => {
            warn!("Provider: unknown transaction code {}", code);
            Err(format!("unknown transaction: {}", code))
        }
    }
}

/// AIDL transaction dispatch for ICameraDevice.
pub fn dispatch_device_transaction(
    device: &CameraDevice,
    code: i32,
    data: &[u8],
) -> Result<Vec<u8>, String> {
    match code {
        device_transaction::GET_CAMERA_CHARACTERISTICS => {
            info!("Device({}): GET_CAMERA_CHARACTERISTICS", device.camera_id());
            let info = device.info();
            let meta = build_camera_characteristics(
                info.max_resolution.0,
                info.max_resolution.1,
                info.facing,
                info.orientation,
            );
            Ok(meta.to_bytes())
        }
        device_transaction::GET_RESOURCE_COST => {
            info!("Device({}): GET_RESOURCE_COST", device.camera_id());
            let cost = device.get_resource_cost();
            let mut buf = Vec::new();
            buf.extend_from_slice(&cost.cost.to_ne_bytes());
            buf.extend_from_slice(&(cost.conflict_devices.len() as i32).to_ne_bytes());
            for dev in &cost.conflict_devices {
                buf.extend_from_slice(&(dev.len() as i32).to_ne_bytes());
                buf.extend_from_slice(dev.as_bytes());
            }
            Ok(buf)
        }
        device_transaction::SET_TORCH_MODE => {
            info!("Device({}): SET_TORCH_MODE", device.camera_id());
            if data.len() >= 4 {
                let on = i32::from_ne_bytes(data[0..4].try_into().unwrap()) != 0;
                device.set_torch_mode(on);
            }
            Ok(Vec::new())
        }
        device_transaction::OPEN => {
            info!("Device({}): OPEN", device.camera_id());
            // The callback would be passed via binder in real AIDL
            // For now, return success
            Ok(Vec::new())
        }
        device_transaction::CLOSE => {
            info!("Device({}): CLOSE", device.camera_id());
            device.close();
            Ok(Vec::new())
        }
        _ => {
            warn!(
                "Device({}): unknown transaction {}",
                device.camera_id(),
                code
            );
            Err(format!("unknown transaction: {}", code))
        }
    }
}

/// AIDL transaction dispatch for ICameraDeviceSession.
pub fn dispatch_session_transaction(
    session: &CameraDeviceSession,
    code: i32,
    data: &[u8],
) -> Result<Vec<u8>, String> {
    match code {
        session_transaction::CONFIGURE_STREAMS => {
            info!("Session: CONFIGURE_STREAMS");
            // Parse stream configurations from data
            let configs = parse_stream_configs(data);
            let stream_ids = session.configure_streams(&configs);
            let mut buf = Vec::new();
            buf.extend_from_slice(&(stream_ids.len() as i32).to_ne_bytes());
            for id in &stream_ids {
                buf.extend_from_slice(&id.to_ne_bytes());
            }
            Ok(buf)
        }
        session_transaction::PROCESS_CAPTURE_REQUEST => {
            info!("Session: PROCESS_CAPTURE_REQUEST");
            let request = parse_capture_request(data);
            let buffers = session.process_capture_request(&request);
            let mut buf = Vec::new();
            buf.extend_from_slice(&(buffers.len() as i32).to_ne_bytes());
            for buffer in &buffers {
                buf.extend_from_slice(&buffer.status.to_ne_bytes());
                buf.extend_from_slice(&buffer.frame_number.to_ne_bytes());
            }
            Ok(buf)
        }
        session_transaction::FLUSH => {
            info!("Session: FLUSH");
            session.flush();
            Ok(Vec::new())
        }
        session_transaction::CLOSE => {
            info!("Session: CLOSE");
            session.close();
            Ok(Vec::new())
        }
        session_transaction::SIGNAL_STREAM_FLUSH => {
            info!("Session: SIGNAL_STREAM_FLUSH");
            if data.len() >= 12 {
                let stream_id = i32::from_ne_bytes(data[0..4].try_into().unwrap());
                let frame_number = i64::from_ne_bytes(data[4..12].try_into().unwrap());
                session.signal_stream_flush(stream_id, frame_number);
            }
            Ok(Vec::new())
        }
        session_transaction::SET_REPEATING_REQUEST => {
            info!("Session: SET_REPEATING_REQUEST");
            let request = parse_capture_request(data);
            session.set_repeating_request(&request)?;
            Ok(Vec::new())
        }
        session_transaction::GET_REQUEST_LIST => {
            info!("Session: GET_REQUEST_LIST");
            let requests = session.get_request_list();
            let mut buf = Vec::new();
            buf.extend_from_slice(&(requests.len() as i32).to_ne_bytes());
            Ok(buf)
        }
        _ => {
            warn!("Session: unknown transaction {}", code);
            Err(format!("unknown transaction: {}", code))
        }
    }
}

/// Parse stream configurations from AIDL parcel bytes.
fn parse_stream_configs(data: &[u8]) -> Vec<StreamConfig> {
    if data.len() < 4 {
        return Vec::new();
    }
    let count = i32::from_ne_bytes(data[0..4].try_into().unwrap()) as usize;
    let mut configs = Vec::new();
    let mut offset = 4;

    for _ in 0..count {
        if offset + 16 > data.len() {
            break;
        }
        let stream_id = i32::from_ne_bytes(data[offset..offset + 4].try_into().unwrap());
        let width = i32::from_ne_bytes(data[offset + 4..offset + 8].try_into().unwrap());
        let height = i32::from_ne_bytes(data[offset + 8..offset + 12].try_into().unwrap());
        let format = i32::from_ne_bytes(data[offset + 12..offset + 16].try_into().unwrap());
        configs.push(StreamConfig::new(stream_id, width, height, format));
        offset += 16;
    }

    configs
}

/// Parse capture request from AIDL parcel bytes.
fn parse_capture_request(data: &[u8]) -> CaptureRequest {
    if data.len() < 8 {
        return CaptureRequest::preview(0, 0);
    }
    let frame_number = i64::from_ne_bytes(data[0..8].try_into().unwrap());
    let stream_id = if data.len() >= 12 {
        i32::from_ne_bytes(data[8..12].try_into().unwrap())
    } else {
        0
    };
    CaptureRequest::preview(frame_number, stream_id)
}

/// Build capture result metadata bytes.
pub fn build_result_metadata(
    frame_number: i64,
    ae_state: i32,
    af_state: i32,
    awb_state: i32,
    exposure_ns: i64,
    sensitivity: i32,
) -> Vec<u8> {
    let meta = build_capture_result_metadata(
        frame_number,
        ae_state,
        af_state,
        awb_state,
        exposure_ns,
        sensitivity,
    );
    meta.to_bytes()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_stream_configs() {
        let mut data = Vec::new();
        data.extend_from_slice(&2i32.to_ne_bytes()); // count = 2
        data.extend_from_slice(&0i32.to_ne_bytes()); // stream_id = 0
        data.extend_from_slice(&1920i32.to_ne_bytes()); // width
        data.extend_from_slice(&1080i32.to_ne_bytes()); // height
        data.extend_from_slice(&0x1i32.to_ne_bytes()); // format = RGBA
        data.extend_from_slice(&1i32.to_ne_bytes()); // stream_id = 1
        data.extend_from_slice(&640i32.to_ne_bytes()); // width
        data.extend_from_slice(&480i32.to_ne_bytes()); // height
        data.extend_from_slice(&0x23i32.to_ne_bytes()); // format = YUV

        let configs = parse_stream_configs(&data);
        assert_eq!(configs.len(), 2);
        assert_eq!(configs[0].width, 1920);
        assert_eq!(configs[1].width, 640);
    }

    #[test]
    fn test_parse_capture_request() {
        let mut data = Vec::new();
        data.extend_from_slice(&42i64.to_ne_bytes()); // frame_number
        data.extend_from_slice(&0i32.to_ne_bytes()); // stream_id

        let req = parse_capture_request(&data);
        assert_eq!(req.frame_number, 42);
    }

    #[test]
    fn test_build_result_metadata() {
        let bytes = build_result_metadata(1, 2, 4, 2, 33333333, 400);
        assert!(!bytes.is_empty());
        assert!(bytes.len() > 12); // At least header
    }
}
