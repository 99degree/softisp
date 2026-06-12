//! AIDL parcelable types for Camera HAL.
//!
//! Matches the real Android Camera HAL AIDL types:
//! - CameraDeviceStatus
//! - TorchModeStatus
//! - VendorTagSection / VendorTag
//! - StreamConfig
//! - StreamBuffer
//! - BufferRequest
//! - CaptureRequest
//! - CaptureResult
//! - CameraInfo
//! - ConcurrentCameraIdCombination
//! - CameraIdAndStreamCombination

use cam_types::FrameFormat;

// ── Device status ──

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum CameraDeviceStatus {
    NotPresent = 0,
    Present = 1,
    Enumerating = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum TorchModeStatus {
    Off = 0,
    On = 1,
}

// ── Vendor tags ──

#[derive(Debug, Clone)]
pub struct VendorTag {
    pub tag_id: i32,
    pub tag_name: String,
    pub tag_type: i32, // TYPE_BYTE, TYPE_INT32, etc.
}

#[derive(Debug, Clone)]
pub struct VendorTagSection {
    pub section_name: String,
    pub tags: Vec<VendorTag>,
}

// ── Stream configuration ──

#[derive(Debug, Clone)]
pub struct StreamConfig {
    pub stream_id: i32,
    pub width: i32,
    pub height: i32,
    pub format: i32,       // HAL pixel format
    pub buffer_count: i32,
    pub usage: i64,
    pub data_space: i32,
    pub rotation: i32,
}

impl StreamConfig {
    pub fn new(stream_id: i32, width: i32, height: i32, format: i32) -> Self {
        Self {
            stream_id,
            width,
            height,
            format,
            buffer_count: 4,
            usage: 0,
            data_space: 0,
            rotation: 0,
        }
    }

    /// Convert to cam-hal StreamConfig.
    pub fn to_hal_config(&self) -> cam_hal::camera::StreamConfig {
        cam_hal::camera::StreamConfig {
            width: self.width as u32,
            height: self.height as u32,
            format: FrameFormat::Rgba8888, // default
            fps: 30,
        }
    }
}

// ── Stream buffer ──

#[derive(Debug, Clone)]
pub struct StreamBuffer {
    pub stream_id: i32,
    pub buffer_id: i64,
    pub width: i32,
    pub height: i32,
    pub format: i32,
    pub stride: i32,
    pub data: Vec<u8>,        // pixel data (or empty if using dmabuf)
    pub timestamp_ns: i64,
    pub status: i32,          // 0 = ok
    pub frame_number: i64,
}

impl StreamBuffer {
    pub fn ok(stream_id: i32, width: i32, height: i32, data: Vec<u8>) -> Self {
        Self {
            stream_id,
            buffer_id: 0,
            width,
            height,
            format: 0x1, // HAL_PIXEL_FORMAT_RGBA_8888
            stride: width * 4,
            data,
            timestamp_ns: 0,
            status: 0,
            frame_number: 0,
        }
    }

    pub fn error(stream_id: i32, status: i32) -> Self {
        Self {
            stream_id,
            buffer_id: 0,
            width: 0,
            height: 0,
            format: 0,
            stride: 0,
            data: Vec::new(),
            timestamp_ns: 0,
            status,
            frame_number: 0,
        }
    }
}

// ── Buffer request ──

#[derive(Debug, Clone)]
pub struct BufferRequest {
    pub stream_id: i32,
    pub num_buffers: i32,
    pub frame_number: i64,
}

// ── Capture request ──

#[derive(Debug, Clone)]
pub struct CaptureRequest {
    pub frame_number: i64,
    pub template: i32,
    pub buffer_requests: Vec<BufferRequest>,
    // 3A settings
    pub control_mode: i32,
    pub ae_mode: i32,
    pub af_mode: i32,
    pub awb_mode: i32,
    pub exposure_time_ns: i64,
    pub sensitivity: i32,
    pub focus_distance: f32,
    pub zoom_ratio: f32,
    pub flash_mode: i32,
    pub target_fps: i32,
}

impl CaptureRequest {
    pub fn preview(frame_number: i64, stream_id: i32) -> Self {
        Self {
            frame_number,
            template: TEMPLATE_PREVIEW,
            buffer_requests: vec![BufferRequest {
                stream_id,
                num_buffers: 1,
                frame_number,
            }],
            control_mode: 1, // AUTO
            ae_mode: 1,      // ON
            af_mode: 4,      // CONTINUOUS_PICTURE
            awb_mode: 1,     // AUTO
            exposure_time_ns: 0,
            sensitivity: 0,
            focus_distance: 0.0,
            zoom_ratio: 1.0,
            flash_mode: 0,
            target_fps: 30,
        }
    }

    pub fn still_capture(frame_number: i64, stream_id: i32) -> Self {
        let mut req = Self::preview(frame_number, stream_id);
        req.template = TEMPLATE_STILL_CAPTURE;
        req
    }
}

// Template types (matching CameraDevice.TEMPLATE_*)
pub const TEMPLATE_PREVIEW: i32 = 1;
pub const TEMPLATE_STILL_CAPTURE: i32 = 2;
pub const TEMPLATE_VIDEO_RECORD: i32 = 3;
pub const TEMPLATE_VIDEO_SNAPSHOT: i32 = 4;
pub const TEMPLATE_ZERO_SHUTTER_LAG: i32 = 5;
pub const TEMPLATE_MANUAL: i32 = 6;

// ── Capture result ──

#[derive(Debug, Clone)]
pub struct CaptureResult {
    pub frame_number: i64,
    pub buffers: Vec<StreamBuffer>,
    pub exposure_time_ns: i64,
    pub sensitivity: i32,
    pub focus_distance: f32,
    pub timestamp_ns: i64,
    pub status: i32,
}

// ── Camera info ──

#[derive(Debug, Clone)]
pub struct CameraInfo {
    pub camera_id: String,
    pub facing: i32,           // LENS_FACING_BACK=1, LENS_FACING_FRONT=2
    pub orientation: i32,      // sensor orientation (0, 90, 180, 270)
    pub supported_formats: Vec<i32>,
    pub max_resolution: (i32, i32),
    pub hardware_level: i32,   // INFO_SUPPORTED_HARDWARE_LEVEL_*
}

// ── Concurrent camera ──

#[derive(Debug, Clone)]
pub struct ConcurrentCameraIdCombination {
    pub camera_ids: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct CameraIdAndStreamCombination {
    pub camera_id: String,
    pub stream_configs: Vec<StreamConfig>,
}

// ── Camera characteristics ──

#[derive(Debug, Clone)]
pub struct CameraCharacteristics {
    pub camera_id: String,
    pub info: CameraInfo,
    pub raw_metadata: Vec<u8>, // serialized CameraMetadata
}

// ── Device state ──

pub const DEVICE_STATE_NORMAL: i64 = 0;
pub const DEVICE_STATE_BACK_COVERED: i64 = 1;
pub const DEVICE_STATE_FRONT_COVERED: i64 = 2;
pub const DEVICE_STATE_FOLDED: i64 = 4;
