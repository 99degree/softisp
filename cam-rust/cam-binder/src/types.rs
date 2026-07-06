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

// ── Buffer Usage Flags (matching AOSP GrallocUsage.h) ──

/// Buffer usage flags for Gralloc allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BufferUsage(pub i64);

impl BufferUsage {
    pub const CPU_READ_RARELY: i64 = 0x00000002;
    pub const CPU_READ_OFTEN: i64 = 0x00000003;
    pub const CPU_WRITE_RARELY: i64 = 0x00000020;
    pub const CPU_WRITE_OFTEN: i64 = 0x00000030;
    pub const GPU_TEXTURE: i64 = 0x00000100;
    pub const GPU_RENDER_TARGET: i64 = 0x00000200;
    pub const COMPOSER_OVERLAY: i64 = 0x00000800;
    pub const COMPOSER_CLIENT_TARGET: i64 = 0x00001000;
    pub const VIDEO_DECODER: i64 = 0x00004000;
    pub const VIDEO_ENCODER: i64 = 0x00010000;
    pub const CAMERA_READ: i64 = 0x00020000;
    pub const CAMERA_WRITE: i64 = 0x00040000;
    pub const HW_COMPOSER: i64 = 0x00080000;
    pub const PREDICTED: i64 = 0x00200000;
    pub const SENSOR_DIRECT: i64 = 0x00400000;
    pub const GPU_DATA_SPACE: i64 = 0x01000000;
    pub const GPU_MIPMAP: i64 = 0x02000000;
    pub const HW_IMAGE_ENCODER: i64 = 0x04000000;
}

// ── DataSpace (matching AOSP DataSpace.h) ──

/// Color data space for stream buffers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataSpace(pub i32);

impl DataSpace {
    pub const UNKNOWN: i32 = 0;
    pub const STANDARD_UNSPECIFIED: i32 = 0x00000001;
    pub const STANDARD_BT601_625: i32 = 0x00000004;
    pub const STANDARD_BT601_525: i32 = 0x00000005;
    pub const STANDARD_BT709: i32 = 0x00000006;
    pub const STANDARD_BT2020: i32 = 0x00000007;
    pub const STANDARD_BT470M: i32 = 0x00000008;
    pub const STANDARD_FILM: i32 = 0x0000000a;
    pub const RANGE_UNSPECIFIED: i32 = 0x00000000;
    pub const RANGE_FULL: i32 = 0x00000100;
    pub const RANGE_LIMITED: i32 = 0x00000200;
    pub const RANGE_EXTENDED: i32 = 0x00000300;
    pub const TRANSFER_UNSPECIFIED: i32 = 0x00000000;
    pub const TRANSFER_LINEAR: i32 = 0x00001000;
    pub const TRANSFER_SRGB: i32 = 0x00002000;
    pub const TRANSFER_ST2084: i32 = 0x00003000;
    pub const TRANSFER_HLG: i32 = 0x00004000;
    pub const DPY_P3: i32 = DataSpace::STANDARD_DCI_P3 | DataSpace::RANGE_FULL | DataSpace::TRANSFER_SRGB;
    pub const STANDARD_DCI_P3: i32 = 0x00000009;
    pub const STANDARD_ADOBE_RGB: i32 = 0x0000000b;
    pub const V0_SRGB: i32 = DataSpace::STANDARD_BT709 | DataSpace::RANGE_FULL | DataSpace::TRANSFER_SRGB;
    pub const V0_JFIF: i32 = DataSpace::STANDARD_BT601_625 | DataSpace::RANGE_FULL | DataSpace::TRANSFER_UNSPECIFIED;
    pub const BT2020_PQ: i32 = DataSpace::STANDARD_BT2020 | DataSpace::RANGE_FULL | DataSpace::TRANSFER_ST2084;
}

// ── Stream Combination ──

/// Stream combination for concurrent camera query.
#[derive(Debug, Clone)]
pub struct StreamCombination {
    pub stream_id: i32,
    pub format: i32,
    pub width: i32,
    pub height: i32,
    pub is_input: bool,
}

impl StreamCombination {
    pub fn output(stream_id: i32, format: i32, width: i32, height: i32) -> Self {
        Self { stream_id, format, width, height, is_input: false }
    }

    pub fn input(stream_id: i32, format: i32, width: i32, height: i32) -> Self {
        Self { stream_id, format, width, height, is_input: true }
    }
}

// ── HAL Pixel Formats ──

/// HAL pixel format constants (matching AOSP HAL_PIXEL_FORMAT_*).
#[allow(non_camel_case_types)]
pub mod hal_pixel_format {
    pub const UNKNOWN: i32 = 0;
    pub const RGBA_8888: i32 = 0x1;
    pub const RGBX_8888: i32 = 0x2;
    pub const RGB_888: i32 = 0x3;
    pub const RGB_565: i32 = 0x4;
    pub const BGR_888: i32 = 0x5;
    pub const YCbCr_422_SP: i32 = 0x10;
    pub const YCrCb_420_SP: i32 = 0x11;
    pub const YCbCr_422_P: i32 = 0x12;
    pub const YCrCb_422_P: i32 = 0x13;
    pub const YCbCr_420_SP: i32 = 0x14;
    pub const YCbCr_420_P: i32 = 0x15;
    pub const YCrCb_422_I: i32 = 0x1E;
    pub const YCbCr_422_I: i32 = 0x1F;
    pub const Blob: i32 = 0x21;
    pub const IMPLEMENTATION_DEFINED: i32 = 0x22;
    pub const YCbCr_420_888: i32 = 0x23;
    pub const YV12: i32 = 0x32355659;
    pub const RAW16: i32 = 0x20;
    pub const RAW10: i32 = 0x25;
    pub const RAW8: i32 = 0x26;
    pub const BLOB_: i32 = 0x21;
    pub const RAW_SINGLE: i32 = 0x28;
    pub const RAW_PRIVATE: i32 = 0x2C;
}

// ── AIDL Transaction Codes ──

/// AIDL transaction codes for ICameraProvider.
pub mod provider_transaction {
    pub const SET_CALLBACK: i32 = 1;
    pub const GET_VENDOR_TAGS: i32 = 2;
    pub const GET_CAMERA_ID_LIST: i32 = 3;
    pub const GET_CAMERA_DEVICE_INTERFACE: i32 = 4;
    pub const NOTIFY_DEVICE_STATE_CHANGE: i32 = 5;
    pub const GET_CONCURRENT_CAMERA_IDS: i32 = 6;
    pub const IS_CONCURRENT_STREAM_COMBINATION_SUPPORTED: i32 = 7;
}

/// AIDL transaction codes for ICameraDevice.
pub mod device_transaction {
    pub const GET_CAMERA_CHARACTERISTICS: i32 = 1;
    pub const DESERIALIZE: i32 = 2;
    pub const GET_RESOURCE_COST: i32 = 3;
    pub const SUPPORTS_TORCH_MODE: i32 = 4;
    pub const SET_TORCH_MODE: i32 = 5;
    pub const OPEN: i32 = 6;
    pub const CLOSE: i32 = 7;
}

/// AIDL transaction codes for ICameraDeviceSession.
pub mod session_transaction {
    pub const CONFIGURE_STREAMS: i32 = 1;
    pub const PROCESS_CAPTURE_REQUEST: i32 = 2;
    pub const GET_STREAM_BUFFER: i32 = 3;
    pub const RETURN_STREAM_BUFFER: i32 = 4;
    pub const RETURN_INPUT_BUFFER: i32 = 5;
    pub const RETURN_RESULT_METADATA: i32 = 6;
    pub const NOTIFY: i32 = 7;
    pub const FLUSH: i32 = 8;
    pub const CLOSE: i32 = 9;
    pub const SIGNAL_STREAM_FLUSH: i32 = 10;
    pub const SET_REPEATING_REQUEST: i32 = 11;
    pub const GET_REQUEST_LIST: i32 = 12;
}

// ── Notify Type (for capture result notifications) ──

/// Capture notification type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NotifyType {
    /// Shutter event (timestamp available).
    Shutter = 0,
    /// Error (buffer error or device error).
    Error = 1,
}

/// Capture error code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureErrorCode {
    /// No error.
    NoError = 0,
    /// Buffer not ready.
    BufferError = 1,
    /// Device-level error.
    DeviceError = 2,
    /// Request error (couldn't process this request).
    RequestError = 3,
    /// Result error (metadata lost).
    ResultError = 4,
    /// Buffer lost.
    BufferLostError = 5,
}

/// Capture notification (shutter or error).
#[derive(Debug, Clone)]
pub struct CaptureNotification {
    pub notify_type: NotifyType,
    pub frame_number: i64,
    pub timestamp_ns: i64,
    pub stream_id: Option<i32>,
    pub error_code: Option<CaptureErrorCode>,
}

impl CaptureNotification {
    pub fn shutter(frame_number: i64, timestamp_ns: i64) -> Self {
        Self {
            notify_type: NotifyType::Shutter,
            frame_number,
            timestamp_ns,
            stream_id: None,
            error_code: None,
        }
    }

    pub fn error(frame_number: i64, stream_id: i32, code: CaptureErrorCode) -> Self {
        Self {
            notify_type: NotifyType::Error,
            frame_number,
            timestamp_ns: 0,
            stream_id: Some(stream_id),
            error_code: Some(code),
        }
    }
}

// ── Resource Cost ──

/// Camera resource cost.
#[derive(Debug, Clone)]
pub struct CameraResourceCost {
    pub cost: i32,          // 0-100
    pub conflict_devices: Vec<String>,
}
