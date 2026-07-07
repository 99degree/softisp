//! # CameraMetadata — AOSP-Compatible Metadata Parcelable
//!
//! Implements `android.hardware.camera.metadata.CameraMetadata` from AOSP.
//! Contains static camera characteristics and per-frame capture results.
//!
//! ## Overview
//!
//! CameraMetadata is the standard container for camera parameters in Android.
//! It stores key-value pairs where keys are tag IDs and values are typed arrays.
//!
//! ## Structure
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │ Header                                  │
//! │   version: u32                          │
//! │   entry_count: u32                      │
//! │   data_size: u32                        │
//! ├─────────────────────────────────────────┤
//! │ Entries (sorted by tag_id)              │
//! │   tag_id: u32                           │
//! │   count: u32                            │
//! │   type: u32 (0=Byte,1=Int32,2=Float...) │
//! │   offset: u32                           │
//! ├─────────────────────────────────────────┤
//! │ Data Section                            │
//! │   [values...]                           │
//! └─────────────────────────────────────────┘
//! ```
//!
//! ## Tag IDs
//!
//! Common AOSP tag IDs:
//!
//! | Tag | Description |
//! |-----|-------------|
//! | `ANDROID_CONTROL_AE_MODE` | Auto-exposure mode |
//! | `ANDROID_CONTROL_AF_MODE` | Auto-focus mode |
//! | `ANDROID_CONTROL_AWB_MODE` | Auto-whitebalance mode |
//! | `ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE` | Sensor active area |
//! | `ANDROID_LENS_FACING` | Lens facing direction |
//!
//! ## Usage
//!
//! ```rust,ignore
//! use cam_binder::metadata::{CameraMetadata, MetadataEntry};
//!
//! // Create metadata
//! let mut meta = CameraMetadata::new();
//! meta.set(MetadataEntry::new_int32(0x01, &[1, 2, 3]));
//!
//! // Serialize
//! let bytes = meta.to_bytes();
//!
//! // Deserialize
//! let restored = CameraMetadata::from_bytes(&bytes).unwrap();
//! ```

use std::collections::HashMap;

/// Metadata tag type (matching AOSP CameraMetadataType).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum MetadataType {
    Byte = 0,
    Int32 = 1,
    Float = 2,
    Int64 = 3,
    Double = 4,
    Rational = 5,
}

/// Single metadata entry.
#[derive(Debug, Clone)]
pub struct MetadataEntry {
    pub tag_id: u32,
    pub data_type: MetadataType,
    pub count: u32,
    pub value: Vec<u8>,
}

impl MetadataEntry {
    pub fn new_int32(tag_id: u32, values: &[i32]) -> Self {
        let mut value = Vec::with_capacity(values.len() * 4);
        for v in values {
            value.extend_from_slice(&v.to_ne_bytes());
        }
        Self {
            tag_id,
            data_type: MetadataType::Int32,
            count: values.len() as u32,
            value,
        }
    }

    pub fn new_int64(tag_id: u32, values: &[i64]) -> Self {
        let mut value = Vec::with_capacity(values.len() * 8);
        for v in values {
            value.extend_from_slice(&v.to_ne_bytes());
        }
        Self {
            tag_id,
            data_type: MetadataType::Int64,
            count: values.len() as u32,
            value,
        }
    }

    pub fn new_float(tag_id: u32, values: &[f32]) -> Self {
        let mut value = Vec::with_capacity(values.len() * 4);
        for v in values {
            value.extend_from_slice(&v.to_ne_bytes());
        }
        Self {
            tag_id,
            data_type: MetadataType::Float,
            count: values.len() as u32,
            value,
        }
    }

    pub fn new_byte(tag_id: u32, values: &[u8]) -> Self {
        Self {
            tag_id,
            data_type: MetadataType::Byte,
            count: values.len() as u32,
            value: values.to_vec(),
        }
    }

    pub fn new_rational(tag_id: u32, numerators: &[i32], denominators: &[i32]) -> Self {
        assert_eq!(numerators.len(), denominators.len());
        let mut value = Vec::with_capacity(numerators.len() * 8);
        for (n, d) in numerators.iter().zip(denominators.iter()) {
            value.extend_from_slice(&n.to_ne_bytes());
            value.extend_from_slice(&d.to_ne_bytes());
        }
        Self {
            tag_id,
            data_type: MetadataType::Rational,
            count: numerators.len() as u32,
            value,
        }
    }

    pub fn to_int32_slice(&self) -> Vec<i32> {
        self.value.chunks_exact(4)
            .map(|c| i32::from_ne_bytes([c[0], c[1], c[2], c[3]]))
            .collect()
    }

    pub fn to_int64_slice(&self) -> Vec<i64> {
        self.value.chunks_exact(8)
            .map(|c| i64::from_ne_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]))
            .collect()
    }

    pub fn to_float_slice(&self) -> Vec<f32> {
        self.value.chunks_exact(4)
            .map(|c| f32::from_ne_bytes([c[0], c[1], c[2], c[3]]))
            .collect()
    }
}

/// CameraMetadata — AOSP-compatible metadata container.
///
/// Contains camera characteristics (static) or per-frame capture results.
/// Stored as a flat byte buffer matching AOSP's `camera_metadata_t`.
#[derive(Debug, Clone)]
pub struct CameraMetadata {
    /// Version (always 2 for AIDL).
    pub version: u32,
    /// Number of metadata entries.
    pub entry_count: u32,
    /// Data section size in bytes.
    pub data_size: u32,
    /// Individual metadata entries.
    entries: HashMap<u32, MetadataEntry>,
}

impl Default for CameraMetadata {
    fn default() -> Self {
        Self::new()
    }
}

impl CameraMetadata {
    pub fn new() -> Self {
        Self {
            version: 2,
            entry_count: 0,
            data_size: 0,
            entries: HashMap::new(),
        }
    }

    /// Add a metadata entry.
    pub fn set(&mut self, entry: MetadataEntry) {
        self.data_size += entry.value.len() as u32;
        self.entry_count += 1;
        self.entries.insert(entry.tag_id, entry);
    }

    /// Get a metadata entry by tag ID.
    pub fn get(&self, tag_id: u32) -> Option<&MetadataEntry> {
        self.entries.get(&tag_id)
    }

    /// Get int32 values for a tag.
    pub fn get_int32(&self, tag_id: u32) -> Option<Vec<i32>> {
        self.entries.get(&tag_id).map(|e| e.to_int32_slice())
    }

    /// Get int64 values for a tag.
    pub fn get_int64(&self, tag_id: u32) -> Option<Vec<i64>> {
        self.entries.get(&tag_id).map(|e| e.to_int64_slice())
    }

    /// Get float values for a tag.
    pub fn get_float(&self, tag_id: u32) -> Option<Vec<f32>> {
        self.entries.get(&tag_id).map(|e| e.to_float_slice())
    }

    /// Check if a tag exists.
    pub fn has(&self, tag_id: u32) -> bool {
        self.entries.contains_key(&tag_id)
    }

    /// Get all tag IDs.
    pub fn tag_ids(&self) -> Vec<u32> {
        self.entries.keys().copied().collect()
    }

    /// Get entry count.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Serialize to AOSP camera_metadata_t format.
    ///
    /// Layout:
    ///   \[0\] u32: version
    ///   \[1\] u32: entry_count
    ///   \[2\] u32: data_size
    ///   \[3..3+entry_count*4\] entries (tag_id:u32, count:u32, type:u32, offset:u32)
    ///   \[...\] data section
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::new();

        // Header
        buf.extend_from_slice(&self.version.to_ne_bytes());
        buf.extend_from_slice(&(self.entry_count).to_ne_bytes());
        buf.extend_from_slice(&self.data_size.to_ne_bytes());

        // Entries (sorted by tag_id for deterministic output)
        let mut sorted_entries: Vec<_> = self.entries.values().collect();
        sorted_entries.sort_by_key(|e| e.tag_id);

        let mut data_offset: u32 = 0;
        for entry in &sorted_entries {
            buf.extend_from_slice(&entry.tag_id.to_ne_bytes());
            buf.extend_from_slice(&entry.count.to_ne_bytes());
            buf.extend_from_slice(&(entry.data_type as u32).to_ne_bytes());
            buf.extend_from_slice(&data_offset.to_ne_bytes());
            data_offset += entry.value.len() as u32;
        }

        // Data section
        for entry in &sorted_entries {
            buf.extend_from_slice(&entry.value);
        }

        buf
    }

    /// Deserialize from AOSP camera_metadata_t format.
    pub fn from_bytes(data: &[u8]) -> Result<Self, String> {
        if data.len() < 12 {
            return Err("metadata too short".into());
        }

        let version = u32::from_ne_bytes([data[0], data[1], data[2], data[3]]);
        let entry_count = u32::from_ne_bytes([data[4], data[5], data[6], data[7]]);
        let data_size = u32::from_ne_bytes([data[8], data[9], data[10], data[11]]);

        let header_size = 12;
        let entries_size = entry_count as usize * 16;
        let expected = header_size + entries_size + data_size as usize;
        if data.len() < expected {
            return Err(format!("metadata too short: need {}, got {}", expected, data.len()));
        }

        let mut entries = HashMap::new();
        let mut offset = header_size;

        for _ in 0..entry_count {
            let tag_id = u32::from_ne_bytes(data[offset..offset+4].try_into().unwrap());
            let count = u32::from_ne_bytes(data[offset+4..offset+8].try_into().unwrap());
            let data_type = u32::from_ne_bytes(data[offset+8..offset+12].try_into().unwrap());
            let value_offset = u32::from_ne_bytes(data[offset+12..offset+16].try_into().unwrap());
            offset += 16;

            let type_size = match data_type {
                0 => 1,  // Byte
                1 => 4,  // Int32
                2 => 4,  // Float
                3 => 8,  // Int64
                4 => 8,  // Double
                5 => 8,  // Rational (i32 numerator + i32 denominator)
                _ => return Err(format!("unknown type: {}", data_type)),
            };

            let value_start = header_size + entries_size + value_offset as usize;
            let value_end = value_start + (count as usize * type_size);
            let value = data[value_start..value_end].to_vec();

            entries.insert(tag_id, MetadataEntry {
                tag_id,
                data_type: match data_type {
                    0 => MetadataType::Byte,
                    1 => MetadataType::Int32,
                    2 => MetadataType::Float,
                    3 => MetadataType::Int64,
                    4 => MetadataType::Double,
                    5 => MetadataType::Rational,
                    _ => unreachable!(),
                },
                count,
                value,
            });
        }

        Ok(Self {
            version,
            entry_count,
            data_size,
            entries,
        })
    }
}

// ── AOSP Camera Metadata Tag IDs ──

/// ANDROID_CONTROL_AE_MODE
pub const ANDROID_CONTROL_AE_MODE: u32 = 0x00000009;
/// ANDROID_CONTROL_AE_STATE
pub const ANDROID_CONTROL_AE_STATE: u32 = 0x0000000f;
/// ANDROID_CONTROL_AF_MODE
pub const ANDROID_CONTROL_AF_MODE: u32 = 0x00000022;
/// ANDROID_CONTROL_AF_STATE
pub const ANDROID_CONTROL_AF_STATE: u32 = 0x00000023;
/// ANDROID_CONTROL_AWB_MODE
pub const ANDROID_CONTROL_AWB_MODE: u32 = 0x00000034;
/// ANDROID_CONTROL_AWB_STATE
pub const ANDROID_CONTROL_AWB_STATE: u32 = 0x00000035;
/// ANDROID_CONTROL_MODE
pub const ANDROID_CONTROL_MODE: u32 = 0x00000001;
/// ANDROID_CONTROL_AVAILABLE_MODES
pub const ANDROID_CONTROL_AVAILABLE_MODES: u32 = 0x00000002;

/// ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE
pub const ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE: u32 = 0x00000032;
/// ANDROID_SENSOR_INFO_SENSITIVITY_RANGE
pub const ANDROID_SENSOR_INFO_SENSITIVITY_RANGE: u32 = 0x00000037;
/// ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE
pub const ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE: u32 = 0x00000036;
/// ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE
pub const ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE: u32 = 0x00000034;
/// ANDROID_SENSOR_ORIENTATION
pub const ANDROID_SENSOR_ORIENTATION: u32 = 0x0000003c;

/// ANDROID_LENS_FACING
pub const ANDROID_LENS_FACING: u32 = 0x00000061;
/// ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS
pub const ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS: u32 = 0x00000062;

/// ANDROID_REQUEST_AVAILABLE_CAPABILITIES
pub const ANDROID_REQUEST_AVAILABLE_CAPABILITIES: u32 = 0x00000034;
/// ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS
pub const ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS: u32 = 0x0000003b;
/// ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS
pub const ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS: u32 = 0x0000003c;

/// ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS
pub const ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS: u32 = 0x00000060;
/// ANDROID_SCALER_AVAILABLE_FORMATS
pub const ANDROID_SCALER_AVAILABLE_FORMATS: u32 = 0x00000002;

/// ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL
pub const ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL: u32 = 0x00000011;

// ── Hardware levels ──
pub const INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY: i32 = 0;
pub const INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED: i32 = 1;
pub const INFO_SUPPORTED_HARDWARE_LEVEL_FULL: i32 = 2;
pub const INFO_SUPPORTED_HARDWARE_LEVEL_3: i32 = 3;
pub const INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL: i32 = 4;

// ── AE modes ──
pub const CONTROL_AE_MODE_OFF: i32 = 0;
pub const CONTROL_AE_MODE_ON: i32 = 1;
pub const CONTROL_AE_MODE_ON_AUTO_FLASH: i32 = 2;
pub const CONTROL_AE_MODE_ON_ALWAYS_FLASH: i32 = 3;
pub const CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE: i32 = 4;

// ── AF modes ──
pub const CONTROL_AF_MODE_OFF: i32 = 0;
pub const CONTROL_AF_MODE_AUTO: i32 = 1;
pub const CONTROL_AF_MODE_MACRO: i32 = 2;
pub const CONTROL_AF_MODE_CONTINUOUS_VIDEO: i32 = 3;
pub const CONTROL_AF_MODE_CONTINUOUS_PICTURE: i32 = 4;
pub const CONTROL_AF_MODE_EDOF: i32 = 5;

// ── AWB modes ──
pub const CONTROL_AWB_MODE_OFF: i32 = 0;
pub const CONTROL_AWB_MODE_AUTO: i32 = 1;
pub const CONTROL_AWB_MODE_INCANDESCENT: i32 = 2;
pub const CONTROL_AWB_MODE_FLUORESCENT: i32 = 3;
pub const CONTROL_AWB_MODE_WARM_FLUORESCENT: i32 = 4;
pub const CONTROL_AWB_MODE_DAYLIGHT: i32 = 5;
pub const CONTROL_AWB_MODE_CLOUDY_DAYLIGHT: i32 = 6;
pub const CONTROL_AWB_MODE_TWILIGHT: i32 = 7;
pub const CONTROL_AWB_MODE_SHADE: i32 = 8;

// ── AE states ──
pub const CONTROL_AE_STATE_INACTIVE: i32 = 0;
pub const CONTROL_AE_STATE_SEARCHING: i32 = 1;
pub const CONTROL_AE_STATE_CONVERGED: i32 = 2;
pub const CONTROL_AE_STATE_LOCKED: i32 = 3;
pub const CONTROL_AE_STATE_FLASH_REQUIRED: i32 = 4;
pub const CONTROL_AE_STATE_PRECAPTURE: i32 = 5;

// ── AF states ──
pub const CONTROL_AF_STATE_INACTIVE: i32 = 0;
pub const CONTROL_AF_STATE_PASSIVE_SCAN: i32 = 1;
pub const CONTROL_AF_STATE_PASSIVE_FOCUSED: i32 = 2;
pub const CONTROL_AF_STATE_ACTIVE_SCAN: i32 = 3;
pub const CONTROL_AF_STATE_FOCUSED_LOCKED: i32 = 4;
pub const CONTROL_AF_STATE_NOT_FOCUSED_LOCKED: i32 = 5;
pub const CONTROL_AF_STATE_PASSIVE_UNFOCUSED: i32 = 6;

// ── AWB states ──
pub const CONTROL_AWB_STATE_INACTIVE: i32 = 0;
pub const CONTROL_AWB_STATE_SEARCHING: i32 = 1;
pub const CONTROL_AWB_STATE_CONVERGED: i32 = 2;
pub const CONTROL_AWB_STATE_LOCKED: i32 = 3;

// ── Lens facing ──
pub const LENS_FACING_BACK: i32 = 0;
pub const LENS_FACING_FRONT: i32 = 1;
pub const LENS_FACING_EXTERNAL: i32 = 2;

/// Build a standard camera characteristics metadata for the given resolution.
pub fn build_camera_characteristics(
    width: i32,
    height: i32,
    facing: i32,
    orientation: i32,
) -> CameraMetadata {
    let mut meta = CameraMetadata::new();

    // Control
    meta.set(MetadataEntry::new_int32(ANDROID_CONTROL_MODE, &[1])); // AUTO
    meta.set(MetadataEntry::new_int32(ANDROID_CONTROL_AE_MODE, &[1])); // ON
    meta.set(MetadataEntry::new_int32(ANDROID_CONTROL_AF_MODE, &[4])); // CONTINUOUS_PICTURE
    meta.set(MetadataEntry::new_int32(ANDROID_CONTROL_AWB_MODE, &[1])); // AUTO
    meta.set(MetadataEntry::new_int32(ANDROID_CONTROL_AVAILABLE_MODES, &[0, 1, 2, 3, 4]));

    // Sensor
    meta.set(MetadataEntry::new_int32(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE, &[0, 0, width, height]));
    meta.set(MetadataEntry::new_int32(ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE, &[width, height]));
    meta.set(MetadataEntry::new_int32(ANDROID_SENSOR_ORIENTATION, &[orientation]));
    meta.set(MetadataEntry::new_int32(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, &[100, 3200]));
    meta.set(MetadataEntry::new_int64(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, &[100000, 300000000])); // 100µs - 300ms

    // Lens
    meta.set(MetadataEntry::new_int32(ANDROID_LENS_FACING, &[facing]));
    meta.set(MetadataEntry::new_float(ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS, &[3.5]));

    // Request
    meta.set(MetadataEntry::new_int32(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS, &[1]));
    meta.set(MetadataEntry::new_int32(ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS, &[3, 1, 0]));

    // Scaler (stream configurations: format, width, height, input?)
    // Format 0x23 = YUV_420_888, 0x1 = RGBA_8888
    meta.set(MetadataEntry::new_int32(ANDROID_SCALER_AVAILABLE_FORMATS, &[0x23, 0x1, 0x20]));
    meta.set(MetadataEntry::new_int32(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &[
        0x23, width, height, 0,  // YUV 420 output
        0x1, width, height, 0,   // RGBA output
        0x20, width, height, 0,  // BLOB output
    ]));

    // Info
    meta.set(MetadataEntry::new_int32(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL, &[INFO_SUPPORTED_HARDWARE_LEVEL_FULL]));

    meta
}

/// Build per-frame capture result metadata.
pub fn build_capture_result_metadata(
    _frame_number: i64,
    ae_state: i32,
    af_state: i32,
    awb_state: i32,
    exposure_time_ns: i64,
    sensitivity: i32,
) -> CameraMetadata {
    let mut meta = CameraMetadata::new();

    meta.set(MetadataEntry::new_int32(ANDROID_CONTROL_AE_STATE, &[ae_state]));
    meta.set(MetadataEntry::new_int32(ANDROID_CONTROL_AF_STATE, &[af_state]));
    meta.set(MetadataEntry::new_int32(ANDROID_CONTROL_AWB_STATE, &[awb_state]));
    meta.set(MetadataEntry::new_int64(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, &[exposure_time_ns]));
    meta.set(MetadataEntry::new_int32(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, &[sensitivity]));

    meta
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_metadata_new() {
        let meta = CameraMetadata::new();
        assert!(meta.is_empty());
        assert_eq!(meta.len(), 0);
    }

    #[test]
    fn test_metadata_set_get_int32() {
        let mut meta = CameraMetadata::new();
        meta.set(MetadataEntry::new_int32(0x01, &[1, 2, 3]));
        let vals = meta.get_int32(0x01).unwrap();
        assert_eq!(vals, vec![1, 2, 3]);
    }

    #[test]
    fn test_metadata_set_get_float() {
        let mut meta = CameraMetadata::new();
        meta.set(MetadataEntry::new_float(0x02, &[1.5, 2.5]));
        let vals = meta.get_float(0x02).unwrap();
        assert!((vals[0] - 1.5).abs() < 0.001);
        assert!((vals[1] - 2.5).abs() < 0.001);
    }

    #[test]
    fn test_metadata_set_get_int64() {
        let mut meta = CameraMetadata::new();
        meta.set(MetadataEntry::new_int64(0x03, &[1000000000]));
        let vals = meta.get_int64(0x03).unwrap();
        assert_eq!(vals, vec![1000000000]);
    }

    #[test]
    fn test_metadata_has() {
        let mut meta = CameraMetadata::new();
        assert!(!meta.has(0x01));
        meta.set(MetadataEntry::new_int32(0x01, &[1]));
        assert!(meta.has(0x01));
    }

    #[test]
    fn test_metadata_serialize_roundtrip() {
        let mut meta = CameraMetadata::new();
        meta.set(MetadataEntry::new_int32(0x01, &[1, 2, 3]));
        meta.set(MetadataEntry::new_float(0x02, &[1.5]));
        meta.set(MetadataEntry::new_int64(0x03, &[42]));

        let bytes = meta.to_bytes();
        let restored = CameraMetadata::from_bytes(&bytes).unwrap();

        assert_eq!(restored.len(), 3);
        assert_eq!(restored.get_int32(0x01), Some(vec![1, 2, 3]));
        let floats = restored.get_float(0x02).unwrap();
        assert!((floats[0] - 1.5).abs() < 0.001);
        assert_eq!(restored.get_int64(0x03), Some(vec![42]));
    }

    #[test]
    fn test_metadata_empty_roundtrip() {
        let meta = CameraMetadata::new();
        let bytes = meta.to_bytes();
        let restored = CameraMetadata::from_bytes(&bytes).unwrap();
        assert!(restored.is_empty());
    }

    #[test]
    fn test_metadata_characteristics() {
        let meta = build_camera_characteristics(1920, 1080, LENS_FACING_BACK, 90);
        assert!(!meta.is_empty());
        assert!(meta.has(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE));
        assert!(meta.has(ANDROID_LENS_FACING));
        assert!(meta.has(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL));
    }

    #[test]
    fn test_metadata_capture_result() {
        let meta = build_capture_result_metadata(
            1,
            CONTROL_AE_STATE_CONVERGED,
            CONTROL_AF_STATE_FOCUSED_LOCKED,
            CONTROL_AWB_STATE_CONVERGED,
            33333333,  // 30fps
            400,
        );
        assert!(!meta.is_empty());
        assert_eq!(meta.get_int32(ANDROID_CONTROL_AE_STATE), Some(vec![CONTROL_AE_STATE_CONVERGED]));
    }

    #[test]
    fn test_metadata_tag_ids() {
        let mut meta = CameraMetadata::new();
        meta.set(MetadataEntry::new_int32(0x01, &[1]));
        meta.set(MetadataEntry::new_int32(0x02, &[2]));
        let mut ids = meta.tag_ids();
        ids.sort();
        assert_eq!(ids, vec![0x01, 0x02]);
    }
}
