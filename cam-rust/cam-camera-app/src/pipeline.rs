//! Adapter between the camera capture path and the `cam-isp` pipeline.
//!
//! Frames arrive as little-endian `u16` Bayer samples; the engine's
//! `ProcessParams` wants a byte buffer plus the sensor's full-well value. This
//! module owns that conversion and keeps one reusable byte buffer alive so a
//! preview loop does not allocate per frame.

use cam_isp::engine::{OutputFormat, ProcessParams};

use crate::camera::RawFrame;

/// Reusable staging buffers for the `u16` → `Vec<u8>` widening the API needs.
pub struct FrameStager {
    bytes: Vec<u8>,
}

impl FrameStager {
    /// Create a stager with an empty staging buffer.
    pub fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    /// Widen `frame` into the staging buffer as little-endian `u16` bytes.
    ///
    /// The returned slice borrows the stager, so it stays valid until the next
    /// call — which is exactly the lifetime `ProcessParams` needs.
    pub fn stage(&mut self, frame: &RawFrame) -> &[u8] {
        self.bytes.clear();
        self.bytes.reserve(frame.data.len() * 2);
        for &sample in &frame.data {
            self.bytes.push((sample & 0xFF) as u8);
            self.bytes.push((sample >> 8) as u8);
        }
        &self.bytes
    }
}

impl Default for FrameStager {
    fn default() -> Self {
        Self::new()
    }
}

/// Description of the sensor the pipeline is being driven from.
#[derive(Debug, Clone, Copy)]
pub struct PipelineConfig {
    /// Full-well capacity of the sensor.
    pub sensor_max: f32,
    /// CFA phase, as the ISP's `bayer_pattern` encoding.
    pub bayer_pattern: i32,
}

impl PipelineConfig {
    /// Build `ProcessParams` for one frame.
    ///
    /// # Panics
    /// Never — but the returned params borrow `staged`, so the caller must
    /// keep the staging buffer alive across the `process` call.
    pub fn params<'a>(
        &self,
        width: u32,
        height: u32,
        staged: &'a [u8],
        ts_ns: u64,
    ) -> ProcessParams<'a> {
        let mut params = ProcessParams::new(width, height, staged);
        params.sensor_max = self.sensor_max;
        params.bayer_pattern = self.bayer_pattern;
        // The CPU engine writes BGRA through the display block, so the
        // intermediate packed format is never observed by this app.
        params.output_format = OutputFormat::Bgra;
        params.timestamp_ns = ts_ns;
        params
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::RawFrame;

    #[test]
    fn stage_emits_little_endian_u16() {
        let frame = RawFrame {
            data: vec![0x0102, 0xFF00],
            width: 2,
            height: 1,
            timestamp_ns: 0,
        };
        let mut stager = FrameStager::new();
        assert_eq!(stager.stage(&frame), &[0x02, 0x01, 0x00, 0xFF]);
    }

    #[test]
    fn stage_reuses_buffer_across_frames() {
        let mut stager = FrameStager::new();
        let big = RawFrame {
            data: vec![7; 64],
            width: 8,
            height: 8,
            timestamp_ns: 0,
        };
        stager.stage(&big);
        let capacity = stager.bytes.capacity();
        let small = RawFrame {
            data: vec![1, 2],
            width: 2,
            height: 1,
            timestamp_ns: 0,
        };
        assert_eq!(stager.stage(&small), &[1, 0, 2, 0]);
        assert_eq!(
            stager.bytes.capacity(),
            capacity,
            "buffer should not reallocate"
        );
    }

    #[test]
    fn stage_of_empty_frame_is_empty() {
        let frame = RawFrame {
            data: Vec::new(),
            width: 0,
            height: 0,
            timestamp_ns: 0,
        };
        assert!(FrameStager::new().stage(&frame).is_empty());
    }

    #[test]
    fn params_carry_sensor_and_bayer() {
        let cfg = PipelineConfig {
            sensor_max: 4095.0,
            bayer_pattern: 2,
        };
        let staged = [0u8; 8];
        let p = cfg.params(2, 1, &staged, 1234);
        assert_eq!(p.sensor_max, 4095.0);
        assert_eq!(p.bayer_pattern, 2);
        assert_eq!(p.timestamp_ns, 1234);
        assert_eq!(p.output_format, OutputFormat::Bgra);
        assert_eq!(p.width, 2);
        assert_eq!(p.height, 1);
    }
}
