//! Adapter between the camera capture path and the `cam-isp` pipeline.
//!
//! Frames arrive as little-endian `u16` Bayer samples; the engine's
//! `ProcessParams` wants a byte buffer plus the sensor's full-well value. The
//! frame exposes its samples as a byte slice directly, so this module only
//! supplies the shared graph/display configuration.

use cam_isp::engine::{OutputFormat, ProcessParams};

/// The output format both the graph and the engine must agree on.
///
/// `PackedRgb` is the production path: the fused `isp.display_uint32_bgra`
/// shader writes one INT32 word per two pixels, whose bytes are already
/// little-endian BGRA. The host presents them directly, with no plane
/// interleave and no per-pixel conversion. Float RGB is not a camera-preview
/// format — it leaves the display in the graph's float tail and needs a host
/// conversion path that hides readback errors.
///
/// This is shared so the graph's `DisplayBlock` and `ProcessParams` cannot
/// disagree: a mismatch is silent, because the frame keeps the right length and
/// `present()` still returns `Ok`.
pub const OUTPUT_FORMAT: OutputFormat = OutputFormat::PackedRgb;

/// Description of the sensor the pipeline is being driven from.
#[derive(Debug, Clone, Copy)]
pub struct PipelineConfig {
    /// Full-well capacity of the sensor.
    pub sensor_max: f32,
    /// Black level per Bayer channel `[R, Gr, Gb, B]` in raw sensor counts.
    pub black_level: [f32; 4],
    /// CFA phase, as the ISP's `bayer_pattern` encoding.
    pub bayer_pattern: i32,
}

impl PipelineConfig {
    /// Whether the engine emits planar f32 RGB rather than bytes.
    ///
    /// The production path emits packed RGB (BGRA bytes), so this is always
    /// false; the flag keeps the frame loop from duplicating the format choice.
    pub fn float_output(&self) -> bool {
        false
    }

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
        // Black level from the sensor's `ACAMERA_SENSOR_BLACK_LEVEL_PATTERN`
        // drives BlcBlock / Blc50Block subtraction at the head of the chain.
        params.blc_values = Some(self.black_level);
        // `PackedRgb`: the fused display shader writes packed BGRA bytes,
        // which the window can present without any host-side conversion.
        params.output_format = OUTPUT_FORMAT;
        params.timestamp_ns = ts_ns;
        params
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::RawFrame;

    #[test]
    fn frame_data_bytes_are_little_endian() {
        let frame = RawFrame {
            data: vec![0x0102, 0xFF00],
            width: 2,
            height: 1,
            timestamp_ns: 0,
            ahb: None,
        };
        assert_eq!(frame.data_bytes(), &[0x02, 0x01, 0x00, 0xFF]);
    }

    #[test]
    fn frame_data_bytes_of_empty_frame_are_empty() {
        let frame = RawFrame {
            data: Vec::new(),
            width: 0,
            height: 0,
            timestamp_ns: 0,
            ahb: None,
        };
        assert!(frame.data_bytes().is_empty());
    }

    #[test]
    fn params_carry_sensor_and_bayer() {
        let cfg = PipelineConfig {
            sensor_max: 4095.0,
            black_level: [0.0; 4],
            bayer_pattern: 2,
        };
        let staged = [0u8; 8];
        let p = cfg.params(2, 1, &staged, 1234);
        assert_eq!(p.sensor_max, 4095.0);
        assert_eq!(p.bayer_pattern, 2);
        assert_eq!(p.timestamp_ns, 1234);
        assert_eq!(p.output_format, OUTPUT_FORMAT);
        assert_eq!(p.width, 2);
        assert_eq!(p.height, 1);
        assert_eq!(p.blc_values, Some([0.0; 4]));
    }
}
