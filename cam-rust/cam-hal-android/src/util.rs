//! Camera2-derived format utilities for Android camera buffers.
//!
//! Provides frame format conversion between Android HAL pixel formats
//! and the ISP pipeline's expected input. These functions were originally
//! part of the Camera2 adapter and are still useful for the HAL3 adapter.

#![allow(dead_code)]

// ── Android HAL pixel format constants ─────────────────────────────────────

/// 16-bit Bayer RAW (BGGR/GRBG/RGGB/GBRG).
pub const HAL_PIXEL_FORMAT_RAW16: i32 = 0x20;
/// Compressed JPEG blob.
pub const HAL_PIXEL_FORMAT_BLOB: i32 = 0x21;
/// Implementation-defined (gralloc picks).
pub const HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED: i32 = 0x22;
/// Opaque RAW (sensor-specific).
pub const HAL_PIXEL_FORMAT_RAW_OPAQUE: i32 = 0x24;
/// 10-bit packed Bayer.
pub const HAL_PIXEL_FORMAT_RAW10: i32 = 0x25;
/// 12-bit packed Bayer.
pub const HAL_PIXEL_FORMAT_RAW12: i32 = 0x26;
/// YUV 420 NV21 (Y plane + interleaved VU).
pub const HAL_PIXEL_FORMAT_YCRCB_420_SP: i32 = 0x11;
/// YUV 420 NV12 (Y plane + interleaved UV).
pub const HAL_PIXEL_FORMAT_YCBCR_420_SP: i32 = 0x12;
/// YUV 420 planar (YU12/I420).
pub const HAL_PIXEL_FORMAT_YV12: i32 = 0x32315659;
/// RGBA 8888
pub const HAL_PIXEL_FORMAT_RGBA_8888: i32 = 0x1;

// ── Bayer pattern detection ────────────────────────────────────────────────

/// Bayer phase order (CFA pattern).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BayerPattern {
    /// R G / G B (RGGB) — most common
    Rggb,
    /// G R / B G (GRBG)
    Grbg,
    /// G B / R G (GBRG)
    Gbrg,
    /// B G / G R (BGGR)
    Bggr,
    /// Unknown — treat as BGGR (most common default)
    Unknown,
}

impl BayerPattern {
    /// Index of the red pixel in a 2×2 Bayer block.
    pub fn red_index(self) -> usize {
        match self {
            BayerPattern::Rggb => 3,    // (1,1)
            BayerPattern::Grbg => 2,    // (1,0)
            BayerPattern::Gbrg => 1,    // (0,1)
            BayerPattern::Bggr => 0,    // (0,0)
            BayerPattern::Unknown => 3, // default RGGB-like
        }
    }

    /// Index of the blue pixel in a 2×2 Bayer block.
    pub fn blue_index(self) -> usize {
        match self {
            BayerPattern::Rggb => 0, // (1,1) → opposite of red
            BayerPattern::Grbg => 1,
            BayerPattern::Gbrg => 2,
            BayerPattern::Bggr => 3,
            BayerPattern::Unknown => 0,
        }
    }

    /// Parse from Android camera metadata string (e.g. "BGGR", "RGGB").
    pub fn from_metadata_string(s: &str) -> Self {
        match s.to_uppercase().as_str() {
            "RGGB" => BayerPattern::Rggb,
            "GRBG" => BayerPattern::Grbg,
            "GBRG" => BayerPattern::Gbrg,
            "BGGR" => BayerPattern::Bggr,
            _ => BayerPattern::Unknown,
        }
    }
}

// ── Format conversions ─────────────────────────────────────────────────────

/// Bytes per pixel for a given HAL format.
pub fn hal_format_bpp(format: i32) -> usize {
    match format {
        HAL_PIXEL_FORMAT_RAW16 | HAL_PIXEL_FORMAT_RAW10 | HAL_PIXEL_FORMAT_RAW12 => 2,
        HAL_PIXEL_FORMAT_RAW_OPAQUE => 2,
        HAL_PIXEL_FORMAT_RGBA_8888 => 4,
        HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED => 4,
        HAL_PIXEL_FORMAT_YCRCB_420_SP | HAL_PIXEL_FORMAT_YCBCR_420_SP => 1,
        HAL_PIXEL_FORMAT_YV12 => 1,
        HAL_PIXEL_FORMAT_BLOB => 1,
        _ => 2, // fallback
    }
}

/// Estimate buffer size for a given format and resolution.
/// Some formats (NV21, YV12) have non-trivial size calculations.
pub fn hal_format_buffer_size(format: i32, width: u32, height: u32, stride: u32) -> usize {
    let w = stride.max(width) as usize;
    let h = height as usize;
    match format {
        HAL_PIXEL_FORMAT_YCRCB_420_SP | HAL_PIXEL_FORMAT_YCBCR_420_SP => {
            // Y plane: w*h, UV plane: w*h/2
            w * h + w * h / 2
        }
        HAL_PIXEL_FORMAT_YV12 => {
            // Y plane: w*h, U plane: (w/2)*(h/2), V plane: (w/2)*(h/2)
            w * h + (w / 2) * (h / 2) * 2
        }
        _ => {
            let bpp = hal_format_bpp(format);
            w * h * bpp
        }
    }
}

/// Unpack 10-bit packed Bayer data to 16-bit (little-endian).
///
/// RAW10 packs 4 pixels into 5 bytes:
///   Byte 0:  Pixel0\[7:0\] (bits 9:2)
///   Byte 1:  Pixel1\[7:0\]
///   Byte 2:  Pixel2\[7:0\]
///   Byte 3:  Pixel3\[7:0\]
///   Byte 4:  Pixel3\[1:0\] | Pixel2\[1:0\] << 2 | Pixel1\[1:0\] << 4 | Pixel0\[1:0\] << 6
///
/// Output: width×height×2 bytes of 16-bit LE data.
pub fn unpack_raw10_to_16le(
    input: &[u8],
    width: usize,
    height: usize,
    stride_packed: usize,
) -> Vec<u8> {
    let expected_packed = stride_packed * height * 5 / 4; // 4 pixels per 5 bytes
    let actual = input.len().min(expected_packed);
    let mut out = vec![0u8; width * height * 2];

    for y in 0..height {
        let row_offset = y * stride_packed * 5 / 4;
        let out_row = y * width * 2;

        for x in (0..width).step_by(4) {
            let block = row_offset + (x / 4) * 5;
            if block + 5 > actual {
                break;
            }

            let b0 = input[block] as u16;
            let b1 = input[block + 1] as u16;
            let b2 = input[block + 2] as u16;
            let b3 = input[block + 3] as u16;
            let b4 = input[block + 4] as u16;

            // Extract 10-bit values
            let pix0 = (b0 << 2) | ((b4 >> 6) & 0x03);
            let pix1 = (b1 << 2) | ((b4 >> 4) & 0x03);
            let pix2 = (b2 << 2) | ((b4 >> 2) & 0x03);
            let pix3 = (b3 << 2) | (b4 & 0x03);

            for (j, pix) in [pix0, pix1, pix2, pix3].iter().enumerate() {
                let idx = out_row + (x + j) * 2;
                if idx + 2 <= out.len() {
                    let v = *pix << 6; // left-justify to 16 bits
                    out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
                }
            }
        }
    }
    out
}

/// Unpack 12-bit packed Bayer data to 16-bit (little-endian).
///
/// RAW12 packs 2 pixels into 3 bytes:
///   Byte 0:  Pixel0\[11:4\]
///   Byte 1:  Pixel0\[3:0\] | Pixel1\[11:8\] << 4
///   Byte 2:  Pixel1\[7:0\]
///
/// Output: width×height×2 bytes of 16-bit LE data.
pub fn unpack_raw12_to_16le(
    input: &[u8],
    width: usize,
    height: usize,
    stride_packed: usize,
) -> Vec<u8> {
    let expected_packed = stride_packed * height * 3 / 2; // 2 pixels per 3 bytes
    let actual = input.len().min(expected_packed);
    let mut out = vec![0u8; width * height * 2];

    for y in 0..height {
        let row_offset = y * stride_packed * 3 / 2;
        let out_row = y * width * 2;

        for x in (0..width).step_by(2) {
            let block = row_offset + (x / 2) * 3;
            if block + 3 > actual {
                break;
            }

            let b0 = input[block] as u16;
            let b1 = input[block + 1] as u16;
            let b2 = input[block + 2] as u16;

            let pix0 = (b0 << 4) | (b1 >> 4);
            let pix1 = ((b1 & 0x0F) << 8) | b2;

            let v0 = pix0 << 4; // left-justify to 16 bits
            let v1 = pix1 << 4;

            let idx = out_row + x * 2;
            if idx + 4 <= out.len() {
                out[idx..idx + 2].copy_from_slice(&v0.to_le_bytes());
                out[idx + 2..idx + 4].copy_from_slice(&v1.to_le_bytes());
            }
        }
    }
    out
}

/// Extract the Y (luma) plane from an NV21 buffer.
/// NV21: Y plane = first w*h bytes, then VU interleaved.
pub fn nv21_extract_y(data: &[u8], width: usize, height: usize) -> &[u8] {
    let y_size = width * height;
    &data[..y_size.min(data.len())]
}

/// Collapse NV21 Y+UV into a single RGBA frame (gray from Y, ignore UV).
pub fn nv21_to_gray_rgba(data: &[u8], width: usize, height: usize) -> Vec<u8> {
    let y_size = width * height;
    let y_plane = &data[..y_size.min(data.len())];
    let mut rgba = vec![0u8; width * height * 4];

    for (i, &y) in y_plane.iter().enumerate() {
        let idx = i * 4;
        rgba[idx] = y;
        rgba[idx + 1] = y;
        rgba[idx + 2] = y;
        rgba[idx + 3] = 255;
    }
    rgba
}

/// Convert NV21 to RGBA (full color using UV interpolation).
/// This is a simplified converter — for production use, a SIMD-optimized
/// version is recommended.
pub fn nv21_to_rgba(data: &[u8], width: usize, height: usize) -> Vec<u8> {
    let y_size = width * height;
    let uv_size = width * height / 2;
    if data.len() < y_size + uv_size {
        return nv21_to_gray_rgba(data, width, height);
    }

    let y_plane = &data[..y_size];
    let uv_plane = &data[y_size..];
    let mut rgba = vec![0u8; width * height * 4];

    for y in 0..height {
        for x in 0..width {
            let y_idx = y * width + x;
            let uv_x = x / 2;
            let uv_y = y / 2;
            let uv_idx = uv_y * width + uv_x * 2;

            let y_val = y_plane[y_idx] as i32;
            let v = uv_plane[uv_idx] as i32 - 128; // V
            let u = uv_plane[uv_idx + 1] as i32 - 128; // U

            // BT.601 conversion (limited range)
            let r = (y_val + 359 * v / 256).clamp(0, 255) as u8;
            let g = (y_val - 88 * u / 256 - 183 * v / 256).clamp(0, 255) as u8;
            let b = (y_val + 454 * u / 256).clamp(0, 255) as u8;

            let idx = y_idx * 4;
            rgba[idx] = r;
            rgba[idx + 1] = g;
            rgba[idx + 2] = b;
            rgba[idx + 3] = 255;
        }
    }
    rgba
}

/// Detect Bayer pattern from a raw frame header or sensor metadata string.
/// Defaults to BGGR if detection fails.
pub fn detect_bayer_pattern(metadata_string: Option<&str>) -> BayerPattern {
    match metadata_string {
        Some(s) => BayerPattern::from_metadata_string(s),
        None => BayerPattern::Bggr,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bayer_pattern_detection() {
        assert_eq!(
            BayerPattern::from_metadata_string("BGGR"),
            BayerPattern::Bggr
        );
        assert_eq!(
            BayerPattern::from_metadata_string("RGGB"),
            BayerPattern::Rggb
        );
        assert_eq!(
            BayerPattern::from_metadata_string("GRBG"),
            BayerPattern::Grbg
        );
        assert_eq!(
            BayerPattern::from_metadata_string("GBRG"),
            BayerPattern::Gbrg
        );
        assert_eq!(
            BayerPattern::from_metadata_string("unknown"),
            BayerPattern::Unknown
        );
    }

    #[test]
    fn test_format_bpp() {
        assert_eq!(hal_format_bpp(HAL_PIXEL_FORMAT_RAW16), 2);
        assert_eq!(hal_format_bpp(HAL_PIXEL_FORMAT_RGBA_8888), 4);
        assert_eq!(hal_format_bpp(HAL_PIXEL_FORMAT_YCRCB_420_SP), 1);
    }

    #[test]
    fn test_unpack_raw10() {
        // 2x2 Bayer block = 4 pixels, packed into 5 bytes
        let packed = vec![
            0x00, 0x40, 0x80, 0xC0, // pixel[0..3] top 8 bits
            0x00, // pixel[0..3] bottom 2 bits packed
        ];
        let unpacked = unpack_raw10_to_16le(&packed, 4, 1, 4);
        // 4 pixels × 2 bytes = 8 bytes
        assert_eq!(unpacked.len(), 8);
    }

    #[test]
    fn test_unpack_raw12() {
        // 2 pixels packed into 3 bytes
        let packed = vec![0x00, 0x00, 0x00]; // pixel0=0x000, pixel1=0x000
        let unpacked = unpack_raw12_to_16le(&packed, 2, 1, 2);
        assert_eq!(unpacked.len(), 4);
    }
}
