//! V4L2 pixel format handling and conversions.

#[cfg(feature = "v4l2")]
use rscam::Camera;

use cam_types::FrameFormat;

/// V4L2 format description.
#[derive(Debug, Clone)]
pub struct V4L2FormatInfo {
    pub fourcc: u32,
    pub description: String,
    pub width: u32,
    pub height: u32,
    pub bytes_per_line: u32,
}

/// Convert V4L2 fourcc to FrameFormat
#[cfg(feature = "v4l2")]
pub fn fourcc_to_frame_format(fourcc: u32) -> Option<FrameFormat> {
    let fmt_bytes = fourcc.to_le_bytes();
    match &fmt_bytes {
        b"ARGB" | b"RGBA" | b"RGB4" => Some(FrameFormat::Rgba8888),
        b"RGB3" | b"RGB\0" => Some(FrameFormat::Rgb888),
        b"YUYV" | b"NV12" | b"NV21" => Some(FrameFormat::Yuv420888),
        _ => None,
    }
}

/// Convert FrameFormat to V4L2 fourcc
#[cfg(feature = "v4l2")]
pub fn frame_format_to_fourcc(format: FrameFormat) -> u32 {
    match format {
        FrameFormat::Rgba8888 => u32::from_le_bytes(*b"RGBA"),
        FrameFormat::Rgb888 => u32::from_le_bytes(*b"RGB3"),
        FrameFormat::Yuv420888 => u32::from_le_bytes(*b"YUYV"),
        FrameFormat::RawSensor => u32::from_le_bytes(*b"R010"),
        FrameFormat::Raw10 => u32::from_le_bytes(*b"R010"),
        FrameFormat::Raw12 => u32::from_le_bytes(*b"R012"),
        _ => 0,
    }
}

/// Get available formats from a V4L2 device
/// Note: rscam formats() returns FormatInfo with [u8; 4] fourcc and description only.
/// Width/height are obtained via resolutions().
#[cfg(feature = "v4l2")]
pub fn get_supported_formats(cam: &Camera) -> Vec<V4L2FormatInfo> {
    let mut result = Vec::new();
    for fmt in cam.formats() {
        let fmt = match fmt {
            Ok(f) => f,
            Err(_) => continue,
        };
        let fourcc = u32::from_le_bytes(fmt.format);
        // Try to get first resolution for this format
        if let Ok(res) = cam.resolutions(&fmt.format) {
            let (w, h) = match res {
                rscam::ResolutionInfo::Discretes(list) => list.first().copied().unwrap_or((0, 0)),
                rscam::ResolutionInfo::Stepwise { min, .. } => min,
            };
            result.push(V4L2FormatInfo {
                fourcc,
                description: fmt.description,
                width: w,
                height: h,
                bytes_per_line: 0,
            });
        }
    }
    result
}

/// Stub for when V4L2 is not available.
#[cfg(not(feature = "v4l2"))]
pub fn fourcc_to_frame_format(_fourcc: u32) -> Option<FrameFormat> {
    None
}

/// Stub for when V4L2 is not available.
#[cfg(not(feature = "v4l2"))]
pub fn frame_format_to_fourcc(_format: FrameFormat) -> u32 {
    0
}

/// Stub for when V4L2 is not available.
#[cfg(not(feature = "v4l2"))]
pub fn get_supported_formats<T>(_cam: &T) -> Vec<V4L2FormatInfo> {
    vec![]
}
