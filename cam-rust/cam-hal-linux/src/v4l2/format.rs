//! V4L2 pixel format handling and conversions.

#[derive(Debug, Clone)]
pub struct FormatInfo {
    pub fourcc: u32,
    pub description: String,
    pub width: u32,
    pub height: u32,
    pub bytes_per_line: u32,
}

/// Convert V4L2 fourcc to FrameFormat
pub fn fourcc_to_frame_format(fourcc: u32) -> Option<FrameFormat> {
    match fourcc & 0xFF {
        // BAWER formats
        b'B' if fourcc == 0x47553241 => Some(FrameFormat::Rgba8888), // "ARGB"
        b'R' if fourcc == 0x32424752 => Some(FrameFormat::Rgba8888), // "RGB24"
        // YUV formats
        b'Y' if fourcc == 0x32595559 => Some(FrameFormat::Yuv420), // "YUYV"
        b'N' if fourcc == 0x32554E4E => Some(FrameFormat::Nv12),   // "NV12"
        b'U' if fourcc == 0x32315555 => Some(FrameFormat::Nv21),   // "UYVY"
        _ => None,
    }
}

/// Convert FrameFormat to V4L2 fourcc
pub fn frame_format_to_fourcc(format: FrameFormat) -> u32 {
    match format {
        FrameFormat::Rgba8888 => 0x47503252, // "RGBA"
        FrameFormat::Rgb888 => 0x32424752,   // "RGB24"
        FrameFormat::Yuv420888 => 0x32315559, // "YUYV"
        FrameFormat::RawSensor => 0x30313052, // "R010" RAW10
        FrameFormat::Raw10 => 0x30313052,    // "R010"
        FrameFormat::Raw12 => 0x32313052,    // "R012"
        _ => 0x47503252, // Default to RGBA
    }
}

/// Get available formats from a V4L2 device
#[cfg(feature = "v4l2")]
pub fn get_supported_formats(cam: &rscam::Camera) -> Vec<FormatInfo> {
    let fmts = cam.query_formats().unwrap_or_default();
    fmts.into_iter()
        .map(|fmt| FormatInfo {
            fourcc: fmt.pixelformat,
            description: String::new(),
            width: fmt.width,
            height: fmt.height,
            bytes_per_line: fmt.bytesperline,
        })
        .collect()
}
