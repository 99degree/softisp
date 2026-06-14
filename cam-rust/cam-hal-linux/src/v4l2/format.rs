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
    match fourcc & 0xFFFFFFFF {
        0x47503252 | 0x47553241 => Some(FrameFormat::Rgba8888), // "RGBA" or "ARGB"
        0x32424752 => Some(FrameFormat::Rgb888),   // "RGB24"
        0x32315559 => Some(FrameFormat::Yuv420888), // "YUYV"
        0x3131344D => Some(FrameFormat::Yuv420888), // "M411"
        0x30313052 => Some(FrameFormat::Raw10),    // "R010" RAW10
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
