//! V4L2 buffer management and conversion to ByteFrame.

use cam_hal::camera::ByteFrame;
use cam_types::FrameFormat;

/// Convert raw V4L2 frame buffer to ByteFrame.
pub fn buffer_to_byte_frame(
    data: &[u8],
    width: u32,
    height: u32,
    format: FrameFormat,
    timestamp: u64,
) -> ByteFrame {
    let data = &buf.data[..];
    let stride = match format {
        FrameFormat::Rgba8888 => width * 4,
        FrameFormat::Rgb888 => width * 3,
        FrameFormat::Yuv420 => width,
        FrameFormat::Nv12 => width,
        FrameFormat::Nv21 => width,
        _ => width,
    };

    ByteFrame {
        data: data.to_vec(),
        width,
        height,
        stride,
        format: format.to_string(),
        timestamp: buf.get_timestamp(),
    }
}

#[cfg(not(feature = "v4l2"))]
pub fn buffer_to_byte_frame(
    _buf: &rscam::Buffer,
    _width: u32,
    _height: u32,
    _format: FrameFormat,
) -> ByteFrame {
    ByteFrame {
        data: Vec::new(),
        width: 0,
        height: 0,
        stride: 0,
        format: String::new(),
        timestamp: 0,
    }
}
