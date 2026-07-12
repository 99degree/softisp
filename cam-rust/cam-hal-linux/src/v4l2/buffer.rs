//! V4L2 buffer management (mmap).

use cam_hal::camera::ByteFrame;
use cam_types::FrameFormat;

#[cfg(feature = "v4l2")]
use rscam::Buffer;

/// Buffer information.
#[cfg(feature = "v4l2")]
#[derive(Debug, Clone)]
pub struct V4L2BufferInfo {
    pub index: u32,
    pub length: usize,
    pub offset: usize,
}

/// Convert rscam buffer to our ByteFrame.
#[cfg(feature = "v4l2")]
pub fn buffer_to_byte_frame(
    buf: &Buffer,
    width: u32,
    height: u32,
    format: FrameFormat,
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
