//! V4L2 buffer management (mmap).

use cam_hal::camera::ByteFrame;
use cam_types::FrameFormat;

/// Buffer information.
#[cfg(feature = "v4l2")]
#[derive(Debug, Clone)]
pub struct V4L2BufferInfo {
    pub index: u32,
    pub length: usize,
    pub offset: usize,
}

/// Convert rscam frame to our ByteFrame.
#[cfg(feature = "v4l2")]
pub fn buffer_to_byte_frame(
    frame: &rscam::Frame,
    width: u32,
    height: u32,
    format: FrameFormat,
) -> ByteFrame {
    ByteFrame {
        data: frame.to_vec(),
        width,
        height,
        format,
        timestamp: frame.get_timestamp(),
    }
}

/// Stub for when V4L2 is not available.
/// Not meant to be called - only exists so the module compiles.
#[cfg(not(feature = "v4l2"))]
pub fn buffer_to_byte_frame<T>(
    _frame: &T,
    _width: u32,
    _height: u32,
    _format: FrameFormat,
) -> ByteFrame {
    ByteFrame {
        data: Vec::new(),
        width: 0,
        height: 0,
        format: _format,
        timestamp: 0,
    }
}
