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
    ByteFrame {
        data: data.to_vec(),
        width,
        height,
        format,
        timestamp,
    }
}
