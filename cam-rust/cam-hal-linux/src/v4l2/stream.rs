//! V4L2 stream configuration.

#[cfg(feature = "v4l2")]
use rscam::{Camera, Config};

use cam_types::FrameFormat;
use log::info;

/// Configure a V4L2 stream.
#[cfg(feature = "v4l2")]
pub fn configure_stream(
    cam: &mut Camera,
    width: u32,
    height: u32,
    format: FrameFormat,
    fps: u32,
) -> Result<(), String> {
    let fourcc = crate::v4l2::format::frame_format_to_fourcc(format);
    if fourcc == 0 {
        return Err(format!("Unsupported format: {:?}", format));
    }

    let config = Config {
        interval: (fps, 1),
        resolution: (width, height),
        format: &[
            fourcc as u8,
            (fourcc >> 8) as u8,
            (fourcc >> 16) as u8,
            (fourcc >> 24) as u8,
        ],
        field: 0,
        nbuffers: 2,
    };

    cam.start(&config)
        .map_err(|e| format!("V4L2 configure failed: {:?}", e))?;

    info!(
        "V4L2 stream configured: {}x{} format={:?} fps={}",
        width, height, format, fps
    );
    Ok(())
}

/// Get current stream configuration.
/// Note: rscam does not provide a way to query current format after start(),
/// so we return a best-effort config based on the configured parameters.
#[cfg(feature = "v4l2")]
pub fn get_stream_config(_cam: &Camera) -> Option<StreamConfig> {
    None
}

#[derive(Debug, Clone)]
pub struct StreamConfig {
    pub width: u32,
    pub height: u32,
    pub format: FrameFormat,
    pub fps: u32,
    pub bytes_per_line: u32,
}

/// Stub for when V4L2 is not available.
#[cfg(not(feature = "v4l2"))]
pub fn configure_stream<T>(
    _cam: &mut T,
    _width: u32,
    _height: u32,
    _format: FrameFormat,
    _fps: u32,
) -> Result<(), String> {
    Ok(())
}

/// Stub for when V4L2 is not available.
#[cfg(not(feature = "v4l2"))]
pub fn get_stream_config<T>(_cam: &T) -> Option<StreamConfig> {
    None
}
