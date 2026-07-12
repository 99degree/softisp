//! V4L2 stream configuration.

#[cfg(feature = "v4l2")]
use rscam::{Camera, Config, Format};

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

    let mut config = Config::new();
    config
        .resolution(width, height)
        .format(fourcc)
        .frame_rate(fps, 1);

    cam.configure(&config)
        .map_err(|e| format!("V4L2 configure failed: {:?}", e))?;

    info!(
        "V4L2 stream configured: {}x{} format={:?} fps={}",
        width, height, format, fps
    );
    Ok(())
}

/// Get current stream configuration.
#[cfg(feature = "v4l2")]
pub fn get_stream_config(cam: &Camera) -> Option<StreamConfig> {
    let format = cam.get_format().ok()?;
    let width = format.width;
    let height = format.height;
    let fourcc = format.pixelformat;
    let frame_rate = cam.get_frame_rate().ok()?;

    let frame_format = crate::v4l2::format::fourcc_to_frame_format(fourcc)?;

    Some(StreamConfig {
        width,
        height,
        format: frame_format,
        fps: frame_rate.numerator / frame_rate.denominator,
        bytes_per_line: format.bytesperline,
    })
}

#[derive(Debug, Clone)]
pub struct StreamConfig {
    pub width: u32,
    pub height: u32,
    pub format: FrameFormat,
    pub fps: u32,
    pub bytes_per_line: u32,
}

#[cfg(not(feature = "v4l2"))]
pub fn configure_stream(
    _cam: &mut rscam::Camera,
    _width: u32,
    _height: u32,
    _format: FrameFormat,
    _fps: u32,
) -> Result<(), String> {
    Ok(())
}

#[cfg(not(feature = "v4l2"))]
pub fn get_stream_config(_cam: &rscam::Camera) -> Option<StreamConfig> {
    None
}
