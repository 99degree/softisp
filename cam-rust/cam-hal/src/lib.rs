//! Hardware Abstraction Layer for the camera pipeline.
//! Ported from com.camhal

pub mod camera;
pub use camera::*;

use cam_types::CameraSourceType;