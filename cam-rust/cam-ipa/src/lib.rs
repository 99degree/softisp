//! cam-ipa: Stable Rust IPA API for camera image processing
//!
//! This crate provides a stable, reusable Rust API for Image Processing Algorithm (IPA)
//! functionality, designed to be used by multiple frontends:
//! - libcamera-ipa: libcamera C FFI plugin
//! - jni: Android JNI bridge
//! - anw: ANativeWindow consumer

pub mod buffer;
pub mod context;
pub mod controller;
pub mod engine_manager;
pub mod error;
pub mod events;
pub mod types;

// Re-export public API
pub use buffer::{BufferManager, DmaBufBuffer, MemBuffer};
pub use context::IpaContext;
pub use controller::IpaController;
pub use engine_manager::EngineManager;
pub use error::{IpaError, IpaResult};
pub use events::EventEmitter;
pub use types::{
    Buffer, ControlInfoMap, ControlList, EngineBackend, IpaCallbacks, IpaConfig, IpaConfigureInfo,
    IpaFormat, IpaInitResult, IpaSensorInfo, IpaSettings, Rectangle, Size,
};

// Re-export commonly used types from dependencies
pub use cam_isp::isp_params::IspParams;
pub use cam_types::{FrameFormat, ToneParams};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = IpaConfig::default();
        assert_eq!(config.image_width, 1920);
        assert_eq!(config.image_height, 1080);
        assert_eq!(config.format, IpaFormat::BGRA);
    }

    #[test]
    fn test_control_list() {
        let mut controls = ControlList::new();
        assert!(controls.is_empty());

        controls.set_f32("ExposureTime", 1000.0);
        assert_eq!(controls.get_f32("ExposureTime"), Some(1000.0));

        controls.set_f32_array("ColourGains", &[1.5, 1.2]);
        let gains = controls.get_f32_array("ColourGains").unwrap();
        assert_eq!(gains, vec![1.5, 1.2]);
    }

    #[test]
    fn test_context_creation() {
        let config = IpaConfig::default();
        let ctx = IpaContext::new(config);
        assert!(ctx.is_ok());
    }
}
