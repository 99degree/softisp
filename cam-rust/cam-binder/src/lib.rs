//! Android Camera HAL binder service.
//!
//! Implements the `android.hardware.camera.provider.ICameraProvider` AIDL
//! interface using the `binder` crate. Registers with the Android service
//! manager as "media.camera" (the standard camera HAL service name).
//!
//! ## Architecture
//!
//! ```text
//! BnCameraProvider (binder service)
//!   ├─ setCallback(ICameraProviderCallback)
//!   ├─ getCameraIdList() → [String]
//!   ├─ getCameraDeviceInterface(id) → ICameraDevice
//!   ├─ notifyDeviceStateChange(state)
//!   └─ getVendorTags() → [VendorTag]
//!
//! BnCameraDevice (binder service, one per camera)
//!   ├─ open() → ICameraDeviceSession
//!   ├─ getCharacteristics() → CameraCharacteristics
//!   └─ close()
//!
//! BnCameraDeviceSession (binder service, per session)
//!   ├─ processCaptureRequest(request) → CaptureResult
//!   ├─ flush()
//!   └─ close()
//! ```
//!
//! Each session owns an IspEngine instance and an IspBlock pipeline.
//! Raw frames from the camera are fed through the pipeline to produce
//! the output frames sent back as CaptureResults.

#![allow(dead_code)]

use log::{info, warn, error};
use std::sync::{Arc, Mutex};

pub mod provider;
pub mod device;
pub mod session;

// Re-export the key types
pub use provider::CameraProviderService;
pub use device::CameraDeviceService;
pub use session::CameraDeviceSession;

/// The standard service name for the camera HAL.
pub const CAMERA_HAL_SERVICE_NAME: &str = "media.camera";
