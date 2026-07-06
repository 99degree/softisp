//! Android Camera HAL Binder Interfaces.
//!
//! Implements the AIDL interfaces matching:
//! - `android.hardware.camera.provider.ICameraProvider`
//! - `android.hardware.camera.provider.ICameraProviderCallback`
//! - `android.hardware.camera.device.ICameraDevice`
//! - `android.hardware.camera.device.ICameraDeviceCallback`
//! - `android.hardware.camera.device.ICameraDeviceSession`
//!
//! On Android, these are backed by the real `binder` crate (AOSP).
//! On host (Linux), we provide a local simulated binder using channels,
//! so the demo app can run without Android framework.

pub mod provider;
pub mod device;
pub mod session;
pub mod types;
pub mod callback;
pub mod service;
pub mod isp_session;
pub mod metadata;
pub mod dispatch;

pub use provider::CameraProvider;
pub use device::CameraDevice;
pub use session::CameraDeviceSession;
pub use service::CameraHalService;
pub use isp_session::IspCameraSession;
pub use metadata::{CameraMetadata, MetadataEntry, MetadataType};
pub use dispatch::{dispatch_provider_transaction, dispatch_device_transaction, dispatch_session_transaction};
