#![allow(unused, non_upper_case_globals)]
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
pub mod factory;
pub mod binder;
pub mod aidl;

// Re-export buffer management from cam-hal (unified across all crates)
pub use cam_hal::buffer::{
    MappedBuffer, CameraBuffer, BufferAllocator,
    HeapAllocator, HeapBuffer, HeapCameraBuffer, GenericCameraBuffer,
    allocator, set_allocator, allocate, allocate_frame,
};

// Re-export platform-specific buffer backends
#[cfg(feature = "v4l2")]
pub use cam_hal_linux::dmabuf::{DMABuf, DMABufAllocator, MemfdBuf, MemfdAllocator};
#[cfg(feature = "android")]
pub use cam_hal_android::gralloc::{native_handle_t, BufferHandleT, GrallocInterop};

pub use provider::CameraProvider;
pub use device::CameraDevice;
pub use session::CameraDeviceSession;
pub use service::CameraHalService;
pub use isp_session::IspCameraSession;
pub use metadata::{CameraMetadata, MetadataEntry, MetadataType};
pub use dispatch::{dispatch_provider_transaction, dispatch_device_transaction, dispatch_session_transaction};
pub use factory::{CameraProviderFactory, HidlToAidlShim, VndkCompatLayer, VintfManifestEntry};
pub use binder::{Parcel, IBinder, BinderStatus, BpCameraProvider, BpCameraDevice, BpCameraDeviceSession, ServiceManager, BinderThreadPool, LocalBinder};
pub use aidl::{BnCameraProvider, BnCameraDevice, BnCameraDeviceSession};
