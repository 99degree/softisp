#![allow(unused, non_upper_case_globals)]
//! # cam-binder — Android Camera HAL Binder Interface
//!
//! A complete Rust implementation of the AOSP Camera HAL3 AIDL interface,
//! with integrated ISP pipeline processing and V4L2 camera support.
//!
//! ## Overview
//!
//! This crate provides a binary-compatible Android Camera HAL implementation
//! that bridges the AOSP binder camera interface with the ISP pipeline:
//!
//! ```text
//! Android Framework
//!     ↓ (AIDL binder)
//! ICameraProvider → ICameraDevice → ICameraDeviceSession
//!     ↓                                    ↓
//! CameraProvider                    ISP Pipeline
//!     ↓                              (ONNX → MNN/Vulkan)
//! V4L2 Camera ─────────────────────────→ Output (RGBA)
//! ```
//!
//! ## Modules
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`provider`] | `ICameraProvider` — camera enumeration and device access |
//! | [`device`] | `ICameraDevice` — camera characteristics and session creation |
//! | [`session`] | `ICameraDeviceSession` — stream config and capture requests |
//! | [`types`] | AIDL types: `StreamConfig`, `CaptureRequest`, `BufferUsage`, etc. |
//! | [`metadata`] | `CameraMetadata` — AOSP-compatible metadata serialization |
//! | [`callback`] | Callback traits for frame capture and device events |
//! | [`binder`] | Core binder types: `Parcel`, `IBinder`, `ServiceManager` |
//! | [`aidl`] | AIDL stubs: `BnCameraProvider`, `BnCameraDevice`, `BnCameraDeviceSession` |
//! | [`factory`] | Service factory: `CameraProviderFactory`, `VintfManifestEntry` |
//! | [`service`] | High-level `CameraHalService` for easy integration |
//!
//! ## Quick Start
//!
//! ### Test Pattern (no camera needed)
//!
//! ```bash
//! cargo run --bin cam-hal-service -- --width 640 --height 480 --frames 3
//! ```
//!
//! ### V4L2 Real Camera
//!
//! ```bash
//! cargo run --features v4l2 --bin cam-hal-service -- --auto-detect --isp
//! ```
//!
//! ### Binder Service Mode (Android)
//!
//! ```bash
//! cargo run --bin cam-hal-service -- --service
//! ```
//!
//! ## AIDL Interface Coverage
//!
//! | Interface | Methods | Transaction Codes |
//! |-----------|---------|-------------------|
//! | ICameraProvider | 7 | 1-7 |
//! | ICameraDevice | 7 | 1-7 |
//! | ICameraDeviceSession | 12 | 1-12 |
//! | **Total** | **26** | |
//!
//! ## Feature Flags
//!
//! | Feature | Description |
//! |---------|-------------|
//! | `v4l2` | Enable V4L2 camera capture (requires rscam) |
//! | `android` | Enable Android-specific binder registration |
//! | `mnn` | Enable MNN inference engine |
//!
//! ## Architecture
//!
//! ### Binder Layer
//!
//! The binder layer provides binary-compatible AIDL interface implementation:
//!
//! - **`Parcel`** — AIDL-compatible serialization buffer
//! - **`IBinder`** — Interface for binder objects
//! - **`BpXxx`** — Client-side proxy stubs
//! - **`BnXxx`** — Server-side stub implementations
//!
//! ### Camera HAL Layer
//!
//! The camera HAL layer implements the AOSP camera interface:
//!
//! - **`CameraProvider`** — Enumerates cameras, creates devices
//! - **`CameraDevice`** — Camera characteristics, opens sessions
//! - **`CameraDeviceSession`** — Configures streams, processes captures
//!
//! ### ISP Pipeline Layer
//!
//! The ISP layer processes camera frames:
//!
//! - **`PipelineBuilder`** — Constructs ISP pipeline (Unpack → Demosaic → Display)
//! - **`IspEngine`** — Executes pipeline (CPU/Vulkan)
//! - **`ProcessParams`** — Input frame parameters
//!
//! ### V4L2 Layer
//!
//! The V4L2 layer captures real camera frames:
//!
//! - **`list_v4l2_cameras()`** — Scans `/dev/video*`
//! - **`capture_single_v4l2_frame()`** — Single frame capture
//! - **`V4L2CameraAdapter`** — Streaming capture with callbacks
//!
//! ## Integration Example
//!
//! ```rust,ignore
//! use cam_binder::{CameraHalService, types::*};
//!
//! // Create HAL service
//! let service = CameraHalService::new();
//!
//! // Enumerate cameras
//! let cameras = service.get_camera_id_list();
//!
//! // Open camera with callback
//! let session = service.open_camera("0", my_callback)?;
//!
//! // Configure streams
//! let config = StreamConfig::new(0, 1920, 1080, 0x1);
//! session.lock().unwrap().configure_streams(&[config]);
//!
//! // Capture frames
//! for i in 0..10 {
//!     let request = CaptureRequest::preview(i, 0);
//!     let buffers = session.lock().unwrap().process_capture_request(&request);
//!     // buffers contain RGBA frame data
//! }
//!
//! // Cleanup
//! session.lock().unwrap().close();
//! ```
//!
//! ## Binary Compatibility
//!
//! This implementation is designed for binary compatibility with AOSP:
//!
//! - Parcel layout matches `android::Parcel`
//! - Transaction codes match AIDL interface spec
//! - String16 encoding for AIDL string format
//! - Interface token verification
//! - CameraMetadata serialize/deserialize

pub mod provider;
pub mod device;
pub mod session;
pub mod types;
pub mod error;
pub mod callback;
pub mod service;
pub mod isp_session;
pub mod metadata;
pub mod dispatch;
pub mod factory;
pub mod binder;
pub mod aidl;
pub mod hal_bridge;
pub mod v4l2_aidl_bridge;

// Re-export buffer management from cam-hal (unified across all crates)
pub use cam_hal::buffer::{
    MappedBuffer, CameraBuffer, BufferAllocator,
    HeapAllocator, HeapBuffer, HeapCameraBuffer, GenericCameraBuffer,
    allocator, set_allocator, allocate, allocate_frame,
};

// Re-export platform-specific buffer backends
#[cfg(feature = "v4l2")]
pub use cam_hal_linux::list_v4l2_devices;
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
