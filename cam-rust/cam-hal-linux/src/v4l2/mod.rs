//! V4L2 implementation using rscam crate.
//!
//! Split into multiple modules for maintainability.

pub mod adapter;
pub mod buffer;
pub mod device;
pub mod format;
pub mod stream;

pub use adapter::V4l2CameraAdapter;
pub use device::list_devices;
