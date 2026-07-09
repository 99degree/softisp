//! ISP pipeline types: block trait, frame, graph composer.
//!
//! This module defines the core types for the ISP processing pipeline:
//! - `IspBlock` trait: interface for processing blocks
//! - `IspFrame`: output frame from the pipeline
//! - `GraphComposer`: builds ONNX models from block chains

pub mod base;
pub mod build;
pub mod types;
pub mod traits;
pub use types::*;
pub use traits::*;
pub use build::*;
