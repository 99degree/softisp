//! Backward-compatible re-export module for the pipeline builder.
//!
//! The `PipelineBuilder` was moved to `pipeline::builder` and `pipeline::types`.
//! This file provides backward-compatible imports for existing code.

pub use crate::pipeline::builder::PipelineBuilder;
pub use crate::pipeline::builder::PipelineError;
