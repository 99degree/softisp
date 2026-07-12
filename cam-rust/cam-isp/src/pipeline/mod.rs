//! ISP pipeline module — complete pipeline infrastructure.
//!
//! This module defines all types and utilities for the ISP processing pipeline:
//! - Core types: `IspBlock`, `IspFrame`, `GraphComposer`
//! - Traits: `ProcessPipeline`, `BuildablePipeline`
//! - Builder: fluent API for constructing pipelines
//! - Diff: compare pipeline configurations
//! - Snapshot: save/restore pipeline state for fast restart

pub mod base;
pub mod build;
pub mod builder;
pub mod diff;
pub mod snapshot;
pub mod traits;
pub mod types;

pub use build::*;
pub use builder::PipelineBuilder;
pub use diff::PipelineDiff;
pub use snapshot::PipelineSnapshot;
pub use traits::*;
pub use types::*;
