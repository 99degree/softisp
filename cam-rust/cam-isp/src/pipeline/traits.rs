//! Common traits for ISP pipelines.
//!
//! Defines shared interfaces that all pipeline implementations
//! (FusedPipeline, UnifiedPipeline, PipelineBuilder) can implement.

use crate::engine::{IspEngine, ProcessParams};
use crate::error::IspResult;
use crate::pipeline::IspFrame;

/// Trait for pipelines that can process frames.
pub trait ProcessPipeline {
    /// Process a frame with the given parameters.
    fn process(&self, params: &ProcessParams) -> IspResult<IspFrame>;

    /// Process raw Bayer data directly.
    fn process_bayer(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
    ) -> IspResult<IspFrame> {
        let mut params = ProcessParams::new(width, height, raw_data);
        self.process(&params)
    }

    /// Get the backend engine.
    fn engine(&self) -> &dyn IspEngine;

    /// Get the backend name.
    fn backend_name(&self) -> &str {
        self.engine().backend_name()
    }

    /// Whether the pipeline is loaded/ready.
    fn is_loaded(&self) -> bool;
}

/// Trait for pipelines that can be built from blocks.
pub trait BuildablePipeline {
    /// Type of the pipeline.
    type Output;

    /// Build from blocks.
    fn build(blocks: Vec<Box<dyn super::IspBlock>>) -> IspResult<Self::Output>;

    /// Build from blocks with target width.
    fn build_with_size(
        blocks: Vec<Box<dyn super::IspBlock>>,
        target_width: u32,
    ) -> IspResult<Self::Output> {
        Self::build(blocks)
    }
}

/// Trait for pipelines that can be serialized.
pub trait SerializedPipeline {
    /// Get the ONNX bytes for this pipeline.
    fn onnx_bytes(&self) -> Option<&[u8]>;

    /// Get the MNN model path (if cached).
    #[cfg(feature = "mnn")]
    fn mnn_path(&self) -> Option<&std::path::Path>;
}

/// Common configuration for all pipelines.
#[derive(Debug, Clone)]
pub struct PipelineConfigCommon {
    pub width: u32,
    pub height: u32,
    pub bayer_pattern: i32,
    pub sensor_max: f32,
}

impl Default for PipelineConfigCommon {
    fn default() -> Self {
        Self {
            width: 1920,
            height: 1080,
            bayer_pattern: 0, // RGGB
            sensor_max: 1023.0,
        }
    }
}

impl PipelineConfigCommon {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            ..Default::default()
        }
    }

    pub fn with_bayer_pattern(mut self, pattern: i32) -> Self {
        self.bayer_pattern = pattern;
        self
    }

    pub fn with_sensor_max(mut self, max: f32) -> Self {
        self.sensor_max = max;
        self
    }
}
