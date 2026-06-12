//! FusedPipeline — higher-level wrapper around any IspEngine backend.
//!
//! Ported from `com.camcore.isp.pipeline.FusedPipeline` (Java).
//! Provides a simpler API for PipelineManager: accepts blocks, builds
//! the best available engine, and processes frames with named parameters.

use crate::blocks;
use crate::engine::{IspEngine, select_engine, default_tone_params};
use crate::pipeline::{IspBlock, IspFrame, GraphComposer};

/// A fused ISP pipeline wrapping a backend engine.
///
/// Auto-selects the best available engine (CPU → MNN → ONNX)
/// and provides a high-level `process()` API.
pub struct FusedPipeline {
    /// The selected backend engine.
    engine: Box<dyn IspEngine>,
    /// Whether the model is loaded.
    loaded: bool,
}

impl FusedPipeline {
    /// Build a new fused pipeline, auto-selecting the best backend.
    pub fn build(
        blocks: Vec<Box<dyn IspBlock>>,
        _target_width: u32,
    ) -> Result<Self, String> {
        let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let _model_bytes = GraphComposer::compose_from_vec(&block_refs, &[], 21)?;

        let mut engine = select_engine().ok_or("No engine available")?;
        let head = blocks::RawInputBlock::new();
        engine.build(Box::new(head), vec![], None, 21)?;
        Ok(Self { engine, loaded: true })
    }

    /// Build using a specific engine.
    pub fn build_with_engine(
        blocks: Vec<Box<dyn IspBlock>>,
        mut engine: Box<dyn IspEngine>,
    ) -> Result<Self, String> {
        let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let _model_bytes = GraphComposer::compose_from_vec(&block_refs, &[], 21)?;
        let head = blocks::RawInputBlock::new();
        engine.build(Box::new(head), vec![], None, 21)?;
        Ok(Self { engine, loaded: true })
    }

    /// Create directly from an existing engine.
    pub fn from_engine(engine: Box<dyn IspEngine>) -> Self {
        let loaded = engine.is_loaded();
        Self { engine, loaded }
    }

    /// Process a raw Bayer frame through the pipeline.
    ///
    /// Named parameters for convenience — delegates to the backend engine.
    #[allow(clippy::too_many_arguments)]
    pub fn process(
        &self,
        width: u32,
        height: u32,
        stride_width: u32,
        buf: &[u8],
        sensor_max: f32,
        target_width: u32,
        ccm_matrix: Option<&[f32; 9]>,
        bayer_gains: Option<&[f32; 4]>,
        awb_gains: Option<&[f32; 3]>,
        analog_gain: f32,
        scene_change: f32,
        lsc_gains: Option<&[f32]>,
        blc_values: Option<&[f32; 4]>,
        warp_grid: Option<&[f32]>,
    ) -> Result<IspFrame, String> {
        if !self.loaded {
            return Err("Pipeline not loaded — call build() first".to_string());
        }
        let tone = default_tone_params();
        self.engine.process(
            width, height, stride_width, buf,
            sensor_max, target_width,
            ccm_matrix,
            &tone,
            bayer_gains,
            awb_gains,
            analog_gain,
            scene_change,
            lsc_gains,
            blc_values,
            warp_grid,
        )
    }

    /// The backend engine.
    pub fn engine(&self) -> &dyn IspEngine {
        self.engine.as_ref()
    }

    /// Backend name.
    pub fn backend_name(&self) -> &str {
        self.engine.backend_name()
    }

    /// Whether the pipeline model is loaded.
    pub fn is_loaded(&self) -> bool {
        self.loaded
    }

    /// Take ownership of the engine.
    pub fn into_engine(self) -> Box<dyn IspEngine> {
        self.engine
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::PipelineProfile;

    #[test]
    fn test_fused_pipeline_cpu() {
        crate::init();
        let profile = PipelineProfile::LITE;
        let blocks = profile.build_blocks(32, 0);

        let pipeline = FusedPipeline::build(blocks, 32);
        assert!(pipeline.is_ok(), "Build failed: {:?}", pipeline.err());
        let pipe = pipeline.unwrap();
        assert!(pipe.is_loaded());
        assert_eq!(pipe.backend_name(), "CPU");
    }
}
