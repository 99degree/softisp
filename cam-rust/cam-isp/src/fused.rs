//! FusedPipeline — higher-level wrapper around any IspEngine backend.
//!
//! Ported from `com.camcore.isp.pipeline.FusedPipeline` (Java).
//! Provides a simpler API for PipelineManager: accepts blocks, builds
//! the best available engine, and processes frames with named parameters.

use crate::engine::{IspEngine, ProcessParams};
use crate::error::IspResult;
use crate::pipeline::{IspBlock, IspFrame};
use crate::pipeline::build::{build_engine, build_engine_with};
use crate::pipeline::traits::{ProcessPipeline, BuildablePipeline};

/// A fused ISP pipeline wrapping a backend engine.
///
/// Auto-selects the best available engine (CPU → MNN → ONNX)
/// and provides a high-level `process()` API.
pub struct FusedPipeline {
    /// The selected backend engine.
    engine: Box<dyn IspEngine>,
}

impl FusedPipeline {
    /// Build using a specific engine.
    pub fn build_with_engine(
        blocks: Vec<Box<dyn IspBlock>>,
        engine: Box<dyn IspEngine>,
    ) -> IspResult<Self> {
        let engine = build_engine_with(blocks, engine)?;
        Ok(Self { engine })
    }

    /// Create directly from an existing engine.
    pub fn from_engine(engine: Box<dyn IspEngine>) -> Self {
        Self { engine }
    }

    /// Take ownership of the engine.
    pub fn into_engine(self) -> Box<dyn IspEngine> {
        self.engine
    }
}

impl ProcessPipeline for FusedPipeline {
    fn process(&self, params: &ProcessParams) -> IspResult<IspFrame> {
        if !self.engine.is_loaded() {
            return Err(crate::error::IspError::Pipeline(
                "Pipeline not loaded — call build() first".into(),
            ));
        }
        self.engine.process(params)
    }

    fn engine(&self) -> &dyn IspEngine {
        self.engine.as_ref()
    }

    fn is_loaded(&self) -> bool {
        self.engine.is_loaded()
    }
}

impl BuildablePipeline for FusedPipeline {
    type Output = Self;

    fn build(blocks: Vec<Box<dyn IspBlock>>) -> IspResult<Self> {
        let engine = build_engine(blocks)?;
        Ok(Self { engine })
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
        let pipeline = FusedPipeline::build_with_engine(
            blocks,
            Box::new(crate::cpu::CpuEngine::new()),
        );
        assert!(
            pipeline.is_ok(),
            "Build failed: {:?}",
            pipeline.err()
        );
        let pipe = pipeline.unwrap();
        assert!(pipe.is_loaded());
        assert_eq!(pipe.backend_name(), "CPU");
    }

    #[test]
    fn test_fused_pipeline_from_engine() {
        crate::init();
        let engine = Box::new(crate::cpu::CpuEngine::new());
        let pipe = FusedPipeline::from_engine(engine);
        assert_eq!(pipe.backend_name(), "CPU");
    }

    #[test]
    fn test_fused_pipeline_into_engine() {
        crate::init();
        let profile = PipelineProfile::LITE;
        let blocks = profile.build_blocks(32, 0);
        let pipe = FusedPipeline::build_with_engine(
            blocks,
            Box::new(crate::cpu::CpuEngine::new()),
        )
        .unwrap();
        let _engine = pipe.into_engine();
    }

    #[test]
    fn test_fused_pipeline_build() {
        crate::init();
        let profile = PipelineProfile::MED;
        let blocks = profile.build_blocks(32, 0);
        let pipe = FusedPipeline::build(blocks);
        assert!(
            pipe.is_ok(),
            "MED profile build failed: {:?}",
            pipe.err()
        );
        let p = pipe.unwrap();
        assert!(p.is_loaded());
        assert!(!p.engine().backend_name().is_empty());
    }

    #[test]
    fn test_fused_pipeline_process() {
        crate::init();
        let profile = PipelineProfile::LITE;
        let blocks = profile.build_blocks(32, 0);
        let pipe = FusedPipeline::build_with_engine(
            blocks,
            Box::new(crate::cpu::CpuEngine::new()),
        )
        .unwrap();
        let raw = vec![0u8; 32 * 32 * 2];
        let params = ProcessParams::new(32, 32, &raw);
        let result = pipe.process(&params);
        assert!(
            result.is_ok(),
            "process failed: {:?}",
            result.err()
        );
    }

    #[test]
    fn test_fused_pipeline_with_all_profiles() {
        crate::init();
        let profiles = vec![
            (PipelineProfile::LITE, "LITE"),
            (PipelineProfile::MED, "MED"),
            (PipelineProfile::HEAVY, "HEAVY"),
            (PipelineProfile::PRO, "PRO"),
        ];
        for (profile, name) in profiles {
            let blocks = profile.build_blocks(64, 0);
            let pipe = FusedPipeline::build_with_engine(
                blocks,
                Box::new(crate::cpu::CpuEngine::new()),
            );
            assert!(
                pipe.is_ok(),
                "{} profile build failed: {:?}",
                name,
                pipe.err()
            );
            let pipe = pipe.unwrap();
            assert!(pipe.is_loaded(), "{} profile not loaded", name);
            let raw = vec![128u8; 64 * 64 * 2];
            let params = ProcessParams::new(64, 64, &raw);
            let result = pipe.process(&params);
            assert!(
                result.is_ok(),
                "{} profile process failed: {:?}",
                name,
                result.err()
            );
            let frame = result.unwrap();
            assert_eq!(frame.width, 64, "{} profile width mismatch", name);
            assert_eq!(frame.height, 64, "{} profile height mismatch", name);
            println!(
                "{}: OK - {} bytes output",
                name,
                frame.data.len()
            );
        }
    }

    #[test]
    fn test_fused_pipeline_process_bayer() {
        crate::init();
        let profile = PipelineProfile::LITE;
        let blocks = profile.build_blocks(64, 0);
        let mut pipe = FusedPipeline::build_with_engine(
            blocks,
            Box::new(crate::cpu::CpuEngine::new()),
        )
        .unwrap();
        let raw = vec![128u8; 64 * 64 * 2];
        let result = pipe.process_bayer(&raw, 64, 64);
        assert!(
            result.is_ok(),
            "process_bayer failed: {:?}",
            result.err()
        );
        let frame = result.unwrap();
        assert_eq!(frame.width, 64);
        assert_eq!(frame.height, 64);
    }
}
