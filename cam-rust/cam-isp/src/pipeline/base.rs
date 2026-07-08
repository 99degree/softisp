//! Base pipeline module — shared functionality for all ISP pipelines.
//!
//! Provides common types and utilities used by both `FusedPipeline`
//! and `UnifiedPipeline`.

use crate::engine::{IspEngine, ProcessParams};
use crate::error::IspResult;
use super::IspFrame;

/// Base struct for pipelines that wrap an IspEngine.
///
/// This provides common functionality for building and managing
/// ISP pipelines. Both `FusedPipeline` and `UnifiedPipeline` can
/// use this as their base.
pub struct EnginePipeline {
    pub engine: Box<dyn IspEngine>,
    pub loaded: bool,
}

impl EnginePipeline {
    /// Create a new engine pipeline.
    pub fn new(mut engine: Box<dyn IspEngine>, loaded: bool) -> Self {
        Self { engine, loaded }
    }

    /// Create from an existing engine (assumes already loaded).
    pub fn from_engine(engine: Box<dyn IspEngine>) -> Self {
        let loaded = engine.is_loaded();
        Self { engine, loaded }
    }

    /// Build from blocks using auto-selected engine.
    pub fn build(
        blocks: Vec<Box<dyn super::IspBlock>>,
        target_width: u32,
    ) -> IspResult<Self> {
        if blocks.is_empty() {
            return Err(crate::error::IspError::Pipeline(
                "No blocks provided".into(),
            ));
        }

        let mut blocks = blocks;
        let head = blocks.remove(0);
        let aux_blocks = blocks;

        let mut engine = crate::engine::select_engine()
            .ok_or(crate::error::IspError::Config("No engine available".into()))?;

        engine.build(head, aux_blocks, None, 21)?;

        Ok(Self {
            engine,
            loaded: true,
        })
    }

    /// Build with a specific engine.
    pub fn build_with_engine(
        blocks: Vec<Box<dyn super::IspBlock>>,
        mut engine: Box<dyn IspEngine>
    ) -> IspResult<Self> {
        if blocks.is_empty() {
            return Err(crate::error::IspError::Pipeline(
                "No blocks provided".into(),
            ));
        }

        let mut blocks = blocks;
        let head = blocks.remove(0);
        let aux_blocks = blocks;

        let loaded = engine.is_loaded();
        engine.build(head, aux_blocks, None, 21)?;

        Ok(Self { engine, loaded })
    }

    /// Process a frame with the given parameters.
    pub fn process_params(&self, params: &ProcessParams) -> IspResult<IspFrame> {
        if !self.loaded {
            return Err(crate::error::IspError::Pipeline(
                "Pipeline not loaded — call build() first".into(),
            ));
        }
        self.engine.process(params)
    }

    /// Get the backend engine.
    pub fn engine(&self) -> &dyn IspEngine {
        self.engine.as_ref()
    }

    /// Get the backend name.
    pub fn backend_name(&self) -> &str {
        self.engine().backend_name()
    }

    /// Whether the pipeline model is loaded.
    pub fn is_loaded(&self) -> bool {
        self.loaded
    }
}
