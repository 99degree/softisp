//! Pipeline helper module — shared utilities for ISP pipelines.
//!
//! Provides common helper functions used by both `FusedPipeline`
//! and `UnifiedPipeline` to reduce code duplication.

use crate::engine::{IspEngine, ProcessParams, select_engine};
use crate::error::IspResult;
use crate::pipeline::IspBlock;

/// Build an ISP engine from blocks using auto-selected backend.
///
/// This is the common pattern used by both FusedPipeline and UnifiedPipeline.
pub fn build_engine(blocks: Vec<Box<dyn IspBlock>>) -> IspResult<Box<dyn IspEngine>> {
    if blocks.is_empty() {
        return Err(crate::error::IspError::Pipeline(
            "No blocks provided".into(),
        ));
    }

    let mut blocks = blocks;
    let head = blocks.remove(0);
    let aux_blocks = blocks;

    let mut engine = select_engine()
        .ok_or(crate::error::IspError::Config("No engine available".into()))?;

    engine.build(head, aux_blocks, None, 21)?;
    Ok(engine)
}

/// Build an ISP engine with a specific engine.
pub fn build_engine_with(
    blocks: Vec<Box<dyn IspBlock>>,
    mut engine: Box<dyn IspEngine>,
) -> IspResult<Box<dyn IspEngine>> {
    if blocks.is_empty() {
        return Err(crate::error::IspError::Pipeline(
            "No blocks provided".into(),
        ));
    }

    let mut blocks = blocks;
    let head = blocks.remove(0);
    let aux_blocks = blocks;

    engine.build(head, aux_blocks, None, 21)?;
    Ok(engine)
}
