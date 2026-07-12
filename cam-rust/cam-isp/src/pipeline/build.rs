//! Pipeline build module — shared engine building utilities.
//!
//! Provides common helper functions for building ISP engines from blocks.
//! Used by both `FusedPipeline` and `UnifiedPipeline`.

use super::IspBlock;
use crate::engine::{select_engine, IspEngine};
use crate::error::IspResult;

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

    let mut engine =
        select_engine().ok_or(crate::error::IspError::Config("No engine available".into()))?;

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

/// Common result type for pipeline operations.
pub type PipelineResult<T> = IspResult<T>;
