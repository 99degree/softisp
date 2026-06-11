//! Backend-agnostic inference engine for the ISP pipeline.
//! Ported from com.camcore.isp.engine.IspEngine

use std::sync::Mutex;
use std::fmt;
use log::info;
use cam_types::ToneParams;

use crate::pipeline::{IspBlock, IspFrame};

/// Factory for creating an IspEngine instance.
#[derive(Clone)]
pub struct EngineFactory {
    pub name: &'static str,
    pub priority: i32,
    pub create_fn: fn() -> Box<dyn IspEngine>,
}

impl fmt::Debug for EngineFactory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EngineFactory")
            .field("name", &self.name)
            .field("priority", &self.priority)
            .finish()
    }
}

/// Trait for an ISP processing engine.
/// Implementations compile an IspBlock chain into a fused model and run per-frame inference.
pub trait IspEngine: Send + Sync {
    fn backend_name(&self) -> &'static str;
    fn priority(&self) -> i32;
    fn is_loaded(&self) -> bool;
    
    /// Build the engine from a pipeline of blocks.
    fn build(
        &mut self,
        pipeline_head: Box<dyn IspBlock>,
        aux_blocks: Vec<Box<dyn IspBlock>>,
        warp_block: Option<Box<dyn IspBlock>>,
        opset_version: i64,
    ) -> Result<(), String>;

    /// Process a raw frame through the pipeline.
    #[allow(clippy::too_many_arguments)]
    fn process(
        &self,
        width: u32,
        height: u32,
        stride_width: u32,
        buf: &[u8],
        sensor_max: f32,
        target_width: u32,
        ccm_matrix: Option<&[f32; 9]>,
        tone_params: &ToneParams,
        bayer_gains: Option<&[f32; 4]>,
        awb_gains: Option<&[f32; 3]>,
        analog_gain: f32,
        scene_change: f32,
        lsc_gains: Option<&[f32]>,
        blc_values: Option<&[f32; 4]>,
        warp_grid: Option<&[f32]>,
    ) -> Result<IspFrame, String>;
}

/// Global registry of engine factories.
static REGISTRY: std::sync::LazyLock<Mutex<Vec<EngineFactory>>> =
    std::sync::LazyLock::new(|| Mutex::new(Vec::new()));

/// Register an engine factory.
pub fn register_engine(factory: EngineFactory) {
    let name = factory.name;
    let priority = factory.priority;
    let mut registry = REGISTRY.lock().unwrap();
    registry.push(factory);
    // Sort by priority descending
    registry.sort_by(|a, b| b.priority.cmp(&a.priority));
    info!("Registered engine: {} (priority={})", name, priority);
}

/// Select the best available engine by priority.
pub fn select_engine() -> Option<Box<dyn IspEngine>> {
    let registry = REGISTRY.lock().unwrap();
    for factory in registry.iter() {
        let engine = (factory.create_fn)();
        info!("Trying engine: {} (priority={})", factory.name, factory.priority);
        // In a real implementation, we would check if the engine can be initialized.
        // For now, we return the first one.
        if engine.priority() > 0 {
            return Some(engine);
        }
    }
    None
}