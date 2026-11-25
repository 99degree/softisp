//! Engine manager wrapping MnnEngine/DualGpuEngine

use cam_isp::{
    engine::{IspEngine, OutputFormat, ProcessParams, BackendCapabilities},
    mnnengine::{MnnEngine, MnnBackend},
    profile::PipelineProfile,
    controller::IspController,
    pipeline::IspBlock,
    blocks::*,
};
use std::sync::{Arc, Mutex};
use crate::{IpaConfig, IpaResult, IpaError, IpaFormat, EngineBackend};

/// Manages the ISP engine lifecycle
pub struct EngineManager {
    engine: Box<dyn IspEngine>,
    config: IpaConfig,
    controller: Arc<Mutex<IspController>>,
    is_built: bool,
    profile: PipelineProfile,
}

impl EngineManager {
    /// Create a new engine manager
    pub fn new(config: IpaConfig) -> IpaResult<Self> {
        let backend = match config.backend {
            EngineBackend::MnnVulkan => MnnBackend::Vulkan,
            EngineBackend::Cpu => {
                return Err(IpaError::InvalidConfig("CPU backend not yet implemented".into()));
            }
            EngineBackend::OnnxRuntime => {
                return Err(IpaError::InvalidConfig("ONNX Runtime backend not yet implemented".into()));
            }
        };
        
        let mut engine = MnnEngine::new(backend);
        
        // Share controller with engine
        let controller = Arc::new(Mutex::new(IspController::new()));
        engine.controller = controller.clone();
        
        let profile = config.profile;
        
        Ok(Self {
            engine: Box::new(engine),
            config,
            controller,
            is_built: false,
            profile,
        })
    }
    
    /// Build the pipeline from blocks using the selected PipelineProfile
    pub fn build(&mut self) -> IpaResult<()> {
        let width = self.config.image_width;
        let height = self.config.image_height;
        
        // Build the main pipeline blocks from the profile
        let main_blocks = self.profile.build_blocks(width, height, 0);
        
        if main_blocks.is_empty() {
            return Err(IpaError::InvalidConfig("Profile produced no blocks".into()));
        }
        
        // Split into head and aux
        let mut main_blocks = main_blocks;
        let head = main_blocks.remove(0);
        let aux_blocks: Vec<Box<dyn IspBlock>> = main_blocks;
        
        // Build aux stats blocks
        let stats_aux = self.profile.build_aux_blocks(height as i64, width as i64);
        let all_aux: Vec<Box<dyn IspBlock>> = aux_blocks.into_iter().chain(stats_aux).collect();
        
        self.engine.build(head, all_aux, None, 18)?;
        
        self.is_built = true;
        Ok(())
    }
    
    /// Process a frame through the engine
    pub fn process(&self, params: ProcessParams) -> IpaResult<cam_isp::pipeline::IspFrame> {
        if !self.is_built {
            return Err(IpaError::NotInitialized.into());
        }
        self.engine.process(&params).map_err(|e| e.into())
    }
    
    /// Get reference to controller for 3A updates
    pub fn controller(&self) -> Arc<Mutex<IspController>> {
        self.controller.clone()
    }
    
    /// Check if engine is built and ready
    pub fn is_ready(&self) -> bool {
        self.is_built && self.engine.is_loaded()
    }
    
    /// Get backend name
    pub fn backend_name(&self) -> &'static str {
        self.engine.backend_name()
    }
    
    /// Convert IpaFormat to OutputFormat
    fn ipa_format_to_output_format(&self, format: IpaFormat) -> OutputFormat {
        match format {
            IpaFormat::RGB => OutputFormat::Rgb,
            IpaFormat::RGBA => OutputFormat::Rgba,
            IpaFormat::BGRA => OutputFormat::Bgra,
            IpaFormat::ARGB => OutputFormat::Argb,
            IpaFormat::ABGR => OutputFormat::Abgr,
            IpaFormat::YUV | IpaFormat::NV12 | IpaFormat::P010 => {
                // YUV formats not directly supported, fall back to BGRA
                OutputFormat::Bgra
            }
        }
    }
    
    /// Get capabilities
    pub fn capabilities(&self) -> BackendCapabilities {
        BackendCapabilities::probe(self.engine.backend_name())
    }
    
    /// Get the active profile
    pub fn profile(&self) -> PipelineProfile {
        self.profile
    }
}
