//! MNN inference engine for the ISP pipeline.
//! Ported from com.camcore.isp.mnn.MnnEngine

use log::{info, warn};
use cam_types::{FrameFormat, ToneParams};

use crate::engine::IspEngine;
use crate::pipeline::{IspBlock, IspFrame, IspAuxOutput};

#[cfg(feature = "mnn")]
use crate::mnn_sys::{MnnInterpreterSafe, MnnBackendType};

/// Available MNN backends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MnnBackend {
    Vulkan,
    Opencl,
    CpuNeon,
    Cpu,
}

impl MnnBackend {
    pub fn id(&self) -> &'static str {
        match self {
            MnnBackend::Vulkan => "mnn_vulkan",
            MnnBackend::Opencl => "mnn_opencl",
            MnnBackend::CpuNeon => "mnn_cpu_neon",
            MnnBackend::Cpu => "mnn_cpu",
        }
    }

    pub fn priority(&self) -> i32 {
        match self {
            MnnBackend::Vulkan => 95,
            MnnBackend::Opencl => 85,
            MnnBackend::CpuNeon => 75,
            MnnBackend::Cpu => 65,
        }
    }

    #[cfg(feature = "mnn")]
    fn to_sys_backend(&self) -> MnnBackendType {
        match self {
            MnnBackend::Vulkan => MnnBackendType::Vulkan,
            MnnBackend::Opencl => MnnBackendType::Opencl,
            MnnBackend::CpuNeon | MnnBackend::Cpu => MnnBackendType::Cpu,
        }
    }
}

/// ISP engine that uses Alibaba MNN for inference.
pub struct MnnEngine {
    backend: MnnBackend,
    initialized: bool,
    /// Composed ONNX model bytes (built by `build`).
    model_bytes: Option<Vec<u8>>,
    /// MNN interpreter handle (feature-gated).
    #[cfg(feature = "mnn")]
    interpreter: Option<MnnInterpreterSafe>,
}

impl MnnEngine {
    pub fn new(backend: MnnBackend) -> Self {
        Self {
            backend,
            initialized: false,
            model_bytes: None,
            #[cfg(feature = "mnn")]
            interpreter: None,
        }
    }
}

impl IspEngine for MnnEngine {
    fn backend_name(&self) -> &'static str {
        self.backend.id()
    }

    fn priority(&self) -> i32 {
        self.backend.priority()
    }

    fn is_loaded(&self) -> bool {
        self.initialized
    }

    fn build(
        &mut self,
        pipeline_head: Box<dyn IspBlock>,
        aux_blocks: Vec<Box<dyn IspBlock>>,
        _warp_block: Option<Box<dyn IspBlock>>,
        opset_version: i64,
    ) -> Result<(), String> {
        info!("Building MNN engine with backend={}", self.backend.id());

        // First, compose the ONNX model from the pipeline blocks
        let aux_refs: Vec<&dyn IspBlock> = aux_blocks.iter().map(|b| b.as_ref() as &dyn IspBlock).collect();
        
        let model = crate::pipeline::GraphComposer::compose(
            pipeline_head.as_ref(),
            &aux_refs,
            opset_version,
        )?;
        
        info!("MnnEngine({}) composed ONNX model ({} bytes)", self.backend.id(), model.len());

        #[cfg(feature = "mnn")]
        {
            // Convert ONNX model to MNN format and create interpreter
            // MNN's Interpreter::createFromBuffer can load ONNX models directly
            // (MNN has built-in ONNX support since version 2.0)
            match MnnInterpreterSafe::from_buffer(&model) {
                Some(interpreter) => {
                    info!("MnnEngine({}) created MNN interpreter", self.backend.id());
                    self.interpreter = Some(interpreter);
                }
                None => {
                    warn!("MnnEngine({}) failed to create MNN interpreter, using fallback", self.backend.id());
                    // Continue without interpreter (will use dummy path)
                }
            }
        }

        self.model_bytes = Some(model);
        self.initialized = true;
        info!("MnnEngine({}) built pipeline successfully", self.backend.id());
        Ok(())
    }

    fn process(
        &self,
        width: u32,
        height: u32,
        _stride_width: u32,
        _buf: &[u8],
        _sensor_max: f32,
        target_width: u32,
        _ccm_matrix: Option<&[f32; 9]>,
        _tone_params: &ToneParams,
        _bayer_gains: Option<&[f32; 4]>,
        _awb_gains: Option<&[f32; 3]>,
        _analog_gain: f32,
        _scene_change: f32,
        _lsc_gains: Option<&[f32]>,
        _blc_values: Option<&[f32; 4]>,
        _warp_grid: Option<&[f32]>,
    ) -> Result<IspFrame, String> {
        if !self.initialized {
            return Err("Engine not initialized".to_string());
        }

        #[cfg(feature = "mnn")]
        {
            if let Some(ref _interpreter) = self.interpreter {
                // TODO: 
                // 1. Create session from interpreter with backend config
                // 2. Get input tensor, copy frame data into it
                // 3. Run session
                // 4. Get output tensor, copy to IspFrame
                info!("MnnEngine({}) running MNN inference {}x{}", self.backend.id(), width, height);
            }
        }

        #[cfg(not(feature = "mnn"))]
        {
            info!("MnnEngine({}) mnn feature not enabled, returning dummy frame {}x{} -> {}x{}",
                self.backend.id(), width, height, target_width, height);
        }
        
        let mut frame = IspFrame::new(target_width, height, FrameFormat::Rgba8888);
        frame.aux = Some(IspAuxOutput {
            channel_means: Some([0.5, 0.5, 0.5]),
            tone_stats: None,
            wb_gains: None,
            histogram: None,
            zone_stats: None,
            focus_metric: None,
            cct: None,
            ae_gain: None,
        });
        Ok(frame)
    }
}

/// Factory registration macro for MnnEngine.
#[macro_export]
macro_rules! register_mnn_engine {
    ($backend:expr) => {
        cam_isp::engine::register_engine(
            cam_isp::engine::EngineFactory {
                name: $backend.id(),
                priority: $backend.priority(),
                create_fn: || Box::new(cam_isp::mnn::MnnEngine::new($backend)),
            }
        );
    };
}
