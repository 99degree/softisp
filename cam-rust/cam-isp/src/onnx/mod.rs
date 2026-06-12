//! ONNX Runtime integration and ONNX protobuf utilities.
//! Ported from com.camcore.isp.onnx

pub mod proto;


use log::info;
#[cfg(feature = "ort")]
use log::warn;
use cam_types::{FrameFormat, ToneParams};

use crate::engine::IspEngine;
use crate::pipeline::{IspBlock, IspFrame, GraphComposer};

// ---------------------------------------------------------------------------
// OrtBackend — available ONNX Runtime execution backends
// ---------------------------------------------------------------------------

/// Available ONNX Runtime execution providers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OrtBackend {
    Nnapi,
    Xnnpack,
    Cpu,
    #[allow(dead_code)]
    Tensorrt,
    #[allow(dead_code)]
    Coreml,
}

impl OrtBackend {
    pub fn id(&self) -> &'static str {
        match self {
            OrtBackend::Nnapi => "onnx_nnapi",
            OrtBackend::Xnnpack => "onnx_xnnpack",
            OrtBackend::Cpu => "onnx_cpu",
            OrtBackend::Tensorrt => "onnx_tensorrt",
            OrtBackend::Coreml => "onnx_coreml",
        }
    }

    pub fn priority(&self) -> i32 {
        match self {
            OrtBackend::Nnapi => 90,
            OrtBackend::Xnnpack => 80,
            OrtBackend::Cpu => 70,
            OrtBackend::Tensorrt => 85,
            OrtBackend::Coreml => 75,
        }
    }
}

// ---------------------------------------------------------------------------
// OnnxEngine — ISP inference via ONNX Runtime
// ---------------------------------------------------------------------------

/// ISP engine that uses ONNX Runtime for inference.
pub struct OnnxEngine {
    backend: OrtBackend,
    initialized: bool,
    /// Composed ONNX model bytes (built by `build`).
    model_bytes: Option<Vec<u8>>,
    /// ORT session (feature-gated).
    #[cfg(feature = "ort")]
    session: Option<::ort::Session>,
}

impl OnnxEngine {
    pub fn new(backend: OrtBackend) -> Self {
        Self {
            backend,
            initialized: false,
            model_bytes: None,
            #[cfg(feature = "ort")]
            session: None,
        }
    }
}

impl IspEngine for OnnxEngine {
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
        let aux_refs: Vec<&dyn IspBlock> = aux_blocks.iter().map(|b| b.as_ref() as &dyn IspBlock).collect();
        
        let model = GraphComposer::compose(
            pipeline_head.as_ref(),
            &aux_refs,
            opset_version,
        )?;
        
        info!("OnnxEngine({}) composed ONNX model ({} bytes)", self.backend.id(), model.len());
        
        #[cfg(feature = "ort")]
        {
            // Load the model into an ORT session
            let env = ::ort::EnvironmentBuilder::new()
                .with_name("cam_isp_onnx")
                .build()
                .map_err(|e| format!("Failed to create ORT environment: {}", e))?;
            
            let mut options = ::ort::SessionOptionsBuilder::new()
                .with_intra_op_num_threads(4)
                .map_err(|e| format!("Failed to create session options: {}", e))?;
            
            // Enable NNAPI if available
            if matches!(self.backend, OrtBackend::Nnapi) {
                let _ = options.with_nnapi(false, None);
            }
            
            let options = options
                .build()
                .map_err(|e| format!("Failed to build session options: {}", e))?;
            
            match ::ort::Session::from_buffer(&model, env, options) {
                Ok(session) => {
                    info!("OnnxEngine({}) loaded ORT session", self.backend.id());
                    self.session = Some(session);
                }
                Err(e) => {
                    warn!("OnnxEngine({}) failed to load ORT session: {}", self.backend.id(), e);
                    // Continue without session (will use CPU fallback)
                }
            }
        }
        
        self.model_bytes = Some(model);
        self.initialized = true;
        info!("OnnxEngine({}) built pipeline successfully", self.backend.id());
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
        
        info!("OnnxEngine({}) processing frame {}x{} -> {}x{}",
            self.backend.id(), width, height, target_width, height);
        
        #[cfg(feature = "ort")]
        {
            if let Some(ref session) = self.session {
                // Run inference using the ORT session
                // TODO: Create input tensor, run session, extract output tensor
                // This requires mapping pipeline inputs to ORT tensor names
                info!("OnnxEngine: running ORT inference");
            }
        }
        
        #[cfg(not(feature = "ort"))]
        {
            info!("OnnxEngine: ort feature not enabled, returning dummy frame");
        }
        
        let frame = IspFrame::new(target_width, height, FrameFormat::Rgba8888);
        Ok(frame)
    }
}

// ---------------------------------------------------------------------------
// ONNX model composer (model generation from blocks)
// ---------------------------------------------------------------------------

/// ONNX model composer for the ISP pipeline.
pub struct OnnxModelComposer;

impl OnnxModelComposer {
    /// Create an ONNX graph from nodes and initializers.
    pub fn compose_model(
        _nodes: Vec<Vec<u8>>,
        _initializers: Vec<Vec<u8>>,
        _input_value_infos: Vec<Vec<u8>>,
        _output_value_infos: Vec<Vec<u8>>,
    ) -> Vec<u8> {
        // TODO: Generate ONNX protobuf model bytes.
        vec![]
    }
}

// ---------------------------------------------------------------------------
// Macro — register an ONNX engine in the global registry
// ---------------------------------------------------------------------------

/// Factory registration macro for OnnxEngine.
#[macro_export]
macro_rules! register_onnx_engine {
    ($backend:expr) => {
        cam_isp::engine::register_engine(
            cam_isp::engine::EngineFactory {
                name: $backend.id(),
                priority: $backend.priority(),
                create_fn: || Box::new(cam_isp::onnx::OnnxEngine::new($backend)),
            }
        );
    };
}
