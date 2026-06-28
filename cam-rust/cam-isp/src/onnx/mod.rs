//! ONNX Runtime integration and ONNX protobuf utilities.
//! Ported from com.camcore.isp.onnx

pub mod proto;

use proto::Proto;
use log::info;
#[cfg(feature = "ort")]
use log::warn;
use cam_types::FrameFormat;

use std::sync::Mutex;
use crate::controller::IspController;
use crate::engine::{IspEngine, ProcessParams};
use crate::pipeline::{IspBlock, IspFrame, GraphComposer};

// ---------------------------------------------------------------------------
// OrtBackend - available ONNX Runtime execution backends
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
// OnnxEngine - ISP inference via ONNX Runtime
// ---------------------------------------------------------------------------

/// ISP engine that uses ONNX Runtime for inference.
pub struct OnnxEngine {
    backend: OrtBackend,
    initialized: bool,
    /// Composed ONNX model bytes (built by `build`).
    model_bytes: Option<Vec<u8>>,
    /// ORT session (feature-gated), wrapped in Mutex for interior mutability.
    #[cfg(feature = "ort")]
    session: std::sync::Mutex<Option<::ort::session::Session>>,
    /// ISP controller for stats feedback (shared across engines).
    pub controller: Mutex<IspController>,
}

impl OnnxEngine {
    pub fn new(backend: OrtBackend) -> Self {
        Self {
            backend,
            initialized: false,
            model_bytes: None,
            controller: Mutex::new(IspController::new()),
            #[cfg(feature = "ort")]
            session: std::sync::Mutex::new(None),
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

    fn as_any(&self) -> &dyn std::any::Any { self }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any { self }

    fn controller(&self) -> &Mutex<IspController> {
        &self.controller
    }

    fn build(
        &mut self,
        pipeline_head: Box<dyn IspBlock>,
        aux_blocks: Vec<Box<dyn IspBlock>>,
        _warp_block: Option<Box<dyn IspBlock>>,
        opset_version: i64,
    ) -> Result<(), String> {
        // Build full pipeline: head + aux_blocks
        let mut all_blocks: Vec<&dyn IspBlock> = vec![pipeline_head.as_ref()];
        let aux_refs: Vec<&dyn IspBlock> = aux_blocks.iter().map(|b| b.as_ref() as &dyn IspBlock).collect();
        all_blocks.extend(aux_refs);
        
        eprintln!("OnnxEngine::build: all_blocks count = {}", all_blocks.len());
        
        // Compose using compose_from_vec to avoid linked list requirement
        let model = GraphComposer::compose_from_vec(&all_blocks, &[], opset_version)?;

        eprintln!("OnnxEngine::build: model size = {} bytes", model.len());
        info!("OnnxEngine({}) composed ONNX model ({} bytes)", self.backend.id(), model.len());

        #[cfg(feature = "ort")]
        {
            // Initialize ORT environment (auto-created if not already done)
            // ort::init() already called by cam_isp::init()

            // Build session from model
            let session = ::ort::session::Session::builder()
                .and_then(|mut b| b.commit_from_memory(&model))
                .map_err(|e| format!("OnnxEngine({}) failed to load ORT session: {}", self.backend.id(), e))?;
            info!("OnnxEngine({}) loaded ORT session", self.backend.id());
            let mut guard = self.session.lock().unwrap();
            *guard = Some(session);
        }

        self.model_bytes = Some(model);
        self.initialized = true;
        info!("OnnxEngine({}) built pipeline successfully", self.backend.id());
        Ok(())
    }

    fn process(&self, p: &ProcessParams) -> Result<IspFrame, String> {
        let width = p.width;
        let height = p.height;
        let target_width = p.target_width;
        let _buf = p.buf;

        if !self.initialized {
            return Err("Engine not initialized".to_string());
        }

        eprintln!("OnnxEngine::process called, feature ort={}, initialized={}", cfg!(feature = "ort"), self.initialized);

        info!("OnnxEngine({}) processing frame {}x{} -> {}x{}",
            self.backend.id(), width, height, target_width, height);

        #[cfg(feature = "ort")]
        {
            let buf = _buf;
            let mut guard = self.session.lock().unwrap();
            eprintln!("OnnxEngine: session lock acquired, session is_some={}", guard.is_some());
            if let Some(ref mut session) = *guard {
                // Build input tensor: INT16 [1, 1, height, width]
                // Reinterpret raw u8 bytes as i16
                let input_i16: Vec<i16> = unsafe {
                    std::slice::from_raw_parts(
                        buf.as_ptr() as *const i16,
                        buf.len() / 2,
                    )
                }.to_vec();

                eprintln!("OnnxEngine: input_i16 len={}, buf len={}", input_i16.len(), buf.len());

                let tensor = match ::ort::value::Tensor::from_array(
                    (vec![1i64, 1, height as i64, width as i64], input_i16.into_boxed_slice())
                ) {
                    Ok(t) => t.upcast(),
                    Err(e) => {
                        warn!("OnnxEngine: failed to create input tensor: {}", e);
                        return Err(format!("Failed to create input tensor: {}", e));
                    }
                };

                info!("OnnxEngine: input tensor created successfully");

                // Run inference
                let input_name = "RawInputBlock/frame";
                let output_name = "DisplayBlock/frame";
                eprintln!("OnnxEngine: running inference with input='{}', output='{}'", input_name, output_name);
                let outputs = match session.run(ort::inputs![input_name => tensor]) {
                    Ok(o) => {
                        eprintln!("OnnxEngine: session.run() returned Ok, outputs count={}", o.len());
                        o
                    }
                    Err(e) => {
                        eprintln!("OnnxEngine: inference failed: {}", e);
                        return Err(format!("ORT inference failed: {}", e));
                    }
                };

                eprintln!("OnnxEngine: inference completed, outputs count={}", outputs.len());

                // Extract output tensor (UINT8 BGRA)
                if let Some(output_val) = outputs.get(output_name) {
                    eprintln!("OnnxEngine: output tensor '{}' found", output_name);
                    match output_val.try_extract_tensor::<u8>() {
                        Ok((_, data)) => {
                            let output_data = data.to_vec();
                            eprintln!("OnnxEngine: inference done, {} bytes", output_data.len());
                            let mut frame = IspFrame::new(target_width, height, FrameFormat::Rgba8888);
                            frame.data = output_data;
                            return Ok(frame);
                        }
                        Err(e) => {
                            eprintln!("OnnxEngine: failed to extract output: {}", e);
                        }
                    }
                } else {
                    eprintln!("OnnxEngine: output '{}' not found", output_name);
                    // List available output names
                    for (i, (name, _)) in outputs.iter().enumerate() {
                        eprintln!("OnnxEngine: available output [{}]: '{}'", i, name);
                    }
                }
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
        nodes: Vec<Vec<u8>>,
        initializers: Vec<Vec<u8>>,
        input_value_infos: Vec<Vec<u8>>,
        output_value_infos: Vec<Vec<u8>>,
    ) -> Vec<u8> {
        // Build graph bytes
        let mut graph = Vec::new();
        // Nodes
        for node in &nodes {
            graph.extend_from_slice(&Proto::raw_bytes(1, node));
        }
        // Name
        graph.extend_from_slice(&Proto::string(1, "isp_graph"));
        // Inputs
        for vi in &input_value_infos {
            graph.extend_from_slice(&Proto::raw_bytes(11, vi));
        }
        // Outputs
        for vi in &output_value_infos {
            graph.extend_from_slice(&Proto::raw_bytes(11, vi));
        }
        // Initializers
        for init in &initializers {
            graph.extend_from_slice(&Proto::raw_bytes(5, init));
        }

        // Build opset
        let opset = Proto::opset("", 13);

        // Build model
        Proto::model(8, &opset, "softisp", &graph)
    }
}

// ---------------------------------------------------------------------------
// Macro - register an ONNX engine in the global registry
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
