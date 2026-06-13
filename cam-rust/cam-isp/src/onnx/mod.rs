//! ONNX Runtime integration and ONNX protobuf utilities.
//! Ported from com.camcore.isp.onnx

pub mod proto;

use std::time::Instant;
use log::{info, debug, warn, error};
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
    /// ORT session (feature-gated), wrapped in Mutex for interior mutability.
    #[cfg(feature = "ort")]
    session: std::sync::Mutex<Option<::ort::session::Session>>,
}

impl OnnxEngine {
    pub fn new(backend: OrtBackend) -> Self {
        Self {
            backend,
            initialized: false,
            model_bytes: None,
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
            // Initialize ORT environment (auto-created if not already done)
            // ort::init() already called by cam_isp::init()

            // Build session from model
            match ::ort::session::Session::builder()
                .and_then(|mut b| b.commit_from_memory(&model))
            {
                Ok(session) => {
                    info!("OnnxEngine({}) loaded ORT session", self.backend.id());
                    let mut guard = self.session.lock().unwrap();
                    *guard = Some(session);
                }
                Err(e) => {
                    warn!("OnnxEngine({}) failed to load ORT session: {}", self.backend.id(), e);
                    // Continue without session (will use fallback)
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
        let t0 = Instant::now();
        
        if !self.initialized {
            error!("OnnxEngine({}) process called before build()", self.backend.id());
            return Err("Engine not initialized".to_string());
        }
        
        debug!("OnnxEngine({}) processing frame {}x{} -> {}x{}",
            self.backend.id(), width, height, target_width, height);
        
        #[cfg(feature = "ort")]
        {
            let buf = _buf;
            let mut guard = self.session.lock().unwrap();
            if let Some(ref mut session) = *guard {
                // Build input tensor: INT16 [1, 1, height, width]
                let t_prep = Instant::now();
                let input_i16: Vec<i16> = unsafe {
                    std::slice::from_raw_parts(
                        buf.as_ptr() as *const i16,
                        buf.len() / 2,
                    )
                }.to_vec();

                let tensor = match ::ort::value::Tensor::from_array(
                    (vec![1i64, 1, height as i64, width as i64], input_i16.into_boxed_slice())
                ) {
                    Ok(t) => t.upcast(),
                    Err(e) => {
                        warn!("OnnxEngine: failed to create input tensor: {}", e);
                        return Err(format!("Failed to create input tensor: {}", e));
                    }
                };

                // Run inference
                let input_name = "RawInputBlock/frame";
                let output_name = "DisplayBlock/frame";
                let t_run = Instant::now();
                let outputs = match session.run(ort::inputs![input_name => tensor]) {
                    Ok(o) => o,
                    Err(e) => {
                        warn!("OnnxEngine: inference failed: {}", e);
                        return Err(format!("ORT inference failed: {}", e));
                    }
                };
                let t_infer = t_run.elapsed();

                // Extract output tensor (UINT8 BGRA)
                if let Some(output_val) = outputs.get(output_name) {
                    match output_val.try_extract_tensor::<u8>() {
                        Ok((_, data)) => {
                            let output_data = data.to_vec();
                            let t_total = t0.elapsed();
                            debug!("OnnxEngine: prepare={:?} infer={:?} total={:?} ({} -> {} bytes)",
                                t_prep.elapsed(), t_infer, t_total,
                                buf.len(), output_data.len());
                            info!("OnnxEngine({}) frame done: {}x{} ({} bytes)",
                                self.backend.id(), target_width, height, output_data.len());
                            let mut frame = IspFrame::new(target_width, height, FrameFormat::Rgba8888);
                            frame.data = output_data;
                            return Ok(frame);
                        }
                        Err(e) => {
                            warn!("OnnxEngine: failed to extract output: {}", e);
                        }
                    }
                } else {
                    warn!("OnnxEngine: output '{}' not found", output_name);
                }
            }
        }
        
        #[cfg(not(feature = "ort"))]
        {
            debug!("OnnxEngine: ort feature not enabled, returning dummy frame");
        }
        
        let t_total = t0.elapsed();
        warn!("OnnxEngine({}) returning dummy frame (total={:?})", self.backend.id(), t_total);
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
