//! MNN inference engine for the ISP pipeline.
//! Ported from com.camcore.isp.mnn.MnnEngine

use log::{info, warn, error};
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

/// MNN interpreter wrapper for inference.
#[cfg(feature = "mnn")]
struct MnnSessionWrapper {
    interpreter: MnnInterpreterSafe,
    session: MnnSessionSafe,
    input_tensor: Option<crate::mnn_sys::MnnTensorSafe>,
    output_tensor: Option<crate::mnn_sys::MnnTensorSafe>,
}

#[cfg(feature = "mnn")]
impl MnnSessionWrapper {
    fn new(interpreter: MnnInterpreterSafe, backend: MnnBackendType) -> Option<Self> {
        let session = interpreter.create_session(backend, 4)?;
        Some(Self {
            interpreter,
            session,
            input_tensor: None,
            output_tensor: None,
        })
    }

    fn get_input_tensor(&mut self, name: &str) -> Option<crate::mnn_sys::MnnTensorSafe> {
        if self.input_tensor.is_none() {
            self.input_tensor = self.interpreter.get_input(&self.session, name);
        }
        self.input_tensor.clone()
    }

    fn get_output_tensor(&mut self, name: &str) -> Option<crate::mnn_sys::MnnTensorSafe> {
        if self.output_tensor.is_none() {
            self.output_tensor = self.interpreter.get_output(&self.session, name);
        }
        self.output_tensor.clone()
    }

    fn run(&mut self) -> Result<(), String> {
        self.session.resize()?;
        self.session.run()
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
    session: Option<MnnSessionWrapper>,
}

impl MnnEngine {
    pub fn new(backend: MnnBackend) -> Self {
        Self {
            backend,
            initialized: false,
            model_bytes: None,
            #[cfg(feature = "mnn")]
            session: None,
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
        warpbock: Option<Box<dyn IspBlock>>,
        opset_version: i64,
    ) -> Result<(), String> {
        info!("Building MNN engine with backend={}", self.backend.id());

        // Compose ONNX model from pipeline blocks
        let aux_refs: Vec<&dyn IspBlock> = aux_blocks.iter().map(|b| b.as_ref() as &dyn IspBlock).collect();
        
        let model = crate::pipeline::GraphComposer::compose(
            pipeline_head.as_ref(),
            &aux_refs,
            opset_version,
        )?;
        
        info!("MnnEngine({}) composed ONNX model ({} bytes)", self.backend.id(), model.len());

        #[cfg(feature = "mnn")]
        {
            // Create MNN interpreter from ONNX model
            let interpreter = MnnInterpreterSafe::from_buffer(&model)
                .ok_or_else(|| "Failed to create MNN interpreter from ONNX model")?;
            
            let backend_type = self.backend.to_sys_backend();
            let session_wrapper = MnnSessionWrapper::new(interpreter, backend_type)
                .ok_or_else(|| "Failed to create MNN session")?;

            self.session = Some(session_wrapper);
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
    ) -> Result<IspFrame, String> {
        if !self.initialized {
            return Err("Engine not initialized".to_string());
        }

        #[cfg(feature = "mnn")]
        {
            // Use interior mutability - session needs mut for run()
            use std::sync::atomic::{AtomicPtr, Ordering};
            use std::ptr;
            // We'll use unsafe to get mutable ref from Option
            let session_ptr = &self.session as *const Option<MnnSessionWrapper>;
            let session_wrapper = unsafe { &mut (*session_ptr).as_mut() };
            if let Some(ref mut sw) = session_wrapper {
                info!("MnnEngine({}) running inference {}x{} -> {}x{}",
                    self.backend.id(), width, height, target_width, height);

                // Get input tensor (RawInputBlock/frame) - expects INT16
                let input_tensor = sw.get_input_tensor("RawInputBlock/frame")
                    .ok_or_else(|| "Input tensor 'RawInputBlock/frame' not found".to_string())?;

                // Copy raw INT16 buffer into input tensor
                let input_data = buf;
                let input_ptr = input_tensor.as_mut_ptr();
                let input_size = input_tensor.data_size();
                if input_data.len() != input_size {
                    warn!("Input size mismatch: got {} expected {}", input_data.len(), input_size);
                }
                let copy_len = std::cmp::min(input_data.len(), input_size);
                unsafe { ptr::copy_nonoverlapping(input_data.as_ptr(), input_ptr as *mut u8, copy_len); }

                // Run inference
                sw.run()?;

                // Get output tensor (DisplayBlock/frame) - UINT8 BGRA
                let output_tensor = sw.get_output_tensor("DisplayBlock/frame")
                    .ok_or_else(|| "Output tensor 'DisplayBlock/frame' not found".to_string())?;

                let output_shape = output_tensor.shape();
                let output_size = output_tensor.data_size();
                let mut output_data = vec![0u8; output_size];
                unsafe {
                    ptr::copy_nonoverlapping(output_tensor.as_ptr(), output_data.as_mut_ptr() as *const u8, output_size);
                }

                info!("MnnEngine({}) inference done, output shape={:?} bytes={}",
                    self.backend.id(), output_shape, output_size);

                // Construct IspFrame
                let mut frame = IspFrame::new(target_width, height, FrameFormat::Rgba8888);
                frame.data = output_data;
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
                return Ok(frame);
            } else {
                error!("MNN session not created");
            }
        }

        #[cfg(not(feature = "mnn"))]
        {
            info!("MnnEngine({}) mnn feature not enabled, returning dummy frame {}x{} -> {}x{}",
                self.backend.id(), width, height, target_width, height);
        }
        
        let mut frame = IspFrame::new(target_width, height, FrameFormat::Rgba8888);
        // Fill with dummy data (gray)
        frame.data.fill(128);
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
