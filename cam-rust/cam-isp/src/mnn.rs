//! MNN inference engine for the ISP pipeline.
//! Ported from com.camcore.isp.mnn.MnnEngine

use log::{info};
use cam_types::{FrameFormat, ToneParams};

use crate::engine::IspEngine;
use crate::pipeline::{IspBlock, IspFrame, IspAuxOutput};

#[cfg(feature = "mnn")]
use log::warn;
#[cfg(feature = "mnn")]
use std::sync::Mutex;
#[cfg(feature = "mnn")]
use crate::mnn_sys::{MnnInterpreterSafe, MnnBackendType, MnnSessionSafe};

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
    session: Mutex<MnnSessionSafe>,
}

#[cfg(feature = "mnn")]
impl MnnSessionWrapper {
    fn new(interpreter: MnnInterpreterSafe, backend: MnnBackendType) -> Option<Self> {
        let session = interpreter.create_session(backend, 4)?;
        Some(Self {
            interpreter,
            session: Mutex::new(session),
        })
    }

    fn set_input_shape(&self, name: &str, shape: &[i32]) -> Result<(), String> {
        let session = self.session.lock().map_err(|_| "MNN session lock poisoned")?;
        let tensor = self.interpreter.get_input(&session, name)
            .ok_or_else(|| format!("Input tensor '{}' not found", name))?;
        tensor.set_shape(self.interpreter.as_ptr(), session.as_ptr(), shape)
    }

    fn copy_input(&self, name: &str, data: &[u8]) -> Result<(), String> {
        let session = self.session.lock().map_err(|_| "MNN session lock poisoned")?;
        let tensor = self.interpreter.get_input(&session, name)
            .ok_or_else(|| format!("Input tensor '{}' not found", name))?;
        let bytes = tensor.as_bytes_mut()
            .ok_or("Input tensor not mutable")?;
        if data.len() != bytes.len() {
            warn!("Input size mismatch: got {} expected {}", data.len(), bytes.len());
        }
        let copy_len = std::cmp::min(data.len(), bytes.len());
        bytes[..copy_len].copy_from_slice(&data[..copy_len]);
        Ok(())
    }

    fn read_output(&self, name: &str) -> Result<Vec<u8>, String> {
        let session = self.session.lock().map_err(|_| "MNN session lock poisoned")?;
        let tensor = self.interpreter.get_output(&session, name)
            .ok_or_else(|| format!("Output tensor '{}' not found", name))?;
        let size = tensor.data_size();
        let mut out = vec![0u8; size];
        unsafe {
            std::ptr::copy_nonoverlapping(tensor.as_ptr(), out.as_mut_ptr(), size);
        }
        Ok(out)
    }

    fn run(&self) -> Result<(), String> {
        let session = self.session.lock().map_err(|_| "MNN session lock poisoned")?;
        session.resize()?;
        session.run()
    }
}

/// ISP engine that uses Alibaba MNN for inference.
pub struct MnnEngine {
    backend: MnnBackend,
    initialized: bool,
    /// Composed ONNX model bytes (built by `build`).
    model_bytes: Option<Vec<u8>>,
    /// MNN session wrapper (feature-gated).
    #[cfg(feature = "mnn")]
    session: Option<Mutex<MnnSessionWrapper>>,
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
        #[allow(unused)] aux_blocks: Vec<Box<dyn IspBlock>>,
        #[allow(unused)] _warp_block: Option<Box<dyn IspBlock>>,
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

            self.session = Some(Mutex::new(session_wrapper));
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
        #[allow(unused)]
        buf: &[u8],
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
            let session_wrapper = self.session.as_ref()
                .ok_or_else(|| "MNN session not created".to_string())?;
            let wrapper = session_wrapper.lock()
                .map_err(|_| "MNN session lock poisoned".to_string())?;

            info!("MnnEngine({}) running inference {}x{} -> {}x{}",
                self.backend.id(), width, height, target_width, height);

            // Set input shape: [1, 1, height, width]
            let input_shape = &[1i32, 1i32, height as i32, width as i32];
            wrapper.set_input_shape("RawInputBlock/frame", input_shape)?;

            // Copy raw INT16 buffer into input tensor
            wrapper.copy_input("RawInputBlock/frame", buf)?;

            // Run inference
            wrapper.run()?;

            // Read output tensor
            let output_data = wrapper.read_output("DisplayBlock/frame")?;
            let output_size = output_data.len();

            info!("MnnEngine({}) inference done, bytes={}",
                self.backend.id(), output_size);

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
                calibration_stats: None,
                scene_category: None,
                af_phase: None,
                vcm_position: None,
                eis_compensation: None,
            });
            return Ok(frame);
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
            calibration_stats: None,
            scene_category: None,
            af_phase: None,
            vcm_position: None,
            eis_compensation: None,
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
