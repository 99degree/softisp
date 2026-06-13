//! MNN inference engine — ONNX → MNN convert then run inference.
//!
//! Build flow:
//!   1. Compose ONNX model from pipeline blocks
//!   2. Convert ONNX → .mnn via MNNConvert CLI (or load pre-converted)
//!   3. Load .mnn with MNN runtime
//!
//! Inference flow:
//!   1. Set input shape [1,1,H,W]
//!   2. Copy INT16 sensor data as normalized float32
//!   3. Run session
//!   4. Read output float32 → BGRA U8

use std::sync::Mutex;
use std::time::Instant;
use log::{info, debug, warn, error};

use cam_types::{FrameFormat, ToneParams};
use crate::engine::IspEngine;
use crate::pipeline::{IspBlock, IspFrame, IspAuxOutput};

#[cfg(feature = "mnn")]
pub use crate::mnn_sys::*;

// ── Backend ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MnnBackend {
    Vulkan,
    Opencl,
    CpuNeon,
    Cpu,
}

impl MnnBackend {
    pub fn id(&self) -> &'static str {
        match self { Self::Vulkan => "mnn_vulkan", Self::Opencl => "mnn_opencl", Self::CpuNeon => "mnn_neon", Self::Cpu => "mnn_cpu" }
    }
    pub fn priority(&self) -> i32 {
        match self { Self::Vulkan => 99, Self::Opencl => 93, Self::CpuNeon => 75, Self::Cpu => 65 }
    }
    #[cfg(feature = "mnn")]
    fn to_sys(&self) -> MnnBackendType {
        match self { Self::Vulkan => MnnBackendType::Vulkan, Self::Opencl => MnnBackendType::Opencl, _ => MnnBackendType::Cpu }
    }
}

// ── Engine ──────────────────────────────────────────────────────────────

pub struct MnnEngine {
    backend: MnnBackend,
    initialized: bool,
    model_path: Option<String>,
    /// MNN interpreter (owning the model). Must outlive session.
    #[cfg(feature = "mnn")]
    interp: Option<MnnInterpreterSafe>,
    /// MNN session (references interpreter). Dropped before interp.
    #[cfg(feature = "mnn")]
    sess: Option<Mutex<MnnSessionSafe>>,
}

#[cfg(feature = "mnn")]
impl Drop for MnnEngine {
    fn drop(&mut self) {
        // Drop session before interpreter to avoid dangling pointer.
        // Session holds a raw interpreter pointer used in releaseSession().
        if let Some(sess) = self.sess.take() {
            drop(sess);
        }
        if let Some(interp) = self.interp.take() {
            drop(interp);
        }
    }
}

impl MnnEngine {
    pub fn new(backend: MnnBackend) -> Self {
        Self {
            backend,
            initialized: false,
            model_path: None,
            #[cfg(feature = "mnn")]
            interp: None,
            #[cfg(feature = "mnn")]
            sess: None,
        }
    }

    /// Point to pre-converted .mnn file (skips on-the-fly conversion).
    pub fn set_model_path(&mut self, path: impl Into<String>) { self.model_path = Some(path.into()); }

    /// Register MNN engine factories for all available backends.
    /// Called automatically by `cam_isp::init()`.
    #[cfg(feature = "mnn")]
    pub fn register_factories() {
        use crate::engine::register_engine;
        use crate::engine::EngineFactory;
        let backends = [MnnBackend::CpuNeon, MnnBackend::Cpu, MnnBackend::Vulkan, MnnBackend::Opencl];
        for be in &backends {
            let name = be.id();
            let pri = be.priority();
            let b = *be;
            let create_fn = Box::new(move || Box::new(MnnEngine::new(b)) as Box<dyn IspEngine>);
            register_engine(EngineFactory { name, priority: pri, create_fn });
        }
        info!("Registered {} MNN engine factories", backends.len());
    }

    // ── helpers ──

    fn norm(buf: &[u8], max: f32) -> Vec<f32> {
        buf.chunks_exact(2).map(|c| (u16::from_ne_bytes([c[0], c[1]]) as f32 / max).clamp(0.0, 1.0)).collect()
    }

    fn to_bgra(data: &[f32], ch: usize, h: usize, w: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(h * w * 4);
        for y in 0..h {
            for x in 0..w {
                let i = (y * w + x) * ch;
                let r = Self::c(data.get(i).copied().unwrap_or(0.0));
                let g = Self::c(data.get(i + 1).copied().unwrap_or(0.0));
                let b = Self::c(data.get(i + 2).copied().unwrap_or(0.0));
                let a = if ch >= 4 { Self::c(data.get(i + 3).copied().unwrap_or(1.0)) } else { 255u8 };
                out.extend_from_slice(&[b, g, r, a]);
            }
        }
        out
    }
    fn c(v: f32) -> u8 { (v * 255.0).round().clamp(0.0, 255.0) as u8 }
}

impl IspEngine for MnnEngine {
    fn backend_name(&self) -> &'static str { self.backend.id() }
    fn priority(&self) -> i32 { self.backend.priority() }
    fn is_loaded(&self) -> bool { self.initialized }

    fn build(&mut self, head: Box<dyn IspBlock>, aux: Vec<Box<dyn IspBlock>>, _warp: Option<Box<dyn IspBlock>>, opset: i64) -> Result<(), String> {
        info!("MNN build backend={}", self.backend.id());

        #[cfg(feature = "mnn")]
        {
            use std::path::Path;

            let mnn = match &self.model_path {
                Some(p) => p.clone(),
                None => {
                    let mut all: Vec<Box<dyn IspBlock>> = vec![head];
                    all.extend(aux);
                    let refs: Vec<&dyn IspBlock> = all.iter().map(|b| b.as_ref()).collect();
                    let onnx = crate::pipeline::GraphComposer::compose_from_vec(&refs, &[], opset)?;
                    debug!("ONNX: {} bytes for MNN conversion", onnx.len());

                    let on = format!(".mnn_temp_{}.onnx", std::process::id());
                    let mn = on.replace(".onnx", ".mnn");
                    std::fs::write(&on, &onnx).map_err(|e| format!("write: {}", e))?;
                    crate::mnn_converter::convert_onnx_to_mnn(&on, &mn, None).map_err(|e| format!("convert: {}", e))?;
                    let _ = std::fs::remove_file(&on);
                    mn
                }
            };

            if !Path::new(&mnn).exists() {
                error!("MNN model not found: {}", mnn);
                return Err(format!("missing .mnn: {}", mnn));
            }
            let interp = MnnInterpreterSafe::from_file(&mnn).ok_or_else(|| {
                error!("Failed to load MNN model: {}", mnn);
                format!("load fail: {}", mnn)
            })?;
            let sess = interp.create_session(self.backend.to_sys(), 4)
                .ok_or("session create fail")?;
            self.interp = Some(interp);
            self.sess = Some(Mutex::new(sess));
            info!("MNN engine loaded from {} (backend={:?})", mnn, self.backend);
        }

        #[cfg(not(feature = "mnn"))] { let _ = (head, aux, opset); }
        self.initialized = true;
        Ok(())
    }

    fn process(&self, w: u32, h: u32, _sw: u32, buf: &[u8], smax: f32, tw: u32,
               _ccm: Option<&[f32; 9]>, _tone: &ToneParams,
               _bayer: Option<&[f32; 4]>, _awb: Option<&[f32; 3]>,
               _ag: f32, _sc: f32, _lsc: Option<&[f32]>, _blc: Option<&[f32; 4]>, _wg: Option<&[f32]>) -> Result<IspFrame, String> {
        if !self.initialized { return Err("not init".into()); }

        #[cfg(feature = "mnn")]
        {
            let sess_lock = self.sess.as_ref().ok_or("no session")?;
            let sess = sess_lock.lock().map_err(|_| "lock")?;
            let interp = self.interp.as_ref().unwrap();

            let t_start = Instant::now();

            // Normalize INT16/UINT16 sensor data to float32
            let f32d = Self::norm(buf, smax);
            let t_norm = t_start.elapsed();

            let shape = [1, 1, h as i32, w as i32];

            // Allocate output buffer (enough for 4K HD RGBA floats)
            let max_out = (tw * h * 4) as i32;
            let mut out_data = vec![0.0f32; max_out as usize];

            // Use mnn_run_host_tensors which handles:
            // 1. Creating proper host tensors (with backend, not portal tensors)
            // 2. Converting float32 input to model's native type (INT16/UINT16/F32)
            // 3. resizeSession + copyFromHostTensor
            // 4. runSession
            // 5. copyToHostTensor output back to float32
            //
            // This bypasses the portal tensor issue where copyFromHostTensor
            // fails because the portal tensor has backend=NULL.
            let n = unsafe {
                crate::mnn_sys::mnn_run_host_tensors(
                    interp.as_ptr(),
                    sess.as_ptr(),
                    f32d.as_ptr(),
                    shape.as_ptr(),
                    shape.len() as i32,
                    out_data.as_mut_ptr(),
                    max_out,
                )
            };

            let t_infer = t_start.elapsed();

            if n <= 0 {
                error!("mnn_run_host_tensors failed: {} (input={}x{})", n, w, h);
                return Err(format!("mnn_run_host_tensors failed: {}", n));
            }

            debug!("MNN inference: norm={:?} infer={:?} total={:?} ({}x{} -> {} flt)",
                t_norm, t_infer - t_norm, t_infer, w, h, n);

            let nf = n as usize;
            let oh = h as usize;
            let ow = tw as usize;
            let ch = nf / (oh * ow);

            let bgra = Self::to_bgra(&out_data[..nf], ch, oh, ow);

            let mut frame = IspFrame::new(tw, h, FrameFormat::Rgba8888);
            frame.data = bgra;
            frame.aux = Some(IspAuxOutput {
                channel_means: Some([0.5, 0.5, 0.5]),
                tone_stats: None, wb_gains: None, histogram: None,
                zone_stats: None, focus_metric: None, cct: None,
                ae_gain: None, calibration_stats: None,
                scene_category: None, af_phase: None,
                vcm_position: None, eis_compensation: None,
            });
            return Ok(frame);
        }

        #[cfg(not(feature = "mnn"))]
        {
            let mut frame = IspFrame::new(tw, h, FrameFormat::Rgba8888);
            frame.data.fill(128);
            Ok(frame)
        }
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
                create_fn: || Box::new(cam_isp::mnnengine::MnnEngine::new($backend)),
            }
        );
    };
}

