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
use log::info;

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
        match self { Self::Vulkan => 95, Self::Opencl => 85, Self::CpuNeon => 75, Self::Cpu => 65 }
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
        // Leak to avoid MNN internal drop-order crashes for now
        // TODO: fix proper release order
        if let Some(interp) = self.interp.take() {
            let _ = std::mem::ManuallyDrop::new(interp);
        }
        if let Some(sess) = self.sess.take() {
            let _ = std::mem::ManuallyDrop::new(sess);
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
                    info!("ONNX: {} bytes", onnx.len());

                    let on = format!(".mnn_temp_{}.onnx", std::process::id());
                    let mn = on.replace(".onnx", ".mnn");
                    std::fs::write(&on, &onnx).map_err(|e| format!("write: {}", e))?;
                    crate::mnn_converter::convert_onnx_to_mnn(&on, &mn, None).map_err(|e| format!("convert: {}", e))?;
                    let _ = std::fs::remove_file(&on);
                    mn
                }
            };

            if !Path::new(&mnn).exists() { return Err(format!("missing .mnn: {}", mnn)); }
            let interp = MnnInterpreterSafe::from_file(&mnn).ok_or_else(|| format!("load fail: {}", mnn))?;
            let sess = interp.create_session(self.backend.to_sys(), 4)
                .ok_or("session create fail")?;
            self.interp = Some(interp);
            self.sess = Some(Mutex::new(sess));
            info!("MNN engine loaded from {}", mnn);
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

            eprintln!("DBG: get input tensor (data)");
            let input = interp.get_input(&sess, "data")
                .or_else(|| interp.get_input(&sess, "RawInputBlock/frame"))
                .or_else(|| interp.get_input(&sess, "input"))
                .ok_or("no input tensor")?;

            eprintln!("DBG: set_shape & resize");
            input.set_shape(interp.as_ptr(), sess.as_ptr(), &[1, 1, h as i32, w as i32])?;
            sess.resize()?;

            // Copy normalized float32 input
            eprintln!("DBG: copy input");
            let f32d = Self::norm(buf, smax);
            let f32bytes: Vec<u8> = f32d.iter().flat_map(|f| f.to_ne_bytes()).collect();
            let host = input.as_mut_ptr();
            eprintln!("DBG: host ptr={:p}", host);
            if !host.is_null() {
                let dst = unsafe { std::slice::from_raw_parts_mut(host, f32bytes.len()) };
                dst.copy_from_slice(&f32bytes);
            } else {
                return Err("input host data not available".into());
            }

            // Run inference
            eprintln!("DBG: run");
            sess.run()?;
            eprintln!("DBG: run done");

            eprintln!("DBG: get output");
            let output = interp.get_first_output(&sess)
                .or_else(|| interp.get_output(&sess, "DisplayBlock/frame"))
                .ok_or("no output tensor")?;
            eprintln!("DBG: read output");
            let out_data = output.as_bytes().ok_or("output not readable")?.to_vec();
            eprintln!("DBG: output bytes={}", out_data.len());

            let nf = out_data.len() / 4;
            let oh = h as usize;
            let ow = tw as usize;
            let ch = if nf >= oh * ow * 4 { 4 } else if nf >= oh * ow * 3 { 3 } else { 1 };

            let floats: Vec<f32> = out_data.chunks_exact(4).map(|c| f32::from_ne_bytes([c[0], c[1], c[2], c[3]])).collect();
            let bgra = Self::to_bgra(&floats, ch, oh, ow);

            let mut frame = IspFrame::new(tw, h, FrameFormat::Rgba8888);
            frame.data = bgra;
            frame.aux = Some(IspAuxOutput { channel_means: Some([0.5, 0.5, 0.5]), tone_stats: None, wb_gains: None, histogram: None, zone_stats: None, focus_metric: None, cct: None, ae_gain: None, calibration_stats: None, scene_category: None, af_phase: None, vcm_position: None, eis_compensation: None });
            return Ok(frame);
        }

        let mut frame = IspFrame::new(tw, h, FrameFormat::Rgba8888);
        frame.data.fill(128);
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
                create_fn: || Box::new(cam_isp::mnnengine::MnnEngine::new($backend)),
            }
        );
    };
}

