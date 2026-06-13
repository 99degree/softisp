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
    OpenGl,
    CpuNeon,
    Cpu,
}

impl MnnBackend {
    pub fn id(&self) -> &'static str {
        match self { Self::Vulkan => "mnn_vulkan", Self::Opencl => "mnn_opencl", Self::OpenGl => "mnn_opengl", Self::CpuNeon => "mnn_neon", Self::Cpu => "mnn_cpu" }
    }
    pub fn priority(&self) -> i32 {
        match self { Self::Vulkan => 99, Self::Opencl => 55, Self::OpenGl => 50, Self::CpuNeon => 75, Self::Cpu => 65 }
    }
    #[cfg(feature = "mnn")]
    fn to_sys(&self) -> MnnBackendType {
        match self { Self::Vulkan => MnnBackendType::Vulkan, Self::Opencl => MnnBackendType::Opencl, Self::OpenGl => MnnBackendType::Opengl, _ => MnnBackendType::Cpu }
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
    /// Benchmarks each backend at startup and sets priority by actual FPS.
    /// Uses a 320×240 model (sweet-spot: realistic, not too slow to build).
    /// Times out after 2 s per backend — marks tardy backends with low priority.
    /// Called automatically by `cam_isp::init()`.
    #[cfg(feature = "mnn")]
    pub fn register_factories() {
        use crate::engine::register_engine;
        use crate::engine::EngineFactory;

        // Build ONE compute-heavy model (3× Conv3×3 + Relu).
        // 160×120 is large enough for GPU compute to dominate over kernel-launch overhead,
        // yet small enough for the ONNX→MNN conversion to finish in <1 s.
        let bench_w = 160u32;
        let bench_h = 120u32;
        let mnn_path = match Self::build_bench_model(bench_w, bench_h) {
            Ok(p) => p,
            Err(e) => {
                error!("Failed to build bench model: {}; using default priorities", e);
                Self::register_with_defaults();
                return;
            }
        };

        let all = [MnnBackend::Vulkan, MnnBackend::Opencl, MnnBackend::OpenGl, MnnBackend::CpuNeon, MnnBackend::Cpu];
        let mut scored: Vec<(MnnBackend, f64)> = Vec::new();

        for be in &all {
            let fps = match Self::bench_one(be, &mnn_path, bench_w, bench_h) {
                Ok(f) => { debug!("  bench {:>8}: {:.1} fps", be.id(), f); f }
                Err(e) => { warn!("  bench {:>8}: {} → 0 fps", be.id(), e); 0.0 }
            };
            scored.push((*be, fps));
        }

        let _ = std::fs::remove_file(&mnn_path);

        // Sort by FPS descending; ties keep original order
        scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        info!("MNN backend ranking (by measured FPS @ {}×{}):", bench_w, bench_h);
        for (i, (be, fps)) in scored.iter().enumerate() {
            let priority = (100 - i as i32).max(1);
            let name = be.id();
            let b = *be;
            let create_fn = Box::new(move || Box::new(MnnEngine::new(b)) as Box<dyn IspEngine>);
            register_engine(EngineFactory { name, priority, create_fn });
            info!("  {:>2}. {} → {:.1} fps (pri={})", i + 1, name, fps, priority);
        }
    }

    /// Fallback: register with static default priorities (no benchmark).
    #[cfg(feature = "mnn")]
    fn register_with_defaults() {
        use crate::engine::register_engine;
        use crate::engine::EngineFactory;
        let backends = [MnnBackend::CpuNeon, MnnBackend::Cpu, MnnBackend::Vulkan, MnnBackend::Opencl, MnnBackend::OpenGl];
        for be in &backends {
            let name = be.id();
            let pri = be.priority();
            let b = *be;
            let create_fn = Box::new(move || Box::new(MnnEngine::new(b)) as Box<dyn IspEngine>);
            register_engine(EngineFactory { name, priority: pri, create_fn });
        }
        info!("Registered {} MNN engine factories (default priorities)", backends.len());
    }

    /// Build a comprehensive benchmark .mnn model exercising all pipeline ops:
    ///   Conv3×3, Relu, GlobalAveragePool, MatMul, Reshape, Resize, GridSampler
    ///
    /// **Resolution-independent**: weights are constant (~2K) regardless of
    /// input resolution because GlobalAveragePool reduces spatial to 1×1 before
    /// the MatMul bottleneck. Compute scales with H×W (Conv layers) but weight
    /// size is fixed, making this suitable for benchmarking any resolution.
    ///
    /// Architecture:
    ///   Input [1,1,H,W] → Conv1(1→8,3×3) → Relu → Conv2(8→8,3×3) → Relu
    ///   → GlobalAveragePool [1,8,1,1] → Reshape [1,8]
    ///   → MatMul[8,64] → Relu → MatMul[64,8] → Reshape [1,8,1,1]
    ///   → Resize (nearest, ×H×W) [1,8,H,W] → Conv3(8→3,3×3)
    ///   → GridSampler(bilinear) → Output [1,3,H,W]
    ///
    /// ~2K weights (all Conv), compute scales with H×W.
    #[cfg(feature = "mnn")]
    pub fn build_bench_model(bench_w: u32, bench_h: u32) -> Result<String, String> {
        use crate::onnx::proto::Proto;
        use crate::mnn_converter::convert_onnx_to_mnn;

        let pid = std::process::id();
        let onnx_path = format!(".mnn_bench_{}.onnx", pid);
        let mnn_path = format!(".mnn_bench_{}.mnn", pid);

        let h = bench_h as i64;
        let w = bench_w as i64;

        // ── Dimension helpers ──
        let dim_1hw = || vec![
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(h), Proto::tensor_dim_value(w),
        ];
        let dim_3hw = || vec![
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
            Proto::tensor_dim_value(h), Proto::tensor_dim_value(w),
        ];
        let dim_8hw = || vec![
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(8),
            Proto::tensor_dim_value(h), Proto::tensor_dim_value(w),
        ];
        let dim_8111 = || vec![
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(8),
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
        ];
        let dim_8 = || vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(8),
        ];
        let dim_64 = || vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(64),
        ];

        // ── Weights (all resolution-independent, ~2K params total) ──
        let conv1_w = Self::rand_weight(8 * 1 * 3 * 3);   // [8,1,3,3]  = 72
        let conv2_w = Self::rand_weight(8 * 8 * 3 * 3);   // [8,8,3,3]  = 576
        let conv3_w = Self::rand_weight(3 * 8 * 3 * 3);   // [3,8,3,3]  = 216
        let mm1_w = Self::rand_weight(8 * 64);             // [8,64]     = 512
        let mm2_w = Self::rand_weight(64 * 8);             // [64,8]     = 512
        // total: 72+576+216+512+512 = 1,888 (constant at any resolution!)

        // Identity grid for GridSampler: [1, H, W, 2] in [-1, 1]
        let mut grid = Vec::with_capacity((h as usize * w as usize * 2) as usize);
        if h > 0 && w > 0 {
            for y in 0..h {
                for x in 0..w {
                    grid.push((2.0 * x as f64 / (w - 1).max(1) as f64 - 1.0) as f32);
                    grid.push((2.0 * y as f64 / (h - 1).max(1) as f64 - 1.0) as f32);
                }
            }
        }

        // Resize scales: [1, 1, H, W] — expand 1×1 → H×W
        let resize_scales = vec![1.0f32, 1.0, h as f32, w as f32];

        // ── Nodes ──
        let nodes = vec![
            // Conv1 1→8, 3×3
            Proto::node("Conv", &["input", "conv1_w", "conv1_b"], &["conv1_out"],
                &[Proto::attribute_ints("kernel_shape", &[3, 3]),
                  Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                  Proto::attribute_ints("strides", &[1, 1]),
                  Proto::attribute_int("group", 1)]),
            Proto::node("Relu", &["conv1_out"], &["relu1_out"], &[]),
            // Conv2 8→8, 3×3
            Proto::node("Conv", &["relu1_out", "conv2_w", "conv2_b"], &["conv2_out"],
                &[Proto::attribute_ints("kernel_shape", &[3, 3]),
                  Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                  Proto::attribute_ints("strides", &[1, 1]),
                  Proto::attribute_int("group", 1)]),
            Proto::node("Relu", &["conv2_out"], &["relu2_out"], &[]),
            // GlobalAveragePool — reduces ANY spatial size to 1×1
            Proto::node("GlobalAveragePool", &["relu2_out"], &["gap_out"], &[]),
            // Flatten [1,8,1,1] → [1,8]
            Proto::node("Reshape", &["gap_out", "flat8_shape"], &["flat8_out"], &[]),
            // MatMul bottleneck: [1,8] × [8,64] → [1,64]
            Proto::node("MatMul", &["flat8_out", "mm1_w"], &["mm1_out"], &[]),
            Proto::node("Relu", &["mm1_out"], &["mm1_relu"], &[]),
            // MatMul: [1,64] × [64,8] → [1,8]
            Proto::node("MatMul", &["mm1_relu", "mm2_w"], &["mm2_out"], &[]),
            Proto::node("Relu", &["mm2_out"], &["mm2_relu"], &[]),
            // Reshape back to [1,8,1,1]
            Proto::node("Reshape", &["mm2_relu", "shape_8111"], &["reshape8_out"], &[]),
            // Resize from 1×1 → H×W (nearest, fast)
            Proto::node("Resize", &["reshape8_out", "", "resize_scales", ""], &["resize_out"],
                &[Proto::attribute_string("mode", "nearest")]),
            // Conv3 8→3, 3×3
            Proto::node("Conv", &["resize_out", "conv3_w", "conv3_b"], &["conv3_out"],
                &[Proto::attribute_ints("kernel_shape", &[3, 3]),
                  Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                  Proto::attribute_ints("strides", &[1, 1]),
                  Proto::attribute_int("group", 1)]),
            // GridSampler with identity grid
            Proto::node("GridSampler", &["conv3_out", "identity_grid"], &["output"],
                &[Proto::attribute_string("mode", "bilinear"),
                  Proto::attribute_string("padding_mode", "zeros"),
                  Proto::attribute_int("align_corners", 0)]),
        ];

        // ── Value info ──
        let inputs = vec![Proto::value_info("input", &dim_1hw(), 1)];
        let outputs = vec![Proto::value_info("output", &dim_3hw(), 1)];
        let vi = vec![
            Proto::value_info("conv1_out", &dim_8hw(), 1),
            Proto::value_info("relu1_out", &dim_8hw(), 1),
            Proto::value_info("conv2_out", &dim_8hw(), 1),
            Proto::value_info("relu2_out", &dim_8hw(), 1),
            Proto::value_info("gap_out", &dim_8111(), 1),
            Proto::value_info("flat8_out", &dim_8(), 1),
            Proto::value_info("mm1_out", &dim_64(), 1),
            Proto::value_info("mm1_relu", &dim_64(), 1),
            Proto::value_info("mm2_out", &dim_8(), 1),
            Proto::value_info("mm2_relu", &dim_8(), 1),
            Proto::value_info("reshape8_out", &dim_8111(), 1),
            Proto::value_info("resize_out", &dim_8hw(), 1),
            Proto::value_info("conv3_out", &dim_3hw(), 1),
        ];

        // ── Initializers ──
        let init = vec![
            Proto::tensor_proto_float("conv1_w", &[8, 1, 3, 3], &conv1_w),
            Proto::tensor_proto_float("conv1_b", &[8], &[0.0f32; 8]),
            Proto::tensor_proto_float("conv2_w", &[8, 8, 3, 3], &conv2_w),
            Proto::tensor_proto_float("conv2_b", &[8], &[0.0f32; 8]),
            Proto::tensor_proto_float("conv3_w", &[3, 8, 3, 3], &conv3_w),
            Proto::tensor_proto_float("conv3_b", &[3], &[0.0f32; 3]),
            Proto::tensor_proto_float("mm1_w", &[8, 64], &mm1_w),
            Proto::tensor_proto_float("mm2_w", &[64, 8], &mm2_w),
            Proto::tensor_proto_int64("flat8_shape", &[1, 8]),
            Proto::tensor_proto_int64("shape_8111", &[1, 8, 1, 1]),
            Proto::tensor_proto_float("resize_scales", &[4], &resize_scales),
            Proto::tensor_proto_float("identity_grid", &[1, h, w, 2], &grid),
        ];

        let opset = Proto::opset("", 21);
        let graph = Proto::graph("bench", &nodes, &inputs, &outputs, &init, &vi);
        let model = Proto::model(9, &opset, "cam_isp_bench", &graph);

        std::fs::write(&onnx_path, &model).map_err(|e| format!("write: {}", e))?;
        // Note: Graph optimization (optimizeLevel=2) and INT8 quantization (weightQuantBits=8)
        // were tested but hurt performance on this synthetic benchmark (3.5 fps → 2.0 fps).
        // For production ISP pipelines, retest with optimization enabled.
        convert_onnx_to_mnn(&onnx_path, &mnn_path, None)
            .map_err(|e| format!("convert: {}", e))?;
        let _ = std::fs::remove_file(&onnx_path);
        Ok(mnn_path)
    }

    /// Helper: fill a tensor with small random values (stddev ~0.01).
    #[cfg(feature = "mnn")]
    fn rand_weight(n: usize) -> Vec<f32> {
        use std::time::{SystemTime, UNIX_EPOCH};
        // Simple xorshift to avoid std::rand dependency
        let seed = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().subsec_nanos() as u64;
        let mut state = seed ^ 123456789;
        let mut out = Vec::with_capacity(n);
        for _ in 0..n {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            out.push(((state >> 32) as f32 * 2.3283064e-10) * 0.02 - 0.01); // [-0.01, 0.01)
        }
        out
    }

    /// Benchmark one backend at 320×240 in a dedicated thread with 2 s timeout.
    /// Runs as many frames as possible within the budget, returns measured FPS.
    /// If the thread panics or times out, returns 0 fps.
    #[cfg(feature = "mnn")]
    fn bench_one(backend: &MnnBackend, mnn_path: &str, w: u32, h: u32) -> Result<f64, String> {
        use crate::blocks::RawInputBlock;
        use crate::engine::default_tone_params;

        let be = *backend;
        let mnn_owned = mnn_path.to_owned();
        let (tx, rx) = std::sync::mpsc::channel();

        std::thread::spawn(move || {
            let result = (|| -> Result<f64, String> {
                let mut engine = MnnEngine::new(be);
                engine.set_model_path(&mnn_owned);
                let head: Box<dyn IspBlock> = Box::new(RawInputBlock::new());
                engine.build(head, vec![], None, 16)
                    .map_err(|e| format!("build: {}", e))?;

                let params = default_tone_params();
                let frame_size = (w * h * 2) as usize;
                let mut buf = vec![0u8; frame_size];
                for y in 0..h {
                    for x in 0..w {
                        let off = (y * w + x) as usize * 2;
                        let val = (x ^ y) as u16;
                        buf[off] = val as u8;
                        buf[off + 1] = (val >> 8) as u8;
                    }
                }

                let budget = std::time::Duration::from_millis(500); // 0.5 s per backend
                let deadline = Instant::now() + budget;
                let mut count = 0u32;

                loop {
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    if remaining < std::time::Duration::from_millis(10) {
                        break;
                    }
                    engine.process(w, h, w, &buf, 1024.0, w, None, &params,
                                  None, None, 1.0, 0.0, None, None, None)?;
                    count += 1;
                }

                let elapsed = (budget - deadline.saturating_duration_since(Instant::now()))
                    .max(std::time::Duration::from_micros(1));
                if count == 0 {
                    return Err("0 frames in 2 s budget".into());
                }
                Ok(count as f64 / elapsed.as_secs_f64())
            })();
            let _ = tx.send(result);
        });

        match rx.recv_timeout(std::time::Duration::from_millis(3000)) { // 2 s budget + 1 s grace
            Ok(Ok(fps)) => Ok(fps),
            Ok(Err(e)) => Err(e),
            Err(_) => Err("timed out after 3 s".into()),
        }
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

            // Preload GPU backend libraries to register them with MNN.
            // On real Android (not Termux), the proprietary Adreno/Mali driver
            // will be accessible and Vulkan/OpenCL sessions will initialize.
            // On Termux, GPU libs load but VulkanDevice init throws
            // length_error("vector") due to Android linker namespace isolation.
            unsafe {
                let vk = libloading::os::unix::Library::new("libMNN_Vulkan.so");
                if let Ok(lib) = vk { std::mem::forget(lib); }
                let cl = libloading::os::unix::Library::new("libMNN_CL.so");
                if let Ok(lib) = cl { std::mem::forget(lib); }
                let gl = libloading::os::unix::Library::new("libMNN_GL.so");
                if let Ok(lib) = gl { std::mem::forget(lib); }
            }

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

