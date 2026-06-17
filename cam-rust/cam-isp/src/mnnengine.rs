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
use std::os::raw::c_void;
use std::time::Instant;
use log::{info, debug, warn, error};

use cam_types::{FrameFormat, ToneParams};
use crate::engine::{IspEngine, ProcessParams};
use crate::controller::IspController;
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
    model_input_type: Option<(i32, i32)>,
    /// True when model expects INT32 packed input (w/2 width, each element = 2 pixels).
    /// Triggers true zero-copy: buffer reinterpreted as i32, host pointer set directly.
    packed_input: bool,
    /// Expected input tensor element count from model (for validation).
    expected_input_elements: Option<u32>,
    /// ISP controller for AWB/AE/CCM/tone parameter estimation.
    /// After each inference, stats output tensors are read and fed
    /// into the controller to update state for the next frame.
    pub controller: Mutex<IspController>,
    /// MNN interpreter (owning the model). Must outlive session.
    #[cfg(feature = "mnn")]
    interp: Option<MnnInterpreterSafe>,
    /// MNN session (references interpreter). Dropped before interp.
    #[cfg(feature = "mnn")]
    sess: Option<Mutex<MnnSessionSafe>>,
    /// Pipeline-aligned output buffer pool.
    /// Pre-allocated at build(), recycled every frame via acquire/release.
    #[cfg(feature = "mnn")]
    buf_pool: Mutex<crate::mnn_buffer::OutputBufferPool>,
    /// Cached tensor handles for extra inputs — avoids CString alloc per frame.
    #[cfg(feature = "mnn")]
    tensor_pool: Vec<(String, crate::mnn_sys::MnnTensorSafe)>,
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
    /// Get the model input type: (code, bits) or None if not yet built.
    pub fn model_input_type(&self) -> Option<(i32, i32)> {
        self.model_input_type
    }
    /// Get expected input tensor element count from the loaded model.
    pub fn expected_input_elements(&self) -> Option<u32> {
        self.expected_input_elements
    }

    pub fn new(backend: MnnBackend) -> Self {
        Self {
            backend,
            initialized: false,
            model_path: None,
            model_input_type: None,
            packed_input: false,
            expected_input_elements: None,
            controller: Mutex::new(IspController::new()),
            #[cfg(feature = "mnn")]
            buf_pool: Mutex::new(crate::mnn_buffer::OutputBufferPool::new(3, 1)),
            #[cfg(feature = "mnn")]
            #[cfg(feature = "mnn")]
            interp: None,
            #[cfg(feature = "mnn")]
            sess: None,
            #[cfg(feature = "mnn")]
            tensor_pool: Vec::new(),
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
        let inputs = vec![Proto::value_info("input", &dim_1hw(), 5)];  // 5 = INT16
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
                    engine.process(&crate::engine::ProcessParams::new(w, h, &buf))?;
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
        if ch >= 4 {
            // bg4a path: model outputs [1,4,H,W] FLOAT [0,255] BGRA.
            // Values are exact integers (Conv(1×1) + bias + round = whole numbers).
            // Just truncate f32→u8, no multiply or clamp needed.
            for i in 0..h * w {
                let base = i * 4;
                out.extend_from_slice(&[
                    data[base] as u8,       // B
                    data[base + 1] as u8,   // G
                    data[base + 2] as u8,   // R
                    255,                    // A
                ]);
            }
        } else {
            // Standard float [0,1] RGB path: multiply by 255, round, swap
            for i in 0..h * w {
                let base = i * ch;
                let r = Self::c(data.get(base).copied().unwrap_or(0.0));
                let g = Self::c(data.get(base + 1).copied().unwrap_or(0.0));
                let b = Self::c(data.get(base + 2).copied().unwrap_or(0.0));
                let a = if ch >= 4 { Self::c(data.get(base + 3).copied().unwrap_or(1.0)) } else { 255u8 };
                out.extend_from_slice(&[b, g, r, a]);
            }
        }
        out
    }
    /// Convert MNN output buffer to BGRA Vec<u8>.
    /// Handles:
    ///   ch=1: packed INT32 (R*65536+G*256+B, must reinterpret &[f32] as &[i32])
    ///   ch=3: float [0,1] RGB
    ///   ch=4: float [0,255] BGRA (bg4a)
    fn convert_output(data: &[f32], ch: usize, h: usize, w: usize) -> Vec<u8> {
        let n_pixels = h * w;
        let mut out = Vec::with_capacity(n_pixels * 4);
        if ch == 1 {
            // Packed INT32 path: each i32 = R*65536 + G*256 + B
            let ints: &[i32] = unsafe {
                std::slice::from_raw_parts(data.as_ptr() as *const i32, data.len())
            };
            for i in 0..n_pixels {
                let packed = ints.get(i).copied().unwrap_or(0);
                let r = ((packed >> 16) & 0xFF) as u8;
                let g = ((packed >> 8) & 0xFF) as u8;
                let b = (packed & 0xFF) as u8;
                out.extend_from_slice(&[b, g, r, 255]);
            }
        } else if ch >= 4 {
            // bg4a path: 4-channel float [0,255] BGRA from Conv(1×1)
            for i in 0..n_pixels {
                let base = i * 4;
                out.extend_from_slice(&[
                    data[base] as u8,
                    data[base + 1] as u8,
                    data[base + 2] as u8,
                    255,
                ]);
            }
        } else {
            Self::to_bgra(data, ch, h, w)
        }
        out
    }

    fn c(v: f32) -> u8 { (v * 255.0).round().clamp(0.0, 255.0) as u8 }

    /// Write extra input tensors (CCM, tone, gains) into the MNN session
    /// BEFORE inference.  Must be called AFTER resizeSession so tensor
    /// host buffers are valid for writing.
    #[cfg(feature = "mnn")]
    /// Set runtime-overridable extra inputs for the MNN session.
    ///
    /// Called AFTER `resizeSession()` but BEFORE inference.
    /// Writes controller-computed params (CCM, tone, WB gains) directly
    /// into MNN host tensors for zero-copy override of initializer values.
    ///
    /// # Bayer pattern codes
    /// - 0 = RGGB (default)
    /// - 1 = BGGR
    /// - 2 = GRBG
    /// - 3 = GBRG
    fn set_extra_inputs(
        pool: &[(String, crate::mnn_sys::MnnTensorSafe)],
        ccm: Option<&[f32; 9]>,
        tone: &ToneParams,
        bayer: Option<&[f32; 4]>,
        _awb: Option<&[f32; 3]>,
        bayer_pattern: i32,
    ) {
        /// Look up a cached tensor handle by name prefix (fast — no CString alloc).
        fn find<'a>(pool: &'a [(String, crate::mnn_sys::MnnTensorSafe)], name: &str) -> Option<&'a crate::mnn_sys::MnnTensorSafe> {
            pool.iter().find(|(n, _)| n == name).map(|(_, t)| t)
        }

        // DemosaicCcmBlock/w [3,4,1,1] — fused CCM × demosaic weights
        //
        // The demosaic operation extracts 2×2 Bayer quadrants into 3 RGB
        // output channels.  The weights depend on the sensor's Bayer pattern:
        //
        //   Pattern 0 (RGGB):  R=[1,0,0,0]  G=[0,½,½,0]  B=[0,0,0,1]
        //   Pattern 1 (BGGR):  R=[0,0,0,1]  G=[0,½,½,0]  B=[1,0,0,0]
        //   Pattern 2 (GRBG):  R=[0,1,0,0]  G=[½,0,0,½]  B=[0,0,1,0]
        //   Pattern 3 (GBRG):  R=[0,0,1,0]  G=[½,0,0,½]  B=[0,1,0,0]
        //
        // fused[i,j] = Σₖ ccm[i,k] * demo[k,j]   (i=RGB, j=quadrant)
        const DEMO_RGGB: [f32; 12] = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.5, 0.5, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        const DEMO_BGGR: [f32; 12] = [
            0.0, 0.0, 0.0, 1.0,
            0.0, 0.5, 0.5, 0.0,
            1.0, 0.0, 0.0, 0.0,
        ];
        const DEMO_GRBG: [f32; 12] = [
            0.0, 1.0, 0.0, 0.0,
            0.5, 0.0, 0.0, 0.5,
            0.0, 0.0, 1.0, 0.0,
        ];
        const DEMO_GBRG: [f32; 12] = [
            0.0, 0.0, 1.0, 0.0,
            0.5, 0.0, 0.0, 0.5,
            0.0, 1.0, 0.0, 0.0,
        ];

        // Select demosaic weights by pattern
        let demo: &[f32; 12] = match bayer_pattern & 3 {
            1 => &DEMO_BGGR,
            2 => &DEMO_GRBG,
            3 => &DEMO_GBRG,
            _ => &DEMO_RGGB,
        };

        // Fused CCM weights: w[i,j] = Σₖ ccm[i,k] * demo[k,j]
        if let Some(ccm) = ccm {
            let mut fused = [0.0f32; 12];
            for i in 0..3 {
                for j in 0..4 {
                    let mut s = 0.0;
                    for k in 0..3 {
                        s += ccm[i * 3 + k] * demo[k * 4 + j];
                    }
                    // When tone is fused in, absorb contrast into weights
                    fused[i * 4 + j] = s * tone.contrast;
                }
            }
            if let Some(t) = find(pool, "DemosaicCcmBlock/w") {
                if let Some(mut bytes) = t.as_bytes_mut() {
                    let src = unsafe {
                        std::slice::from_raw_parts(fused.as_ptr() as *const u8, 48)
                    };
                    if bytes.len() >= 48 { bytes[..48].copy_from_slice(src); }
                }
            }
        }
        // DemosaicCcmBlock/b [3] — CCM bias + tone brightness
        if let Some(t) = find(pool, "DemosaicCcmBlock/b") {
            if let Some(mut bytes) = t.as_bytes_mut() {
                // When tone is fused in, absorb brightness into bias
                let bias = [tone.brightness; 3];
                let src = unsafe { std::slice::from_raw_parts(bias.as_ptr() as *const u8, 12) };
                if bytes.len() >= 12 { bytes[..12].copy_from_slice(src); }
            }
        }
        // BayerWbBlock/gains [1,4,1,1] — white balance per-channel gains
        if let Some(gains) = bayer {
            if let Some(t) = find(pool, "BayerWbBlock/gains") {
                if let Some(mut bytes) = t.as_bytes_mut() {
                    let src = unsafe { std::slice::from_raw_parts(gains.as_ptr() as *const u8, 16) };
                    if bytes.len() >= 16 { bytes[..16].copy_from_slice(src); }
                }
            }
        }
        // ToneBlock/contrast [1]
        if let Some(t) = find(pool, "ToneBlock/contrast") {
            if let Some(mut bytes) = t.as_bytes_mut() {
                let src = unsafe { std::slice::from_raw_parts((&tone.contrast as *const f32) as *const u8, 4) };
                if bytes.len() >= 4 { bytes[..4].copy_from_slice(src); }
            }
        }
        // ToneBlock/brightness [1]
        if let Some(t) = find(pool, "ToneBlock/brightness") {
            if let Some(mut bytes) = t.as_bytes_mut() {
                let src = unsafe { std::slice::from_raw_parts((&tone.brightness as *const f32) as *const u8, 4) };
                if bytes.len() >= 4 { bytes[..4].copy_from_slice(src); }
            }
        }
        // ToneBlock/gamma_recip [1]
        if let Some(t) = find(pool, "ToneBlock/gamma_recip") {
            if let Some(mut bytes) = t.as_bytes_mut() {
                let src = unsafe { std::slice::from_raw_parts((&tone.gamma_recip as *const f32) as *const u8, 4) };
                if bytes.len() >= 4 { bytes[..4].copy_from_slice(src); }
            }
        }
    }
}

impl IspEngine for MnnEngine {
    fn backend_name(&self) -> &'static str { self.backend.id() }
    fn priority(&self) -> i32 { self.backend.priority() }
    fn is_loaded(&self) -> bool { self.initialized }
    fn controller(&self) -> &Mutex<IspController> { &self.controller }

    fn build(&mut self, head: Box<dyn IspBlock>, aux: Vec<Box<dyn IspBlock>>, _warp: Option<Box<dyn IspBlock>>, opset: i64) -> Result<(), String> {
        info!("MNN build backend={}", self.backend.id());

        #[cfg(feature = "mnn")]
        {
            use std::path::Path;

            let mnn = match &self.model_path {
                Some(p) => p.clone(),
                None => {
                    // Build ONNX graph: head + aux as pipeline, stats as aux_blocks
                    // But we don't know which aux are stats — pass all as pipeline for now
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
            
            // Query model input type for zero-copy optimization
            let mut input_code = 0i32;
            let mut input_bits = 0i32;
            unsafe {
                crate::mnn_sys::mnn_get_model_input_type(
                    interp.as_ptr(),
                    sess.as_ptr(),
                    &mut input_code,
                    &mut input_bits,
                );
            }
            self.model_input_type = Some((input_code, input_bits));
            // INT32 (code=0, bits=32) → packed_input mode for true zero-copy.
            // Model expects w/2 INT32 elements; buffer is reinterpreted u16 as i32.
            let expected_elems = unsafe {
                crate::mnn_sys::mnn_get_model_input_elements(interp.as_ptr(), sess.as_ptr())
            };
            self.expected_input_elements = if expected_elems > 0 { Some(expected_elems as u32) } else { None };
            // Packed if INT32 (code=0, bits=32) — all pipeline models use packed input
            self.packed_input = input_code == 0 && input_bits == 32;
            info!("MNN model input type: code={}, bits={} packed={} expected_elems={}",
                input_code, input_bits, self.packed_input, expected_elems);

            // Populate tensor pool: cache all extra input handles to avoid
            // per-frame CString allocations in set_extra_inputs().
            let extra_names = [
                "DemosaicCcmBlock/w",
                "DemosaicCcmBlock/b",
                "BayerWbBlock/gains",
                "ToneBlock/contrast",
                "ToneBlock/brightness",
                "ToneBlock/gamma_recip",
            ];
            for name in &extra_names {
                if let Some(t) = interp.get_input(&sess, name) {
                    self.tensor_pool.push((name.to_string(), t));
                }
            }
            debug!("tensor_pool: {} cached handles", self.tensor_pool.len());

            self.interp = Some(interp);
            self.sess = Some(Mutex::new(sess));
            info!("MNN engine loaded from {} (backend={:?})", mnn, self.backend);
        }

        #[cfg(not(feature = "mnn"))] { let _ = (head, aux, opset); }
        self.initialized = true;
        Ok(())
    }

    fn process(&self, p: &ProcessParams) -> Result<IspFrame, String> {
        let w = p.width;
        let h = p.height;
        let buf = p.buf;
        let smax = p.sensor_max;
        let tw = p.target_width;
        let ccm = p.ccm_matrix.as_ref();
        let tone = &p.tone_params;
        let bayer = p.bayer_gains.as_ref();
        let awb = p.awb_gains.as_ref();
        let bayer_pattern = p.bayer_pattern;

        if !self.initialized { return Err("not init".into()); }

        #[cfg(feature = "mnn")]
        {
            let sess_lock = self.sess.as_ref().ok_or("no session")?;
            let sess = sess_lock.lock().map_err(|_| "lock")?;
            let interp = self.interp.as_ref().unwrap();

            let t_start = Instant::now();

            info!("pipeline stage=arrive raw={}×{} bayer={}B sensor_max={}",
                w, h, buf.len(), smax);

            // Acquire output buffer from pool.
            // Lifecycle: Free → acquire() (Writing) → release() (Free).
            // Pre-allocated at first frame to avoid 66MB per-frame zero-fill.
            let max_out = (tw * h * 4) as i32;
            let mut pool = self.buf_pool.lock().unwrap();
            // Resize pool if first call or resolution changed
            if pool.len() == 0 || pool.buffer_capacity() < max_out as usize {
                *pool = crate::mnn_buffer::OutputBufferPool::new(3, max_out as usize);
            }
            let (buf_id, _buf_slice) = pool.acquire();
            let out_data: &mut [f32] = _buf_slice;

            info!("pipeline stage=acquire buf_id={}", buf_id);

            // Determine inference path based on model input type
            //   1) packed_input (INT32): true zero-copy — reinterpret u16 as i32, host ptr
            //   2) true_zero_copy (INT16/UINT16): direct u16 buffer, blocked by MNN upcast
            //   3) u16_conversion: copy u16→int32 via host tensor
            //   4) float_fallback: normalize to f32 then host tensor
            let path: &str;
            let mut n: i32;
            let t_prep_end: std::time::Instant;
            let t_infer_start: std::time::Instant;

            // Determine if packed: model expects INT32 packed input
            // When expected_elems is Some, verify element count matches h*w/2.
            // When None (unknown height), trust the packed_input flag.
            let is_packed = if self.packed_input {
                if let Some(expected) = self.expected_input_elements {
                    expected == h * w / 2
                } else {
                    // Unknown height — model is still packed, trust the flag
                    true
                }
            } else {
                false
            };

            // ── Pre-inference: resize session and set extra inputs ──
            // Resize BEFORE the convenience wrappers so that:
            //   1. resizeSession allocates all internal buffers
            //   2. Extra inputs (CCM, tone, gains) are written to valid host buffers
            //   3. Convenience wrapper won't resize again (shapes already match)
            {
                let shape = if is_packed {
                    vec![1, 1, h as i32, (w / 2).max(1) as i32]
                } else {
                    vec![1, 1, h as i32, w as i32]
                };
                if let Some(t) = interp.get_first_input(&sess) {
                    let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &shape);
                }
                let _ = sess.resize();
                // Extra inputs: write AFTER resize so host buffers are valid
                Self::set_extra_inputs(&self.tensor_pool, ccm, tone, bayer, awb, bayer_pattern);
            }

            debug!("MNN process: w={}, h={}, packed_w={}, is_packed={}, expected={:?}",
                w, h, (w / 2).max(1), is_packed, self.expected_input_elements);

            // ── PACKED ZERO-COPY (ONLY PATH) ──
            // Model expects INT32[1,1,h,w/2]. Our u16 bayer buffer is reinterpreted
            // as i32[h*w/2] — same memory, half as many elements.
            // Input host pointer set before runSession → backend maps via DMA.
            // Output host pointer set on DisplayBlock/frame before runSession.
            // Result: zero allocation, zero copy, zero memcpy per frame.
            let packed_w = (w / 2).max(1) as i32;
            let packed_shape = [1, 1, h as i32, packed_w];
            let packed_buf: &[i32] = unsafe {
                std::slice::from_raw_parts(
                    buf.as_ptr() as *const i32,
                    buf.len() / 4
                )
            };
            info!("pipeline stage=write_input buf={}B -> {} chans packed=true shape=[1,1,{},{}]",
                buf.len(), packed_buf.len() * 4, h, packed_w);
            path = "packed_zero_copy";
            t_prep_end = Instant::now();
            t_infer_start = Instant::now();
            unsafe {
                n = crate::mnn_sys::mnn_run_with_output(
                    interp.as_ptr(),
                    sess.as_ptr(),
                    packed_buf.as_ptr() as *const c_void,
                    0,                         // halide_type_int
                    32,                        // bits
                    packed_shape.as_ptr(),
                    packed_shape.len() as i32,
                    c"DisplayBlock/frame".as_ptr(), // named output
                    out_data.as_mut_ptr(),
                    max_out,
                );
            }

            let t_total_elapsed = t_start.elapsed();
            let t_infer_elapsed = t_infer_start.elapsed();

            if n <= 0 {
                error!("MNN inference failed: {} (input={}x{}, path={})", n, w, h, path);
                return Err(format!("MNN inference failed: {}", n));
            }

            info!("pipeline stage=infer_done path={} total={:?} ({}x{} -> {} flt)",
                path, t_infer_elapsed, w, h, n);

            // MNN can't resolve output shapes for complex ISP pipelines on OpenCL/CPU.
            // The output tensor has shape=[0,0,0,0] with dtype=4 (FLOAT) regardless
            // of actual model output format.  But after runSession, the output DATA
            // contains the correct number of elements — we detect channel count from n.
            let oh = p.target_height as usize;
            let ow = tw as usize;
            let n_valid = n.max(0) as usize;

            // ── Determine channel count and conversion mode ──
            // For conversion formats (Bgra, Rgba, etc.): detect model channel
            // count from actual output element count (MNN resolves n after
            // inference even when shape was [0,0,0,0]).
            // For model-native formats (FloatRgb, FloatBgra, PackedRgb):
            // channel count is fixed, no conversion.
            use crate::engine::OutputFormat;
            let (ch, raw_bytes_per_pixel) = match p.output_format {
                OutputFormat::FloatRgb => (3usize, 12usize),   // f32×3
                OutputFormat::FloatBgra => (4usize, 16usize),  // f32×4
                OutputFormat::PackedRgb => (1usize, 4usize),   // INT32
                _ => {
                    // Conversion formats: detect from n
                    let ch = if n_valid == oh * ow { 1 }        // INT32 packed
                        else if n_valid == oh * ow * 4 { 4 }    // bg4a float
                        else { 3 };                               // default float RGB
                    (ch, 4) // converted output is always u8×4 (or ×3 for Rgb/Bgr)
                }
            };
            let needs_convert = p.output_format.needs_conversion();
            let expected_n = oh * ow * ch;
            let nf = if n <= 0 || n < (expected_n as i32) / 10 {
                expected_n
            } else {
                n as usize
            };

            // Convert or copy output buffer, then release pool slot.
            let data_out: Vec<u8> = if needs_convert {
                let bgra = Self::convert_output(&out_data[..nf], ch, oh, ow, p.output_format);
                pool.release(buf_id);
                bgra
            } else {
                // Model-native: just memcpy the raw bytes (no unpack math).
                let n_bytes = oh * ow * raw_bytes_per_pixel;
                let raw = out_data.as_ptr() as *const u8;
                let raw_vec = unsafe { std::slice::from_raw_parts(raw, n_bytes) }.to_vec();
                pool.release(buf_id);
                raw_vec
            };

            // ── Read stats output tensors from MNN session and feed to controller ──
            // After inference, all output tensors are available in the session.
            // Try reading known stats output names.  If a tensor doesn't exist
            // in the graph (block not enabled in this profile), get_output returns None.
            #[cfg(feature = "mnn")]
            {
                use crate::mnn_sys::MnnInterpreterSafe;

                // ── Phase 1: Read raw stats tensors into local variables ──
                // (Avoid borrow conflicts with ctrl write_stats() above)
                let mut cm_vals: Option<[f32; 3]> = None;
                let mut ts_vals: Option<[f32; 6]> = None;
                let mut hist_vals: Option<[f32; 16]> = None;
                let mut zone_data: Option<(usize, usize, Vec<Vec<[f32; 3]>>)> = None;

                // ChannelMeansBlock/frame → [1, 3] or [3]
                if let Some(t) = interp.get_output(&sess, "ChannelMeansBlock/frame") {
                    if let Some(bytes) = t.as_bytes() {
                        let floats: &[f32] = unsafe {
                            std::slice::from_raw_parts(bytes.as_ptr() as *const f32, bytes.len() / 4)
                        };
                        if floats.len() >= 3 {
                            cm_vals = Some([floats[0], floats[1], floats[2]]);
                        }
                    }
                }
                // ToneStatsBlock/frame → [6]
                if let Some(t) = interp.get_output(&sess, "ToneStatsBlock/frame") {
                    if let Some(bytes) = t.as_bytes() {
                        let floats: &[f32] = unsafe {
                            std::slice::from_raw_parts(bytes.as_ptr() as *const f32, bytes.len() / 4)
                        };
                        let mut ts = [0.0f32; 6];
                        let n = floats.len().min(6);
                        ts[..n].copy_from_slice(&floats[..n]);
                        ts_vals = Some(ts);
                    }
                }
                // CoarseHistogramBlock/frame → [1, 16]
                if let Some(t) = interp.get_output(&sess, "CoarseHistogramBlock/frame") {
                    if let Some(bytes) = t.as_bytes() {
                        let floats: &[f32] = unsafe {
                            std::slice::from_raw_parts(bytes.as_ptr() as *const f32, bytes.len() / 4)
                        };
                        let mut hist = [0.0f32; 16];
                        let n = floats.len().min(16);
                        hist[..n].copy_from_slice(&floats[..n]);
                        hist_vals = Some(hist);
                    }
                }
                // ZoneStatsBlock/frame → [1, 3, rows, cols]
                if let Some(t) = interp.get_output(&sess, "ZoneStatsBlock/frame") {
                    if let Some(bytes) = t.as_bytes() {
                        let floats: &[f32] = unsafe {
                            std::slice::from_raw_parts(bytes.as_ptr() as *const f32, bytes.len() / 4)
                        };
                        let shape = t.shape();
                        if shape.len() >= 4 {
                            let rows = shape[2] as usize;
                            let cols = shape[3] as usize;
                            let mut zone_rgb = vec![vec![[0.0f32; 3]; cols]; rows];
                            for r in 0..rows {
                                for c in 0..cols {
                                    let idx = r * cols + c;
                                    if idx * 3 + 2 < floats.len() {
                                        zone_rgb[r][c][0] = floats[idx * 3];
                                        zone_rgb[r][c][1] = floats[idx * 3 + 1];
                                        zone_rgb[r][c][2] = floats[idx * 3 + 2];
                                    }
                                }
                            }
                            zone_data = Some((rows, cols, zone_rgb));
                        }
                    }
                }

                // ── Phase 2: Lock controller, write to stats slot, apply state ──
                if let Ok(mut ctrl) = self.controller.lock() {
                    // Check zone state before taking stats borrow
                    let zone_init_needed = zone_data.is_some()
                        && !ctrl.zone_stats_enabled
                        && zone_data.as_ref().map(|(r, c, _)| *r > 0 && *c > 0).unwrap_or(false);

                    if zone_init_needed {
                        if let Some((rows, cols, _)) = zone_data.as_ref() {
                            ctrl.init_zone_stats(*rows, *cols);
                        }
                    }

                    // Write raw stats to the write slot
                    let stats = ctrl.write_stats();
                    if let Some(cm) = cm_vals {
                        stats.channel_means = cm;
                    }
                    if let Some(ts) = ts_vals {
                        let n = ts.len().min(6);
                        stats.tone_stats[..n].copy_from_slice(&ts[..n]);
                    }
                    if let Some(hist) = hist_vals {
                        let n = hist.len().min(16);
                        stats.histogram[..n].copy_from_slice(&hist[..n]);
                        stats.histogram_valid = true;
                    }
                    if let Some((_rows, _cols, zone_rgb)) = zone_data {
                        stats.zone_stats = zone_rgb;
                        stats.zone_stats_valid = true;
                    }
                    drop(stats);

                    // Apply AE clipping gain from tone stats
                    if let Some(ts) = ts_vals {
                        if ts[3] > 0.0 && ts[5] > 1.0 {
                            ctrl.hist_constrained_gain = 1.0 - (ts[3] / ts[5]);
                        }
                    }

                    // ── Phase 3: Rotate triple-buffer ──
                    ctrl.rotate_stats();

                    // ── Phase 4: Process the newly-rotated process slot ──
                    // Read values into locals to avoid borrow conflicts
                    let ps = ctrl.process_stats();
                    let p_cm = ps.channel_means;
                    let p_ts = [ps.tone_stats[0], ps.tone_stats[1], ps.tone_stats[2]];
                    let p_hist = ps.histogram;
                    let p_hist_ok = ps.histogram_valid;
                    drop(ps);

                    if p_cm[0] > 0.0 || p_cm[1] > 0.0 || p_cm[2] > 0.0 {
                        ctrl.update_channel_stats(&p_cm);
                    }
                    if p_ts[0] > 0.0 {
                        ctrl.update_tone_stats(&p_ts);
                    }
                    if p_hist_ok {
                        let n = p_hist.len().min(16);
                        ctrl.update_histogram(&p_hist[..n]);
                    }
                }
            }

            info!("pipeline stage=output_frame {}×{} frame={}B total={:?}",
                tw, oh, data_out.len(), t_total_elapsed);
            let total_duration_ns = t_total_elapsed.as_nanos() as u64;
            let mut frame = IspFrame::new(tw, oh as u32, FrameFormat::Rgba8888);
            frame.data = data_out;
            // Stats are fed into the controller above (not returned in IspAuxOutput).
            // The controller modifies its internal state for the next frame.
            frame.aux = None;
            frame.timestamp_ns = 0;  // TODO: pass from HAL
            // prep = time before inference starts (setup + norm for float path)
            // infer = time for MNN inference only
            // total = prep + infer + bgra conversion
            // t_prep_end - t_start = prep time (shape setup, allocation)
            // For float fallback: prep also includes norm time (t_prep_end reset after norm)
            let prep_ns = t_prep_end.duration_since(t_start).as_nanos() as u64;
            frame.prep_duration_ns = prep_ns;
            frame.inference_duration_ns = t_infer_elapsed.as_nanos() as u64;
            frame.total_duration_ns = total_duration_ns;
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

