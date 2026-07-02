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
use std::collections::VecDeque;

use std::time::Instant;
use std::ffi::CStr;
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

// ── Session Pool (parallel inference) ─────────────────────────────────

/// One session together with its cached extra-input tensor handles.
#[cfg(feature = "mnn")]
struct SessionSlot {
    sess: MnnSessionSafe,
    tensor_pool: Vec<(String, MnnTensorSafe)>,
}

/// A pool of N sessions sharing one interpreter.
/// Threads acquire a slot, use it, release it back.
///
/// ⚠ Fields are dropped in declaration order. `slots` (sessions) must be
/// dropped BEFORE `interp` because session release needs the interpreter.
#[cfg(feature = "mnn")]
struct SessionPool {
    /// Queue of available slots (dropped first — sessions before interpreter).
    slots: Mutex<VecDeque<SessionSlot>>,
    /// Shared interpreter (owns the model). Dropped last.
    interp: MnnInterpreterSafe,
}

#[cfg(feature = "mnn")]
impl SessionPool {
    fn new(interp: MnnInterpreterSafe, backend: MnnBackendType, n: usize, num_threads: i32) -> Result<Self, String> {
        let extra_names = [
            "DemosaicCcmBlock/w",
            "DemosaicCcmBlock/b",
            "BayerWbBlock/gains",
            "ToneBlock/contrast",
            "ToneBlock/brightness",
            "ToneBlock/gamma_recip",
        ];
        let mut slots = VecDeque::with_capacity(n);
        for _ in 0..n {
            let sess = interp.create_session(backend, num_threads)
                .ok_or_else(|| format!("create session {} failed", slots.len()))?;
            let mut tensor_pool = Vec::new();
            for name in &extra_names {
                if let Some(t) = interp.get_input(&sess, name) {
                    tensor_pool.push((name.to_string(), t));
                }
            }
            slots.push_back(SessionSlot { sess, tensor_pool });
        }
        info!("SessionPool: {} slots created with {} extra tensors each", n, slots[0].tensor_pool.len());
        Ok(Self { interp, slots: Mutex::new(slots) })
    }

    /// Acquire a slot (blocks until one is free).
    fn acquire(&self) -> SessionGuard<'_> {
        loop {
            if let Some(slot) = self.slots.lock().unwrap().pop_front() {
                return SessionGuard { pool: self, slot: Some(slot) };
            }
            std::thread::yield_now();
        }
    }

    /// Return a slot to the pool.
    fn release(&self, slot: SessionSlot) {
        self.slots.lock().unwrap().push_back(slot);
    }
}

/// RAII guard: automatically returns the slot on drop.
#[cfg(feature = "mnn")]
struct SessionGuard<'a> {
    pool: &'a SessionPool,
    slot: Option<SessionSlot>,
}

#[cfg(feature = "mnn")]
impl<'a> std::ops::Deref for SessionGuard<'a> {
    type Target = SessionSlot;
    fn deref(&self) -> &Self::Target {
        self.slot.as_ref().unwrap()
    }
}

#[cfg(feature = "mnn")]
impl<'a> std::ops::DerefMut for SessionGuard<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.slot.as_mut().unwrap()
    }
}

#[cfg(feature = "mnn")]
impl<'a> Drop for SessionGuard<'a> {
    fn drop(&mut self) {
        if let Some(slot) = self.slot.take() {
            self.pool.release(slot);
        }
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
    /// Whether to preserve input type (int16/uint16/float16) instead of widening to int32.
    /// Used by NativeInt16 mode to avoid Cast(int32→int16) in UnpackCfaBlock.
    preserve_input_type: bool,
    /// Expected input tensor element count from model (for validation).
    expected_input_elements: Option<u32>,
    /// ISP controller for AWB/AE/CCM/tone parameter estimation.
    /// After each inference, stats output tensors are read and fed
    /// into the controller to update state for the next frame.
    pub controller: Mutex<IspController>,
    /// Number of parallel sessions.
    pool_size: usize,
    /// MNN interpreter + session pool.
    #[cfg(feature = "mnn")]
    pool: Option<SessionPool>,
    /// Pipeline-aligned output buffer pool.
    /// Pre-allocated at build(), recycled every frame via acquire/release.
    #[cfg(feature = "mnn")]
    #[allow(dead_code)]
    buf_pool: Mutex<crate::mnn_buffer::OutputBufferPool>,
}

#[cfg(feature = "mnn")]
impl Drop for MnnEngine {
    fn drop(&mut self) {
        // Drop pool (sessions) before interpreter to avoid dangling pointer.
        if let Some(pool) = self.pool.take() {
            drop(pool);
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
        Self::with_pool_size(backend, 4)
    }

    pub fn with_pool_size(backend: MnnBackend, pool_size: usize) -> Self {
        Self {
            backend,
            initialized: false,
            model_path: None,
            model_input_type: None,
            packed_input: false,
            preserve_input_type: false,
            expected_input_elements: None,
            controller: Mutex::new(IspController::new()),
            pool_size,
            #[cfg(feature = "mnn")]
            pool: None,
            #[cfg(feature = "mnn")]
            buf_pool: Mutex::new(crate::mnn_buffer::OutputBufferPool::new(3, 1)),
        }
    }

    /// Point to pre-converted .mnn file (skips on-the-fly conversion).
    pub fn set_model_path(&mut self, path: impl Into<String>) { self.model_path = Some(path.into()); }

    /// Set preferred workgroup size for Vulkan dispatch.
    /// Call after creating session and before inference.
    pub fn set_workgroup_size(&self, session: *mut std::ffi::c_void, size_x: u32, size_y: u32) {
        use crate::mnn_sys::MNNVulkanSetSessionWorkgroup;
        unsafe {
            MNNVulkanSetSessionWorkgroup(session, size_x as i32, size_y as i32);
        }
    }

    /// Query optimal workgroup size for current GPU.
    pub fn query_optimal_workgroup() -> (u32, u32) {
        use crate::mnn_sys::MNNVulkanQueryOptimalWorkgroup;
        let mut wx: i32 = 16;
        let mut wy: i32 = 16;
        unsafe { MNNVulkanQueryOptimalWorkgroup(&mut wx, &mut wy); }
        (wx as u32, wy as u32)
    }

    /// Set workgroup by preset name ("fast_4k", "low_power", "portrait", "universal").
    pub fn set_workgroup_preset(&self, preset: &str) {
        use crate::mnn_sys::MNNVulkanSetWorkgroupPreset;
        use std::ffi::CString;
        let c_name = CString::new(preset).unwrap();
        unsafe {
            MNNVulkanSetWorkgroupPreset(c_name.as_ptr());
        }
    }

    /// Hot-swap a const buffer at runtime for live 3A adjustments.
    /// Updates GPU const buffer with new float32 data without rebuilding the model.
    pub fn hot_swap_const_buffer(&self, session: *mut std::ffi::c_void, binding: i32, data: &[f32]) {
        use crate::mnn_sys::MNNVulkanHotSwapConstBuffer;
        unsafe {
            MNNVulkanHotSwapConstBuffer(
                session,
                binding,
                data.as_ptr() as *const std::ffi::c_void,
                (data.len() * 4) as i32,
            );
        }
    }

    /// Set whether to preserve input type (int16/uint16/float16) instead of widening to int32.
    /// Used by NativeInt16 mode to avoid Cast(int32→int16) in UnpackCfaBlock.
    pub fn set_preserve_input_type(&mut self, preserve: bool) {
        self.preserve_input_type = preserve;
    }

    /// Get whether to preserve input type.
    pub fn preserve_input_type(&self) -> bool {
        self.preserve_input_type
    }

    /// Register MNN engine factories for all available backends.
    /// Benchmarks each backend at startup and sets priority by actual FPS.
    /// Uses a 320×240 model (sweet-spot: realistic, not too slow to build).
    /// Times out after 2 s per backend — marks tardy backends with low priority.
    /// Called automatically by `cam_isp::init()`.
    #[cfg(feature = "mnn")]
    pub fn register_factories() {
        // Skip GPU benchmark — rapid-fire inference can crash Vulkan drivers
        // on some devices. Use static priorities instead.
        Self::register_with_defaults();
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
        let packed_w = (bench_w / 2).max(1) as i64;

        // ── Dimension helpers ──
        let dim_1packed = || vec![
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(h), Proto::tensor_dim_value(packed_w),
        ];
        let _dim_1hw = || vec![
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
        let dim_8packed = || vec![
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(8),
            Proto::tensor_dim_value(h), Proto::tensor_dim_value(packed_w),
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
        let inputs = vec![Proto::value_info("input", &dim_1packed(), 6)];  // 6 = INT32 packed: [1,1,H,W/2]
        let outputs = vec![Proto::value_info("output", &dim_3hw(), 1)];
        let vi = vec![
            Proto::value_info("conv1_out", &dim_8packed(), 1),
            Proto::value_info("relu1_out", &dim_8packed(), 1),
            Proto::value_info("conv2_out", &dim_8packed(), 1),
            Proto::value_info("relu2_out", &dim_8packed(), 1),
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
    #[allow(dead_code)]
    fn bench_one(backend: &MnnBackend, mnn_path: &str, w: u32, h: u32) -> Result<f64, String> {
        use crate::blocks::RawInputBlock;

        let mut engine = MnnEngine::new(*backend);
        engine.set_model_path(mnn_path);
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

        // Warmup (1 frame)
        let _ = engine.process(&crate::engine::ProcessParams::new(w, h, &buf));

        let budget = std::time::Duration::from_millis(300);
        let deadline = Instant::now() + budget;
        let mut count = 0u32;

        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining < std::time::Duration::from_millis(10) {
                break;
            }
            match engine.process(&crate::engine::ProcessParams::new(w, h, &buf)) {
                Ok(_) => count += 1,
                Err(e) => {
                    warn!("bench process error: {}", e);
                    break;
                }
            }
            // Small yield to avoid GPU starvation
            std::thread::yield_now();
        }

        // Explicitly drop engine to release session before returning
        drop(engine);

        let elapsed = (budget - deadline.saturating_duration_since(Instant::now()))
            .max(std::time::Duration::from_micros(1));
        if count == 0 {
            return Err("0 frames in budget".into());
        }
        Ok(count as f64 / elapsed.as_secs_f64())
    }

    // ── helpers ──

    #[allow(dead_code)]
    fn norm(buf: &[u8], max: f32) -> Vec<f32> {
        buf.chunks_exact(2).map(|c| (u16::from_ne_bytes([c[0], c[1]]) as f32 / max).clamp(0.0, 1.0)).collect()
    }

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
                if let Some(bytes) = t.as_bytes_mut() {
                    let src = unsafe {
                        std::slice::from_raw_parts(fused.as_ptr() as *const u8, 48)
                    };
                    if bytes.len() >= 48 { bytes[..48].copy_from_slice(src); }
                }
            }
        }
        // DemosaicCcmBlock/b [3] — CCM bias + tone brightness
        if let Some(t) = find(pool, "DemosaicCcmBlock/b") {
            if let Some(bytes) = t.as_bytes_mut() {
                // When tone is fused in, absorb brightness into bias
                let bias = [tone.brightness; 3];
                let src = unsafe { std::slice::from_raw_parts(bias.as_ptr() as *const u8, 12) };
                if bytes.len() >= 12 { bytes[..12].copy_from_slice(src); }
            }
        }
        // BayerWbBlock/gains [1,4,1,1] — white balance per-channel gains
        if let Some(gains) = bayer {
            if let Some(t) = find(pool, "BayerWbBlock/gains") {
                if let Some(bytes) = t.as_bytes_mut() {
                    let src = unsafe { std::slice::from_raw_parts(gains.as_ptr() as *const u8, 16) };
                    if bytes.len() >= 16 { bytes[..16].copy_from_slice(src); }
                }
            }
        }
        // ToneBlock/contrast [1]
        if let Some(t) = find(pool, "ToneBlock/contrast") {
            if let Some(bytes) = t.as_bytes_mut() {
                let src = unsafe { std::slice::from_raw_parts((&tone.contrast as *const f32) as *const u8, 4) };
                if bytes.len() >= 4 { bytes[..4].copy_from_slice(src); }
            }
        }
        // ToneBlock/brightness [1]
        if let Some(t) = find(pool, "ToneBlock/brightness") {
            if let Some(bytes) = t.as_bytes_mut() {
                let src = unsafe { std::slice::from_raw_parts((&tone.brightness as *const f32) as *const u8, 4) };
                if bytes.len() >= 4 { bytes[..4].copy_from_slice(src); }
            }
        }
        // ToneBlock/gamma_recip [1]
        if let Some(t) = find(pool, "ToneBlock/gamma_recip") {
            if let Some(bytes) = t.as_bytes_mut() {
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
    fn as_any(&self) -> &dyn std::any::Any { self }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any { self }
    fn controller(&self) -> &Mutex<IspController> { &self.controller }

    fn build(&mut self, head: Box<dyn IspBlock>, aux: Vec<Box<dyn IspBlock>>, _warp: Option<Box<dyn IspBlock>>, opset: i64) -> Result<(), String> {
        info!("MNN build backend={}", self.backend.id());

        #[cfg(feature = "mnn")]
        {
            use std::path::Path;
            use crate::mnn_converter::MnnConvertOptions;


            let mnn = match &self.model_path {
                Some(p) => { info!("build [tid={:?}]: using model_path={}", std::thread::current().id(), p); p.clone() },
                None => {
                    // Build ONNX graph: head + aux as pipeline, stats as aux_blocks
                    // But we don't know which aux are stats — pass all as pipeline for now
                    let mut all: Vec<Box<dyn IspBlock>> = vec![head];
                    all.extend(aux);
                    let refs: Vec<&dyn IspBlock> = all.iter().map(|b| b.as_ref()).collect();
                    info!("build [tid={:?}]: composing ONNX from {} blocks", std::thread::current().id(), refs.len());
                    let onnx = crate::pipeline::GraphComposer::compose_from_vec(&refs, &[], opset)?;
                    info!("ONNX [tid={:?}]: {} bytes for MNN conversion", std::thread::current().id(), onnx.len());

                    let on = format!(".mnn_temp_{}.onnx", std::process::id());
                    let mn = on.replace(".onnx", ".mnn");
                    std::fs::write(&on, &onnx).map_err(|e| format!("write: {}", e))?;
                    // Save a copy for inspection
                    let _ = std::fs::copy(&on, ".mnn_last_pipeline.onnx");
                    let mut opts = MnnConvertOptions::default();
                    opts.preserve_input_type = self.preserve_input_type;
                    info!("preserve_input_type: {}, optimize_level: {}", self.preserve_input_type, opts.optimize_level);
                    crate::mnn_converter::convert_onnx_to_mnn(&on, &mn, Some(&opts)).map_err(|e| format!("convert: {}", e))?;
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

            // Use first session to probe model input type (before building pool)
            let probe_sess = interp.create_session(self.backend.to_sys(), 4)
                .or_else(|| {
                    // Fallback: try CPU if requested backend fails
                    if self.backend != MnnBackend::Cpu {
                        warn!("Backend {:?} unavailable, falling back to CPU", self.backend);
                        self.backend = MnnBackend::Cpu;
                        interp.create_session(MnnBackendType::Cpu, 4)
                    } else {
                        None
                    }
                })
                .ok_or("probe session create fail (all backends exhausted)")?;
            let mut input_code = 0i32;
            let mut input_bits = 0i32;
            unsafe {
                crate::mnn_sys::mnn_get_model_input_type(
                    interp.as_ptr(),
                    probe_sess.as_ptr(),
                    &mut input_code,
                    &mut input_bits,
                );
            }
            self.model_input_type = Some((input_code, input_bits));
            let expected_elems = unsafe {
                crate::mnn_sys::mnn_get_model_input_elements(interp.as_ptr(), probe_sess.as_ptr())
            };
            self.expected_input_elements = if expected_elems > 0 { Some(expected_elems as u32) } else { None };
            self.packed_input = input_code == 0 && input_bits == 32;
            info!("MNN model input type: code={}, bits={} packed={} expected_elems={}",
                input_code, input_bits, self.packed_input, expected_elems);
            // Don't need probe session any more — pool will create its own.
            drop(probe_sess);

            // Build session pool (N sessions for parallel inference)
            // Backend was already validated by probe session above.
            let pool = SessionPool::new(
                interp,
                self.backend.to_sys(),
                self.pool_size.max(1),
                4,
            )?;
            info!("MNN engine loaded from {} (backend={:?}) with {} sessions",
                mnn, self.backend, self.pool_size);
            self.pool = Some(pool);
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
            let pool = self.pool.as_ref().ok_or("no pool")?;
            let slot = pool.acquire();
            let sess = &slot.sess;
            let interp = &pool.interp;

            let t_start = Instant::now();

            info!("pipeline stage=arrive raw={}×{} bayer={}B sensor_max={}",
                w, h, buf.len(), smax);

            // ── Pre-allocate output buffer (zero-copy target) ──
            // MNN writes directly into this Vec via host pointer.
            // We return it as frame.data — no memcpy at all.
            let max_out = (tw * h * 4) as usize; // max f32 elements: H×W×4
            let mut out_bytes: Vec<u8> = Vec::with_capacity(max_out * 4);
            unsafe { out_bytes.set_len(max_out * 4); }
            let out_ptr = out_bytes.as_mut_ptr() as *mut f32;

            // Determine inference path based on model input type
            let path: &str;
            let n: i32;
            let t_prep_end: std::time::Instant;
            let t_infer_start: std::time::Instant;
            let t_tensor_before: std::time::Instant;
            let t_tensor_after: std::time::Instant;

            // Determine if packed: model expects INT32 packed input
            let is_packed = if self.packed_input {
                if let Some(expected) = self.expected_input_elements {
                    expected == h * w / 2
                } else {
                    true
                }
            } else {
                false
            };

            // ── Pre-inference: resize session and set extra inputs ──
            t_tensor_before = Instant::now();
            {
                let shape = if is_packed {
                    vec![1, 1, h as i32, (w / 2).max(1) as i32]
                } else {
                    vec![1, 1, h as i32, w as i32]
                };
                if let Some(t) = interp.get_first_input(sess) {
                    let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &shape);
                }
                let _ = sess.resize();
                Self::set_extra_inputs(&slot.tensor_pool, ccm, tone, bayer, awb, bayer_pattern);
            }
            t_tensor_after = Instant::now();
            info!("pipeline stage=tensor_assign elapsed={:?}",
                t_tensor_after.duration_since(t_tensor_before));

            debug!("MNN process: w={}, h={}, packed_w={}, is_packed={}, expected={:?}",
                w, h, (w / 2).max(1), is_packed, self.expected_input_elements);

            let (buffer_ptr, buffer_type_code, buffer_type_bits, input_shape, path_str) = if is_packed {
                let packed_w = (w / 2).max(1) as i32;
                let packed_shape = [1, 1, h as i32, packed_w];
                let packed_buf: &[i32] = unsafe {
                    std::slice::from_raw_parts(buf.as_ptr() as *const i32, buf.len() / 4)
                };
                (
                    packed_buf.as_ptr() as *const c_void,
                    0,   // INT32
                    32,
                    packed_shape.to_vec(),
                    "packed_zero_copy"
                )
            } else {
                let raw_shape = [1, 1, h as i32, w as i32];
                let (code, bits) = self.model_input_type.unwrap_or((0, 16));
                (
                    buf.as_ptr() as *const c_void,
                    code,
                    bits,
                    raw_shape.to_vec(),
                    "raw_zero_copy"
                )
            };

            info!("pipeline stage=write_input buf={}B -> {} chans packed={} shape=[1,1,{},{}]",
                buf.len(),
                if is_packed {
                    (buf.len() / 4) as usize
                } else {
                    (buf.len() / 2) as usize
                },
                is_packed,
                input_shape[2],
                input_shape[3]
            );

            path = path_str;
            t_prep_end = Instant::now();
            t_infer_start = Instant::now();
            unsafe {
                n = crate::mnn_sys::mnn_run_with_output(
                    interp.as_ptr(), sess.as_ptr(),
                    buffer_ptr,
                    buffer_type_code,
                    buffer_type_bits,
                    input_shape.as_ptr(),
                    input_shape.len() as i32,
                    c"DisplayBlock/frame".as_ptr(),
                    out_ptr,
                    max_out as i32,
                );
                // ------------------------------------------------------------------
                // Optional MNN profiling dump – prints per‑node timings if the
                // MNN session was built with profiling enabled.
                // This is a best‑effort call; if the function is unavailable the
                // code will compile but produce no output.
                let info_c = crate::mnn_sys::MNN_GetSessionInfoString(sess.as_ptr());
                if !info_c.is_null() {
                    let info = CStr::from_ptr(info_c).to_string_lossy();
                    info!("MNN profiling info:\n{}", info);
                }
                // ------------------------------------------------------------------
            }

            let t_total_elapsed = t_start.elapsed();
            let t_infer_elapsed = t_infer_start.elapsed();

            if n <= 0 {
                error!("MNN inference failed: {} (input={}x{}, path={})", n, w, h, path);
                return Err(format!("MNN inference failed: {}", n));
            }

            info!("pipeline stage=infer_done path={} total={:?} ({}x{} -> {} elts) prep={:?} infer={:?}",
                path, t_infer_elapsed, w, h, n,
                t_prep_end - t_start, t_infer_elapsed);

            // ── Determine output size from format and actual element count ──
            let oh = p.target_height as usize;
            let ow = tw as usize;
            let n_valid = n.max(0) as usize;
            let bpp = p.output_format.bytes_per_pixel();
            let n_bytes = oh * ow * bpp;
            // Trim Vec to actual data written by MNN
            let n_bytes_actual = n_valid.min(n_bytes / bpp) * bpp;
            unsafe { out_bytes.set_len(n_bytes_actual); }

            // All conversion is done by the ONNX graph — just return raw bytes.
            let data_out = out_bytes;

            // ── Read stats output tensors from MNN session and feed to controller ──
            let mut calib_vals: Option<[f32; 24]> = None;
            // After inference, all output tensors are available in the session.
            // Try reading known stats output names.  If a tensor doesn't exist
            // in the graph (block not enabled in this profile), get_output returns None.
            #[cfg(feature = "mnn")]
            {
                

                // ── Phase 1: Read raw stats tensors into local variables ──
                // (Avoid borrow conflicts with ctrl write_stats() above)
                let mut cm_vals: Option<[f32; 3]> = None;
                let mut ts_vals: Option<[f32; 6]> = None;
                let mut hist_vals: Option<[f32; 16]> = None;
                let mut zone_data: Option<(usize, usize, Vec<Vec<[f32; 3]>>)> = None;

                // ChannelMeansBlock/frame → [1, 3] or [3]
                if let Some(t) = interp.get_output(sess, "ChannelMeansBlock/frame") {
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
                if let Some(t) = interp.get_output(sess, "ToneStatsBlock/frame") {
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
                if let Some(t) = interp.get_output(sess, "CoarseHistogramBlock/frame") {
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

                // CalibrationBlock/frame → [24] (quad means, vars, mins, maxs, ranges, frame stats)
                if let Some(t) = interp.get_output(sess, "CalibrationBlock/frame") {
                    if let Some(bytes) = t.as_bytes() {
                        let floats: &[f32] = unsafe {
                            std::slice::from_raw_parts(bytes.as_ptr() as *const f32, bytes.len() / 4)
                        };
                        let mut cal = [0.0f32; 24];
                        let n = floats.len().min(24);
                        cal[..n].copy_from_slice(&floats[..n]);
                        calib_vals = Some(cal);
                    }
                }

                // ZoneStatsBlock/frame → [1, 3, rows, cols]
                if let Some(t) = interp.get_output(sess, "ZoneStatsBlock/frame") {
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
                    // drop(stats) — implicit at end of scope

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
                    // drop(ps) — implicit

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

            info!("pipeline stage=output_frame {}×{} frame={}B total={:?} prep={:?} resize={:?} infer={:?}",
                tw, oh, data_out.len(), t_total_elapsed,
                t_prep_end - t_start,
                t_tensor_after - t_tensor_before,
                t_infer_elapsed);
            let total_duration_ns = t_total_elapsed.as_nanos() as u64;
            let mut frame = IspFrame::new(tw, oh as u32, FrameFormat::Rgba8888);
            frame.data = data_out;
            // Stats are fed into the controller above (not returned in IspAuxOutput).
            // The controller modifies its internal state for the next frame.
            frame.aux = Some(IspAuxOutput {
                calibration_stats: calib_vals,
                ..Default::default()
            });
            frame.timestamp_ns = p.timestamp_ns;
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

