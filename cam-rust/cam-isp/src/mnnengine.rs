//! MNN inference engine — ONNX → MNN convert then run inference.
//!
//! Build flow:
//!   1. Compose ONNX model from pipeline blocks
//!   2. Convert ONNX → .mnn via MNNConvert CLI (or load pre-converted)
//!   3. Load .mnn with MNN runtime
//!
//! Inference flow:
//!   1. Set input shape `[1,1,H,W]`
//!   2. Copy INT16 sensor data as normalized float32
//!   3. Run session
//!   4. Read output float32 → BGRA U8

use std::sync::Mutex;

use log::{debug, error, info, warn};
use std::ffi::CStr;
use std::time::Instant;

/// Zone data: (rows, cols, zone_rgb`[r]``[c]` = `[f32; 3]`)
type ZoneData = (usize, usize, Vec<Vec<[f32; 3]>>);

use crate::controller::IspController;
use crate::engine::{IspEngine, ProcessParams};
use crate::pipeline::{IspAuxOutput, IspBlock, IspFrame};
use cam_types::{FrameFormat, ToneParams};

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
        match self {
            Self::Vulkan => "mnn_vulkan",
            Self::Opencl => "mnn_opencl",
            Self::OpenGl => "mnn_opengl",
            Self::CpuNeon => "mnn_neon",
            Self::Cpu => "mnn_cpu",
        }
    }
    pub fn priority(&self) -> i32 {
        match self {
            Self::Vulkan => 99,
            Self::Opencl => 55,
            Self::OpenGl => 50,
            Self::CpuNeon => 75,
            Self::Cpu => 65,
        }
    }
    #[cfg(feature = "mnn")]
    fn to_sys(self) -> MnnBackendType {
        match self {
            Self::Vulkan => MnnBackendType::Vulkan,
            Self::Opencl => MnnBackendType::Opencl,
            Self::OpenGl => MnnBackendType::Opengl,
            _ => MnnBackendType::Cpu,
        }
    }
}

// ── Session Pool (parallel inference) ─────────────────────────────────
// SessionPool extracted to mnn_session_pool.rs
pub(crate) use crate::mnn_session_pool::SessionPool;

// ── Engine ──────────────────────────────────────────────────────────────

/// How the engine feeds the model's input tensor. Inferred at build() from
/// the MNN model's declared input shape (channels) and element type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputMode {
    /// `[1,2,H,W/2]` INT32 — packed u16 pairs split into even/odd i32 lanes
    /// on CPU (the packed-INT32 convention).
    SplitInt32,
    /// `[1,2,H,W/2]` FLOAT32 — native 2×int16 buffer split into even/odd f32
    /// lanes on CPU. MNN's ONNX→MNN converter upcasts INT16-declared graph
    /// inputs to FLOAT32, so a "2×int16" model arrives here as float.
    SplitFloat,
    /// `[1,1,H,W]` FLOAT32 — native int16 buffer converted to f32 on CPU
    /// (unpack_blc16 / UnpackCfa-style heads).
    RawFloat,
    /// `[1,1,H,W]` INT32 — caller buffer passed through as-is.
    RawInt,
}

pub struct MnnEngine {
    backend: MnnBackend,
    initialized: bool,
    model_path: Option<String>,
    model_input_type: Option<(i32, i32)>,
    /// Head block's graph input tensor name (e.g. `RawInputPackedBlock/frame`).
    /// MNN's `getSessionInput(nullptr)` returns the alphabetically-first input,
    /// which is a runtime-fed weight tensor in multi-input models — the frame
    /// must always be looked up by name.
    frame_input_name: Option<String>,
    /// Input convention inferred from the converted model (shape + type).
    input_mode: Option<InputMode>,
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
    /// GPU hang watchdog for fault-tolerant inference.
    /// Triggers CPU fallback when MNN/Vulkan hangs exceed threshold.
    watchdog: crate::gpu_watchdog::GpuWatchdog,
    /// Cached input tensor shape to skip redundant sess.resize().
    /// Fixed-resolution pipelines (e.g. 4K→FHD) never change shape,
    /// so resize() is only needed on the very first frame.
    /// Stores (h, w, mode) of the last fully-configured frame.
    last_input_shape: Mutex<Option<(i32, i32, InputMode)>>,
    /// Reused even/odd lane-split buffer for the packed INT32 input path.
    /// MNN's Vulkan backend cannot run integer elementwise ops, so the
    /// engine splits the two u16 lanes on CPU before feeding `[1,2,H,W/2]`.
    split_scratch: Mutex<Vec<i32>>,
    /// Reused f32 scratch: u16→f32 lane conversion for native-int16 models
    /// (SplitFloat) and full-frame conversion for raw FLOAT32 models (RawFloat).
    scratch_f32: Mutex<Vec<f32>>,
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

    /// Run the ISP session with the input bound to EXTERNAL device memory
    /// (dma-buf fd / AHardwareBuffer) via `Tensor::setDevicePtr`. This is the
    /// true zero-copy path for large MIPI Bayer: the camera's CMA/dma-buf pages
    /// are consumed by the GPU directly, with no CPU staging copy and no heap
    /// allocation. Requires a Vulkan backend that honors external-memory import
    /// (custom MNN build); otherwise `setDevicePtr` is a no-op at the backend
    /// level and inference falls back to host staging.
    ///
    /// # Contract
    /// `ext_handle` must be a valid dma-buf fd (Linux) or AHardwareBuffer*
    /// (Android) sized for the model input tensor. `out_data` must hold at least
    /// the output tensor's element count.
    pub fn run_external_zero_copy(
        &self,
        ext_handle: i64,
        memory_type: i32,
        in_shape: &[i32],
        out_data: &mut [f32],
    ) -> Result<(), String> {
        let pool = self.pool.as_ref().ok_or("MNN engine not initialized")?;
        let interp = pool.interp.as_ptr();
        let guard = pool.acquire();
        let sess = guard.sess.as_ptr();
        let ret = unsafe {
            crate::mnn_sys::mnn_run_external_zero_copy(
                interp,
                sess,
                ext_handle,
                memory_type,
                in_shape.as_ptr(),
                in_shape.len() as crate::mnn_sys::c_int,
                out_data.as_mut_ptr(),
                out_data.len() as crate::mnn_sys::c_int,
            )
        };
        if ret == 0 {
            Ok(())
        } else {
            Err(format!("mnn_run_external_zero_copy failed (code {})", ret))
        }
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
            frame_input_name: None,
            input_mode: None,
            preserve_input_type: false,
            expected_input_elements: None,
            controller: Mutex::new(IspController::new()),
            pool_size,
            #[cfg(feature = "mnn")]
            pool: None,
            #[cfg(feature = "mnn")]
            buf_pool: Mutex::new(crate::mnn_buffer::OutputBufferPool::new(3, 1)),
            watchdog: crate::gpu_watchdog::GpuWatchdog::new(
                crate::gpu_watchdog::WatchdogConfig::default(),
            ),
            last_input_shape: Mutex::new(None),
            split_scratch: Mutex::new(Vec::new()),
            scratch_f32: Mutex::new(Vec::new()),
        }
    }

    /// Point to pre-converted .mnn file (skips on-the-fly conversion).
    pub fn set_model_path(&mut self, path: impl Into<String>) {
        self.model_path = Some(path.into());
    }

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
        unsafe {
            MNNVulkanQueryOptimalWorkgroup(&mut wx, &mut wy);
        }
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
    pub fn hot_swap_const_buffer(
        &self,
        session: *mut std::ffi::c_void,
        binding: i32,
        data: &[f32],
    ) {
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

    /// Start background GPU hang watchdog.
    /// Triggers CPU fallback when GPU execution exceeds threshold.
    pub fn start_watchdog(&self) {
        self.watchdog.start_monitoring();
    }

    /// Stop background GPU hang watchdog.
    pub fn stop_watchdog(&self) {
        self.watchdog.stop_monitoring();
    }

    /// Get reference to GPU watchdog (for advanced configuration)
    pub fn watchdog(&self) -> &crate::gpu_watchdog::GpuWatchdog {
        &self.watchdog
    }

    /// Check if CPU fallback is currently requested by watchdog
    pub fn needs_cpu_fallback(&self) -> bool {
        self.watchdog.fallback_requested()
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
        let backends = [
            MnnBackend::CpuNeon,
            MnnBackend::Cpu,
            MnnBackend::Vulkan,
            MnnBackend::Opencl,
            MnnBackend::OpenGl,
        ];
        for be in &backends {
            let name = be.id();
            let pri = be.priority();
            let b = *be;
            let create_fn = Box::new(move || Box::new(MnnEngine::new(b)) as Box<dyn IspEngine>);
            register_engine(EngineFactory {
                name,
                priority: pri,
                create_fn,
            });
        }
        info!(
            "Registered {} MNN engine factories (default priorities)",
            backends.len()
        );
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
    ///   Input `[1,1,H,W]` → Conv1(1→8,3×3) → Relu → Conv2(8→8,3×3) → Relu
    ///   → GlobalAveragePool `[1,8,1,1]` → Reshape `[1,8]`
    ///   → MatMul`[8,64]` → Relu → MatMul`[64,8]` → Reshape `[1,8,1,1]`
    ///   → Resize (nearest, ×H×W) `[1,8,H,W]` → Conv3(8→3,3×3)
    ///   → GridSampler(bilinear) → Output `[1,3,H,W]`
    ///
    /// ~2K weights (all Conv), compute scales with H×W.
    #[cfg(feature = "mnn")]
    pub fn build_bench_model(bench_w: u32, bench_h: u32) -> Result<String, String> {
        use crate::mnn_converter::convert_onnx_buffer;
        use crate::onnx::proto::Proto;

        let h = bench_h as i64;
        let w = bench_w as i64;
        let packed_w = (bench_w / 2).max(1) as i64;

        // ── Dimension helpers ──
        let dim_1packed = || {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(packed_w),
            ]
        };
        let _dim_1hw = || {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        };
        let dim_3hw = || {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        };
        let dim_8hw = || {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(8),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        };
        let dim_8packed = || {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(8),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(packed_w),
            ]
        };
        let dim_8111 = || {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(8),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
            ]
        };
        let dim_8 = || vec![Proto::tensor_dim_value(1), Proto::tensor_dim_value(8)];
        let dim_64 = || vec![Proto::tensor_dim_value(1), Proto::tensor_dim_value(64)];

        // ── Weights (all resolution-independent, ~2K params total) ──
        let conv1_w = Self::rand_weight(8 * 3 * 3); // [8,1,3,3]  = 72
        let conv2_w = Self::rand_weight(8 * 8 * 3 * 3); // [8,8,3,3]  = 576
        let conv3_w = Self::rand_weight(3 * 8 * 3 * 3); // [3,8,3,3]  = 216
        let mm1_w = Self::rand_weight(8 * 64); // [8,64]     = 512
        let mm2_w = Self::rand_weight(64 * 8); // [64,8]     = 512
                                               // total: 72+576+216+512+512 = 1,888 (constant at any resolution!)

        // Identity grid for GridSampler: [1, H, W, 2] in [-1, 1]
        let mut grid = Vec::with_capacity(h as usize * w as usize * 2);
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
            Proto::node(
                "Conv",
                &["input", "conv1_w", "conv1_b"],
                &["conv1_out"],
                &[
                    Proto::attribute_ints("kernel_shape", &[3, 3]),
                    Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                    Proto::attribute_ints("strides", &[1, 1]),
                    Proto::attribute_int("group", 1),
                ],
            ),
            Proto::node("Relu", &["conv1_out"], &["relu1_out"], &[]),
            // Conv2 8→8, 3×3
            Proto::node(
                "Conv",
                &["relu1_out", "conv2_w", "conv2_b"],
                &["conv2_out"],
                &[
                    Proto::attribute_ints("kernel_shape", &[3, 3]),
                    Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                    Proto::attribute_ints("strides", &[1, 1]),
                    Proto::attribute_int("group", 1),
                ],
            ),
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
            Proto::node(
                "Reshape",
                &["mm2_relu", "shape_8111"],
                &["reshape8_out"],
                &[],
            ),
            // Resize from 1×1 → H×W (nearest, fast)
            Proto::node(
                "Resize",
                &["reshape8_out", "", "resize_scales", ""],
                &["resize_out"],
                &[Proto::attribute_string("mode", "nearest")],
            ),
            // Conv3 8→3, 3×3
            Proto::node(
                "Conv",
                &["resize_out", "conv3_w", "conv3_b"],
                &["conv3_out"],
                &[
                    Proto::attribute_ints("kernel_shape", &[3, 3]),
                    Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                    Proto::attribute_ints("strides", &[1, 1]),
                    Proto::attribute_int("group", 1),
                ],
            ),
            // GridSampler with identity grid
            Proto::node(
                "GridSampler",
                &["conv3_out", "identity_grid"],
                &["output"],
                &[
                    Proto::attribute_string("mode", "bilinear"),
                    Proto::attribute_string("padding_mode", "zeros"),
                    Proto::attribute_int("align_corners", 0),
                ],
            ),
        ];

        // ── Value info ──
        let inputs = vec![Proto::value_info("input", &dim_1packed(), 6)]; // 6 = INT32 packed: [1,1,H,W/2]
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

        let mnn_bytes = convert_onnx_buffer(&model).map_err(|e| format!("convert: {}", e))?;
        let mnn_path = format!(".mnn_bench_{}.mnn", std::process::id());
        std::fs::write(&mnn_path, &mnn_bytes).map_err(|e| format!("write mnn: {}", e))?;
        Ok(mnn_path)
    }

    /// Helper: fill a tensor with small random values (stddev ~0.01).
    #[cfg(feature = "mnn")]
    fn rand_weight(n: usize) -> Vec<f32> {
        use std::time::{SystemTime, UNIX_EPOCH};
        // Simple xorshift to avoid std::rand dependency
        let seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .subsec_nanos() as u64;
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
        engine
            .build(head, vec![], None, 16)
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
        buf.chunks_exact(2)
            .map(|c| (u16::from_ne_bytes([c[0], c[1]]) as f32 / max).clamp(0.0, 1.0))
            .collect()
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
        isp_params: Option<&crate::isp_params::IspParams>,
        warp_grid: Option<&[f32]>,
        warp_shading: Option<&[f32]>,
    ) {
        /// Look up a cached tensor handle by name prefix (fast — no CString alloc).
        fn find<'a>(
            pool: &'a [(String, crate::mnn_sys::MnnTensorSafe)],
            name: &str,
        ) -> Option<&'a crate::mnn_sys::MnnTensorSafe> {
            pool.iter().find(|(n, _)| n == name).map(|(_, t)| t)
        }

        /// Write a single f32 value into a named tensor (4 bytes).
        /// No-op if the tensor is missing or too small.
        fn write_f32(pool: &[(String, crate::mnn_sys::MnnTensorSafe)], name: &str, val: f32) {
            if let Some(t) = find(pool, name) {
                if let Some(bytes) = t.as_bytes_mut() {
                    if bytes.len() >= 4 {
                        bytes[..4].copy_from_slice(&val.to_ne_bytes());
                    }
                }
            }
        }

        /// Write a fixed-size f32 slice into a named tensor.
        /// No-op if the tensor is missing or too small.
        fn write_f32_array(
            pool: &[(String, crate::mnn_sys::MnnTensorSafe)],
            name: &str,
            vals: &[f32],
        ) {
            if let Some(t) = find(pool, name) {
                if let Some(bytes) = t.as_bytes_mut() {
                    let byte_count = vals.len() * 4;
                    if bytes.len() >= byte_count {
                        let src = unsafe {
                            std::slice::from_raw_parts(vals.as_ptr() as *const u8, byte_count)
                        };
                        bytes[..byte_count].copy_from_slice(src);
                    }
                }
            }
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
        const DEMO_RGGB: [f32; 12] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0];
        const DEMO_BGGR: [f32; 12] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.5, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0];
        const DEMO_GRBG: [f32; 12] = [0.0, 1.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0];
        const DEMO_GBRG: [f32; 12] = [0.0, 0.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0];

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
                    let src =
                        unsafe { std::slice::from_raw_parts(fused.as_ptr() as *const u8, 48) };
                    if bytes.len() >= 48 {
                        bytes[..48].copy_from_slice(src);
                    }
                }
            }
        }
        // DemosaicCcmBlock/b [3] — CCM bias + tone brightness
        write_f32_array(pool, "DemosaicCcmBlock/b", &[tone.brightness; 3]);
        // BayerWbBlock/gains [1,4,1,1] — white balance per-channel gains
        if let Some(gains) = bayer {
            write_f32_array(pool, "BayerWbBlock/gains", gains);
        }
        // DemosaicBlock/w + b [3,4,1,1]/[3] — production demosaic conv weights
        // (depend on the sensor Bayer pattern). CcmBlock_ccm/matrix [3,3] —
        // color matrix (identity when not provided). All three are true
        // runtime inputs on the production pipeline (no initializer defaults).
        write_f32_array(pool, "DemosaicBlock/w", demo);
        write_f32_array(pool, "DemosaicBlock/b", &[0.0f32; 3]);
        let ccm_mat: [f32; 9] = match ccm {
            Some(m) => *m,
            None => [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        };
        write_f32_array(pool, "CcmBlock_ccm/matrix", &ccm_mat);
        // ToneBlock/contrast [1]
        write_f32(pool, "ToneBlock/contrast", tone.contrast);
        // ToneBlock/brightness [1]
        write_f32(pool, "ToneBlock/brightness", tone.brightness);
        // ToneBlock/gamma_recip [1]
        write_f32(pool, "ToneBlock/gamma_recip", tone.gamma_recip);
        // SaturationBlock/scale [3] — color saturation strength
        write_f32_array(pool, "saturation/scale", &[1.0f32; 3]);
        // Sharpen/strength [1] — sharpening strength
        write_f32(pool, "Sharpen/strength", 1.0);
        // LdciBlock/strength [1] — local contrast strength
        write_f32(pool, "LdciBlock/strength", 1.0);
        // FcsBlock/gain [3] — per-channel gain (default: identity)
        write_f32_array(pool, "FcsBlock/gain", &[1.0f32; 3]);
        // FcsBlock/bias [3] — per-channel bias (default: zero)
        write_f32_array(pool, "FcsBlock/bias", &[0.0f32; 3]);
        // NormalizeBlock/max_val [1] — max value for division (default: 65535)
        write_f32(pool, "NormalizeBlock/max_val", 65535.0);
        // GammaBlock / AutoContrastBlock / DisplayBlock — always write
        // defaults so pure-input tensors are never left uninitialized when
        // no isp_params are supplied; the isp_params branch below overrides.
        write_f32(pool, "Gamma/inv_gamma", 1.0 / 2.2);
        write_f32(pool, "Gamma/min", 0.0);
        write_f32(pool, "Gamma/max", 1.0);
        write_f32(pool, "Gamma/lift", 0.0);
        write_f32(pool, "Gamma/norm", 1.0);
        write_f32(pool, "AutoContrast/lift", 0.0);
        write_f32(pool, "AutoContrast/half", 0.5);
        write_f32(pool, "AutoContrast/contrast_w", 1.0);
        write_f32(pool, "AutoContrast/zero", 0.0);
        write_f32(pool, "AutoContrast/one", 1.0);
        write_f32(pool, "DisplayBlock/scale", 1.0);
        write_f32(pool, "DisplayBlock/gamma_exp", 1.0 / 2.4);
        write_f32(pool, "DisplayBlock/zero", 0.0);
        write_f32(pool, "DisplayBlock/one", 1.0);
        // GammaBlock — inv_gamma, min, max, lift, norm
        if let Some(isp) = isp_params {
            let inv_gamma = if isp.tone.gamma > 0.0 {
                1.0 / isp.tone.gamma
            } else {
                1.0
            };
            write_f32(pool, "Gamma/inv_gamma", inv_gamma);
            write_f32(pool, "Gamma/min", isp.tone.black_crush);
            write_f32(pool, "Gamma/max", isp.tone.white_clip);
            write_f32(pool, "Gamma/lift", isp.tone.black_crush);
            write_f32(pool, "Gamma/norm", 1.0);
            // AutoContrastBlock — lift, half, contrast_w, zero, one
            write_f32(pool, "AutoContrast/lift", isp.tone.black_crush);
            write_f32(pool, "AutoContrast/half", 0.5);
            write_f32(pool, "AutoContrast/contrast_w", isp.tone.contrast);
            write_f32(pool, "AutoContrast/zero", 0.0);
            write_f32(pool, "AutoContrast/one", 1.0);
            // DisplayBlock — scale, gamma_exp, zero, one (FloatRgb path)
            write_f32(pool, "DisplayBlock/scale", 1.0);
            write_f32(pool, "DisplayBlock/gamma_exp", 1.0 / 2.4);
            write_f32(pool, "DisplayBlock/zero", 0.0);
            write_f32(pool, "DisplayBlock/one", 1.0);
            // SaturationBlock/scale [3] — from saturation.factor
            write_f32_array(pool, "saturation/scale", &[isp.saturation.factor; 3]);
            // Sharpen/strength [1] — from sharpen.amount
            write_f32(pool, "Sharpen/strength", isp.sharpen.amount);
            // LdciBlock/strength [1] — from denoise params (use sharpen radius as proxy)
            write_f32(pool, "LdciBlock/strength", isp.sharpen.radius.max(0.1));
            // FcsBlock/gain [3] — per-channel gain from CCM diagonal
            let fcs_gain = if let Some(ccm) = ccm {
                [ccm[0], ccm[4], ccm[8]]
            } else {
                [1.0f32; 3]
            };
            write_f32_array(pool, "FcsBlock/gain", &fcs_gain);
            // FcsBlock/bias [3] — per-channel bias
            write_f32_array(pool, "FcsBlock/bias", &[isp.tone.brightness; 3]);
            // NormalizeBlock/max_val [1] — from sensor_max or blc
            let max_val = isp_params
                .as_ref()
                .map(|p| p.blc.r * 2.0)
                .unwrap_or(65535.0);
            write_f32(pool, "NormalizeBlock/max_val", max_val);
        }

        // ── Warp grid + shading LUT (runtime extra_inputs) ────────
        // These are declared as graph inputs (not initializers) to prevent
        // MNN from DCE-ing the ISP graph. The engine writes the actual
        // per-frame data here.
        if let Some(grid) = warp_grid {
            write_f32_array(pool, "WarpGrid/grid", grid);
        }
        if let Some(lut) = warp_shading {
            write_f32_array(pool, "WarpGrid/shading_lut", lut);
        }
    }

    /// Resize + fill the vignetting gain map extra input for the current
    /// pipeline size (post-demosaic dims `h/2 × w/2`). Must run BEFORE
    /// `resizeSession` so the Mul broadcast check sees consistent shapes.
    #[cfg(feature = "mnn")]
    fn write_vignetting_gain(
        pool: &[(String, crate::mnn_sys::MnnTensorSafe)],
        interp: &crate::mnn_sys::MnnInterpreterSafe,
        sess: &crate::mnn_sys::MnnSessionSafe,
        h: u32,
        w: u32,
    ) {
        let Some(t) = pool
            .iter()
            .find(|(n, _)| n == "vignetting/gain_map")
            .map(|(_, t)| t)
        else {
            return;
        };
        let gh = (h / 2).max(1);
        let gw = (w / 2).max(1);
        let shape = vec![1i32, 1, gh as i32, gw as i32];
        let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &shape);
        // Strength/falloff must match VignettingBlock::new_default (0.5 / 2.0).
        let gain = crate::blocks::VignettingBlock::gain_map(gh, gw, 0.5, 2.0);
        if let Some(bytes) = t.as_bytes_mut() {
            let byte_count = gain.len() * 4;
            if bytes.len() >= byte_count {
                let src =
                    unsafe { std::slice::from_raw_parts(gain.as_ptr() as *const u8, byte_count) };
                bytes[..byte_count].copy_from_slice(src);
            }
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
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
    fn controller(&self) -> &Mutex<IspController> {
        &self.controller
    }

    fn build(
        &mut self,
        head: Box<dyn IspBlock>,
        aux: Vec<Box<dyn IspBlock>>,
        _warp: Option<Box<dyn IspBlock>>,
        opset: i64,
    ) -> crate::error::IspResult<()> {
        info!("MNN build backend={}", self.backend.id());

        #[cfg(feature = "mnn")]
        {
            use crate::mnn_converter::{convert_onnx_buffer, MnnConvertOptions};
            use std::path::Path;

            let mnn_bytes: Vec<u8> = match &self.model_path {
                Some(p) => {
                    info!(
                        "build [tid={:?}]: reading pre-converted .mnn from {}",
                        std::thread::current().id(),
                        p
                    );
                    // The pre-converted file may declare several inputs (frame +
                    // runtime-fed weights); record the frame tensor name so the
                    // engine never relies on getSessionInput(nullptr) ordering.
                    self.frame_input_name = head.graph_input_name().map(str::to_string);
                    std::fs::read(p).map_err(|e| {
                        error!("Failed to read MNN model: {}", p);
                        crate::error::IspError::Io(format!("read {}: {}", p, e))
                    })?
                }
                None => {
                    // Build ONNX graph: head + aux as pipeline, stats as aux_blocks
                    // But we don't know which aux are stats — pass all as pipeline for now
                    // The head block's graph input name is the model's frame input
                    // tensor; multi-input models (runtime-fed weights) need it
                    // looked up by name, never via getSessionInput(nullptr).
                    self.frame_input_name = head.graph_input_name().map(str::to_string);
                    let mut all: Vec<Box<dyn IspBlock>> = vec![head];
                    all.extend(aux);
                    let refs: Vec<&dyn IspBlock> = all.iter().map(|b| b.as_ref()).collect();
                    info!(
                        "build [tid={:?}]: composing ONNX from {} blocks",
                        std::thread::current().id(),
                        refs.len()
                    );
                    let onnx = crate::pipeline::GraphComposer::compose_from_vec(&refs, &[], opset)?;
                    info!(
                        "ONNX [tid={:?}]: {} bytes for MNN conversion",
                        std::thread::current().id(),
                        onnx.len()
                    );

                    // Save ONNX for inspection (debug only)
                    if cfg!(debug_assertions) {
                        let _ = std::fs::write(".mnn_last_pipeline.onnx", &onnx);
                    }

                    let opts = MnnConvertOptions {
                        preserve_input_type: self.preserve_input_type,
                        allow_custom_op: true,
                        ..Default::default()
                    };
                    info!(
                        "preserve_input_type: {}, optimize_level: {}",
                        self.preserve_input_type, opts.optimize_level
                    );

                    // In-process buffer conversion — zero disk writes
                    convert_onnx_buffer(&onnx).map_err(|e| {
                        crate::error::IspError::Conversion(format!("convert: {}", e))
                    })?
                }
            };

            // Preload GPU backend libraries to register them with MNN.
            // On real Android (not Termux), the proprietary Adreno/Mali driver
            // will be accessible and Vulkan/OpenCL sessions will initialize.
            // On Termux, GPU libs load but VulkanDevice init throws
            // length_error("vector") due to Android linker namespace isolation.
            unsafe {
                let vk = libloading::os::unix::Library::new("libMNN_Vulkan.so");
                if let Ok(lib) = vk {
                    std::mem::forget(lib);
                }
                let cl = libloading::os::unix::Library::new("libMNN_CL.so");
                if let Ok(lib) = cl {
                    std::mem::forget(lib);
                }
                let gl = libloading::os::unix::Library::new("libMNN_GL.so");
                if let Ok(lib) = gl {
                    std::mem::forget(lib);
                }
            }

            if mnn_bytes.is_empty() {
                error!("MNN model bytes are empty");
                return Err(crate::error::IspError::Mnn("MNN model bytes empty".into()));
            }
            let interp = MnnInterpreterSafe::from_buffer(&mnn_bytes).ok_or_else(|| {
                error!(
                    "Failed to load MNN model from buffer ({} bytes)",
                    mnn_bytes.len()
                );
                crate::error::IspError::Mnn(format!("load fail ({} bytes)", mnn_bytes.len()))
            })?;

            // Use first session to probe model input type (before building pool)
            let probe_sess = interp
                .create_session(self.backend.to_sys(), 4)
                .or_else(|| {
                    // Fallback: try CPU if requested backend fails
                    if self.backend != MnnBackend::Cpu {
                        warn!(
                            "Backend {:?} unavailable, falling back to CPU",
                            self.backend
                        );
                        self.backend = MnnBackend::Cpu;
                        interp.create_session(MnnBackendType::Cpu, 4)
                    } else {
                        None
                    }
                })
                .ok_or(crate::error::IspError::Mnn(
                    "probe session create fail (all backends exhausted)".into(),
                ))?;
            // Resolve the frame input by NAME: getSessionInput(nullptr) returns
            // the alphabetically-first input, which is a runtime-fed weight
            // tensor in multi-input models (e.g. CcmBlock_ccm/matrix <
            // RawInputPackedBlock/frame), and reading ITS dims would misclassify
            // the input convention.
            let frame_t = interp
                .get_input(&probe_sess, self.frame_input_name.as_deref().unwrap_or(""))
                .or_else(|| interp.get_first_input(&probe_sess));
            let (input_code, input_bits, input_ndim, input_dims) = match &frame_t {
                Some(t) => {
                    let code = t.data_type();
                    let dims = t.shape();
                    let ndim = dims.len();
                    let mut dims4 = [0i32; 4];
                    for (i, d) in dims.iter().take(4).enumerate() {
                        dims4[i] = *d;
                    }
                    // data_type() returns 4=float32/5=int32/6=uint32; map back
                    // to halide codes (2=float/1=int/3=uint) so
                    // model_input_type stays compatible with the FFI buffer
                    // type codes used by mnn_run_with_output.
                    let halide_code = match code {
                        4 => 2, // halide_type_float
                        5 => 1, // halide_type_int
                        6 => 3, // halide_type_uint
                        _ => code,
                    };
                    (halide_code, 32, ndim, dims4)
                }
                None => (0, 0, 0, [0; 4]),
            };
            self.model_input_type = if input_code != 0 {
                Some((input_code, input_bits))
            } else {
                None
            };
            let is_float = input_code == 2; // halide_type_float
            let expected_elems: i64 = input_dims[..input_ndim].iter().map(|&d| d as i64).product();
            self.expected_input_elements = if expected_elems > 0 {
                Some(expected_elems as u32)
            } else {
                None
            };
            // Infer the input convention from the converted model: channels==2
            // means the engine must split u16 pairs into even/odd lanes
            // ([1,2,H,W/2]); the element type (INT32 stays INT32, INT16-declared
            // inputs are upcast by the converter to FLOAT32) selects i32 vs f32
            // lanes. Raw 1-channel models are fed directly.
            let split = input_ndim == 4 && input_dims[1] == 2;
            let mode = if split {
                if is_float {
                    InputMode::SplitFloat
                } else {
                    InputMode::SplitInt32
                }
            } else if is_float {
                InputMode::RawFloat
            } else {
                InputMode::RawInt
            };
            self.input_mode = Some(mode);
            info!(
                "MNN model input type: code={}, bits={} mode={:?} ndim={} dims={:?} expected_elems={}",
                input_code, input_bits, mode, input_ndim, input_dims, expected_elems
            );
            // Don't need probe session any more — pool will create its own.
            drop(probe_sess);

            // Build session pool (N sessions for parallel inference)
            // Backend was already validated by probe session above.
            let pool = SessionPool::new(interp, self.backend.to_sys(), self.pool_size.max(1), 4)?;
            info!(
                "MNN engine loaded ({} bytes, backend={:?}, {} sessions)",
                mnn_bytes.len(),
                self.backend,
                self.pool_size
            );
            self.pool = Some(pool);
        }

        #[cfg(not(feature = "mnn"))]
        {
            let _ = (head, aux, opset);
        }
        self.initialized = true;
        Ok(())
    }

    fn process(&self, p: &ProcessParams) -> crate::error::IspResult<IspFrame> {
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

        // If isp_params is available, override individual fields
        // This allows per-frame controller params to flow through
        let (override_ccm, override_tone, override_bayer) = if let Some(ref params) = p.isp_params {
            let tone = cam_types::ToneParams {
                contrast: params.tone.contrast,
                brightness: params.tone.brightness,
                gamma_recip: params.tone.gamma,
                ..Default::default()
            };
            (
                Some(&params.ccm.matrix),
                Some(tone),
                Some(&[params.wb.r, params.wb.g, params.wb.g, params.wb.b]),
            )
        } else {
            (None, None, None)
        };
        let ccm = override_ccm.or(ccm);
        let tone = override_tone.as_ref().unwrap_or(tone);
        let bayer = override_bayer.or(bayer);

        if !self.initialized {
            return Err(crate::error::IspError::Config("not init".into()));
        }

        #[cfg(feature = "mnn")]
        {
            let pool = self
                .pool
                .as_ref()
                .ok_or(crate::error::IspError::Config("no pool".into()))?;
            let slot = pool.acquire();
            let sess = &slot.sess;
            let interp = &pool.interp;

            let t_start = Instant::now();

            debug!(
                "pipeline stage=arrive raw={}×{} bayer={}B sensor_max={}",
                w,
                h,
                buf.len(),
                smax
            );

            // ── Pre-allocate output buffer (zero-copy target) ──
            // MNN writes directly into this Vec via host pointer.
            // We return it as frame.data — no memcpy at all.
            let max_out = (tw * h * 4) as usize; // max f32 elements: H×W×4
            let mut out_bytes: Vec<u8> = vec![0u8; max_out * 4];
            let out_ptr = out_bytes.as_mut_ptr() as *mut f32;

            // Determine inference path based on the model's input convention
            // (inferred at build() from the converted model's shape + type).
            let mode = self.input_mode.unwrap_or(InputMode::RawInt);
            let split = matches!(mode, InputMode::SplitInt32 | InputMode::SplitFloat);

            // ── Pre-inference: set input shape and extra inputs ──
            //
            // set_shape() is cheap (just updates a shape descriptor). We
            // always call it so that each session's input tensor has the
            // correct dimensions even on first use after pool rotation.
            //
            // sess.resize() (Interpreter::resizeSession) is EXPENSIVE on
            // Vulkan (~7ms): it propagates shapes through the graph and
            // re-allocates GPU buffers / rebuilds descriptor sets.
            // MNN's own mnn_run_with_output (mnn_wrapper.cpp:412) already
            // calls resizeSession on-demand if the element count changed,
            // so we skip our own call when the shape hasn't changed.
            let t_tensor_before: std::time::Instant = Instant::now();
            let shape = if split {
                vec![1, 2, h as i32, (w / 2).max(1) as i32]
            } else {
                vec![1, 1, h as i32, w as i32]
            };
            // Always set shape (cheap) so C++ on-demand resize works.
            // Resolve the frame input by NAME (never getSessionInput(nullptr),
            // which is the alphabetically-first input in multi-input models).
            let frame_t = interp
                .get_input(sess, self.frame_input_name.as_deref().unwrap_or(""))
                .or_else(|| interp.get_first_input(sess));
            if let Some(t) = frame_t {
                let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &shape);
            }
            // Skip resize if shape unchanged from last full setup.
            // The C++ wrapper handles first-use resize per session.
            let needs_resize = {
                let shape_key = (h as i32, w as i32, mode);
                let mut cached = self.last_input_shape.lock().unwrap();
                let needs = cached.as_ref().is_none_or(|last| *last != shape_key);
                if needs {
                    // Vignetting gain map is a runtime extra input (not a baked
                    // initializer): resize it to the post-demosaic dims (h/2 × w/2)
                    // and fill with computed gains BEFORE resizeSession so the Mul
                    // shape check passes during propagation. A baked [1,1,H,W] gain
                    // would break resizeSession for any input size differing from
                    // the model's baked dims.
                    Self::write_vignetting_gain(&slot.tensor_pool, interp, sess, h, w);
                    let _ = sess.resize();
                    *cached = Some(shape_key);
                }
                needs
            };
            Self::set_extra_inputs(
                &slot.tensor_pool,
                ccm,
                tone,
                bayer,
                awb,
                bayer_pattern,
                p.isp_params.as_ref(),
                p.warp_grid,
                p.warp_shading,
            );
            let t_tensor_after: std::time::Instant = Instant::now();
            debug!(
                "pipeline stage=tensor_assign needs_resize={} elapsed={:?}",
                needs_resize,
                t_tensor_after.duration_since(t_tensor_before)
            );

            debug!(
                "MNN process: w={}, h={}, packed_w={}, mode={:?}, expected={:?}",
                w,
                h,
                (w / 2).max(1),
                mode,
                self.expected_input_elements
            );

            // ── Engine-side input conversion ──
            //
            // The even/odd lane split happens on CPU, not in-graph: MNN's
            // Vulkan backend cannot execute integer elementwise ops
            // (Div/Mod/Sub) or INT16 casts, so extracting two u16 lanes from a
            // packed INT32 in-graph is impossible on GPU. Splitting on the host
            // is exact and signedness-free; the graph only needs a Vulkan-safe
            // Cast INT32→FLOAT32 (UnpackBlock). For INT16-declared models the
            // converter upcasts the input to FLOAT32, so the engine converts
            // the u16 pairs to f32 lanes instead (also exact).
            //
            // The guards are held through inference: the FFI call reads the
            // scratch by raw pointer, and another thread resizing the scratch
            // would otherwise dangle it.
            let split_scratch_i32: Option<std::sync::MutexGuard<'_, Vec<i32>>>;
            let scratch_f32: Option<std::sync::MutexGuard<'_, Vec<f32>>>;
            match mode {
                InputMode::SplitInt32 => {
                    let pairs = ((w / 2).max(1) as usize) * h as usize;
                    let mut guard = self.split_scratch.lock().unwrap();
                    if guard.len() != pairs * 2 {
                        guard.resize(pairs * 2, 0);
                    }
                    let u16s: &[u16] = unsafe {
                        std::slice::from_raw_parts(buf.as_ptr() as *const u16, buf.len() / 2)
                    };
                    for i in 0..pairs {
                        guard[i] = u16s[2 * i] as i32;
                        guard[pairs + i] = u16s[2 * i + 1] as i32;
                    }
                    split_scratch_i32 = Some(guard);
                    scratch_f32 = None;
                }
                InputMode::SplitFloat => {
                    let pairs = ((w / 2).max(1) as usize) * h as usize;
                    let mut guard = self.scratch_f32.lock().unwrap();
                    if guard.len() != pairs * 2 {
                        guard.resize(pairs * 2, 0.0);
                    }
                    let u16s: &[u16] = unsafe {
                        std::slice::from_raw_parts(buf.as_ptr() as *const u16, buf.len() / 2)
                    };
                    for i in 0..pairs {
                        guard[i] = u16s[2 * i] as f32;
                        guard[pairs + i] = u16s[2 * i + 1] as f32;
                    }
                    split_scratch_i32 = None;
                    scratch_f32 = Some(guard);
                }
                InputMode::RawFloat => {
                    let total = (w as usize) * (h as usize);
                    let mut guard = self.scratch_f32.lock().unwrap();
                    if guard.len() != total {
                        guard.resize(total, 0.0);
                    }
                    let u16s: &[u16] = unsafe {
                        std::slice::from_raw_parts(buf.as_ptr() as *const u16, buf.len() / 2)
                    };
                    for i in 0..total {
                        guard[i] = u16s[i] as f32;
                    }
                    split_scratch_i32 = None;
                    scratch_f32 = Some(guard);
                }
                InputMode::RawInt => {
                    split_scratch_i32 = None;
                    scratch_f32 = None;
                }
            }

            let (buffer_ptr, buffer_type_code, buffer_type_bits, input_shape, path_str) = match mode
            {
                InputMode::SplitInt32 => {
                    let v = split_scratch_i32.as_ref().unwrap();
                    (
                        v.as_ptr() as *const c_void,
                        0, // INT32
                        32,
                        vec![1, 2, h as i32, (w / 2).max(1) as i32],
                        "packed_split",
                    )
                }
                InputMode::SplitFloat => {
                    let v = scratch_f32.as_ref().unwrap();
                    (
                        v.as_ptr() as *const c_void,
                        2, // FLOAT32
                        32,
                        vec![1, 2, h as i32, (w / 2).max(1) as i32],
                        "split_f32",
                    )
                }
                InputMode::RawFloat => {
                    let v = scratch_f32.as_ref().unwrap();
                    (
                        v.as_ptr() as *const c_void,
                        2, // FLOAT32
                        32,
                        vec![1, 1, h as i32, w as i32],
                        "raw_f32",
                    )
                }
                InputMode::RawInt => {
                    let raw_shape = [1, 1, h as i32, w as i32];
                    let (code, bits) = self.model_input_type.unwrap_or((0, 16));
                    (
                        buf.as_ptr() as *const c_void,
                        code,
                        bits,
                        raw_shape.to_vec(),
                        "raw_zero_copy",
                    )
                }
            };

            debug!(
                "pipeline stage=write_input buf={}B -> mode={:?} shape=[1,{},{},{}]",
                buf.len(),
                mode,
                input_shape[1],
                input_shape[2],
                input_shape[3]
            );

            let path: &str = path_str;
            let t_prep_end: std::time::Instant = Instant::now();
            let t_infer_start: std::time::Instant = Instant::now();
            let n: i32;
            // Frame input tensor name for the FFI call (None → nullptr for
            // single-input models; MNN then picks its first input).
            let frame_name_c = self
                .frame_input_name
                .as_deref()
                .map(std::ffi::CString::new)
                .transpose()
                .ok()
                .flatten();
            let frame_name_ptr = frame_name_c
                .as_ref()
                .map_or(std::ptr::null(), |c| c.as_ptr());
            unsafe {
                n = crate::mnn_sys::mnn_run_with_output(
                    interp.as_ptr(),
                    sess.as_ptr(),
                    frame_name_ptr,
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
                    debug!("MNN profiling info:\n{}", info);
                }
                // ------------------------------------------------------------------
            }

            let t_total_elapsed = t_start.elapsed();
            let t_infer_elapsed = t_infer_start.elapsed();

            if n <= 0 {
                error!("MNN inference failed: n={} (input={}x{}, output_name='DisplayBlock/frame', path={})", n, w, h, path);
                error!("  This usually means the MNN IspChainFusion pass computed wrong output dimensions.");
                error!("  Check that RawInputBlock has concrete_h and concrete_w set (not -1).");
                return Err(crate::error::IspError::Mnn(format!(
                    "MNN inference failed: n={} (input={}x{})",
                    n, w, h
                )));
            }

            debug!("pipeline stage=infer_done path={} total={:?} ({}x{} -> {} elts) prep={:?} infer={:?}",
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
            unsafe {
                out_bytes.set_len(n_bytes_actual);
            }

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
                let mut zone_data: Option<ZoneData> = None;

                /// Read a named output tensor and reinterpret bytes as `f32`.
                /// Returns up to `max_n` floats, or `None` if the tensor is missing.
                fn read_output_f32(
                    interp: &crate::mnn_sys::MnnInterpreterSafe,
                    sess: &crate::mnn_sys::MnnSessionSafe,
                    name: &str,
                    max_n: usize,
                ) -> Option<Vec<f32>> {
                    let t = interp.get_output(sess, name)?;
                    let bytes = t.as_bytes()?;
                    let n_floats = bytes.len() / 4;
                    let n = n_floats.min(max_n);
                    let floats: &[f32] = unsafe {
                        std::slice::from_raw_parts(bytes.as_ptr() as *const f32, n_floats)
                    };
                    Some(floats[..n].to_vec())
                }

                // ChannelMeansBlock/frame → [1, 3] or [3]
                if let Some(vals) = read_output_f32(interp, sess, "ChannelMeansBlock/frame", 3) {
                    if vals.len() >= 3 {
                        cm_vals = Some([vals[0], vals[1], vals[2]]);
                    }
                }
                // ToneStatsBlock/frame → [6]
                if let Some(vals) = read_output_f32(interp, sess, "ToneStatsBlock/frame", 6) {
                    let mut ts = [0.0f32; 6];
                    let n = vals.len().min(6);
                    ts[..n].copy_from_slice(&vals[..n]);
                    ts_vals = Some(ts);
                }
                // CoarseHistogramBlock/frame → [1, 16]
                if let Some(vals) = read_output_f32(interp, sess, "CoarseHistogramBlock/frame", 16)
                {
                    let mut hist = [0.0f32; 16];
                    let n = vals.len().min(16);
                    hist[..n].copy_from_slice(&vals[..n]);
                    hist_vals = Some(hist);
                }
                // CalibrationBlock/frame → [24] (quad means, vars, mins, maxs, ranges, frame stats)
                if let Some(vals) = read_output_f32(interp, sess, "CalibrationBlock/frame", 24) {
                    let mut cal = [0.0f32; 24];
                    let n = vals.len().min(24);
                    cal[..n].copy_from_slice(&vals[..n]);
                    calib_vals = Some(cal);
                }

                // ZoneStatsBlock/frame → [1, 3, rows, cols]
                if let Some(t) = interp.get_output(sess, "ZoneStatsBlock/frame") {
                    if let Some(bytes) = t.as_bytes() {
                        let floats: &[f32] = unsafe {
                            std::slice::from_raw_parts(
                                bytes.as_ptr() as *const f32,
                                bytes.len() / 4,
                            )
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
                        && zone_data
                            .as_ref()
                            .map(|(r, c, _)| *r > 0 && *c > 0)
                            .unwrap_or(false);

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

            debug!("pipeline stage=output_frame {}×{} frame={}B total={:?} prep={:?} resize={:?} infer={:?}",
                tw, oh, data_out.len(), t_total_elapsed,
                t_prep_end - t_start,
                t_tensor_after - t_tensor_before,
                t_infer_elapsed);
            let total_duration_ns = t_total_elapsed.as_nanos() as u64;
            let mut frame = IspFrame::new(tw, oh as u32, FrameFormat::Rgba8888);
            frame.data = data_out;
            // Propagate controller params from ProcessParams to output frame
            frame.params = p.isp_params.clone().unwrap_or_default();
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
            Ok(frame)
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
        cam_isp::engine::register_engine(cam_isp::engine::EngineFactory {
            name: $backend.id(),
            priority: $backend.priority(),
            create_fn: Box::new(|| Box::new(cam_isp::mnnengine::MnnEngine::new($backend))),
        });
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Returns true if running in CI (GitHub Actions, etc.).
    fn is_ci() -> bool {
        std::env::var("CI").is_ok() || std::env::var("GITHUB_ACTIONS").is_ok()
    }

    #[test]
    fn test_mnn_backend_id() {
        assert_eq!(MnnBackend::Vulkan.id(), "mnn_vulkan");
        assert_eq!(MnnBackend::Cpu.id(), "mnn_cpu");
    }

    #[test]
    fn test_mnn_backend_priority() {
        assert!(MnnBackend::Vulkan.priority() > MnnBackend::Cpu.priority());
    }

    #[test]
    fn test_mnn_engine_new() {
        let engine = MnnEngine::new(MnnBackend::Cpu);
        assert_eq!(engine.backend_name(), "mnn_cpu");
    }

    #[test]
    fn test_mnn_engine_with_pool_size() {
        let engine = MnnEngine::with_pool_size(MnnBackend::Cpu, 4);
        assert_eq!(engine.backend_name(), "mnn_cpu");
    }

    #[test]
    fn test_preserve_input_type() {
        let mut engine = MnnEngine::new(MnnBackend::Cpu);
        assert!(!engine.preserve_input_type());
        engine.set_preserve_input_type(true);
        assert!(engine.preserve_input_type());
    }

    #[test]
    fn test_query_optimal_workgroup() {
        if is_ci() {
            eprintln!("Skipping on CI — requires MNN Vulkan runtime");
            return;
        }
        let (wx, wy) = MnnEngine::query_optimal_workgroup();
        assert!(wx > 0 && wy > 0);
    }
}
