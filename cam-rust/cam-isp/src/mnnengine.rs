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

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use log::{debug, error, info, warn};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Instant;

/// The controller fields the graph is fed from, captured once per frame.
///
/// Taken as a snapshot so the controller lock is released before the encode —
/// holding it across inference would serialise every frame against the 3A maths.
struct ControllerState {
    awb: [f32; 3],
    ccm: [f32; 9],
    tone: cam_types::ToneParams,
    frames: u32,
    /// Auto-exposure gain, already including the histogram constraint.
    exposure_gain: f32,
}

/// Set once the graph's channel count disagrees with the requested format.
///
/// The condition holds for every frame, so it is reported exactly once.
static CHANNEL_MISMATCH_REPORTED: AtomicBool = AtomicBool::new(false);

/// Zone data: (rows, cols, zone_rgb`[r]``[c]` = `[f32; 3]`)
type ZoneData = (usize, usize, Vec<Vec<[f32; 3]>>);

use crate::controller::IspController;
use crate::engine::{IspEngine, OutputFormat, ProcessParams};
use crate::pipeline::{ExternalBuffer, IspAuxOutput, IspBlock, IspFrame};
use cam_types::{FrameFormat, ToneParams};

#[cfg(feature = "mnn")]
pub use crate::mnn_sys::*;

// BinaryAddToEltwise merge pass registration is provided as a strong
// C++ definition in mnn_sys/mnn_vulkan_stubs.cpp (with visibility("default"))
// so the dynamic linker can resolve the mangled symbol
// `_Z34ensureBinaryAddToEltwiseRegisteredv` for MNNConvertDeps at load time.
// See that file for details; no Rust-side declaration is needed because
// nothing in the Rust code calls the pass directly.

// ── Backend ─────────────────────────────────────────────────────────────

/// The MNN inference backend. Only Vulkan is supported: the whole pipeline
/// (GDC grid compute + GridSample + fused isp.* extras) runs on GPU via
/// MNN's Vulkan backend; CPU/OpenCL/OpenGL backends are intentionally not
/// offered so the runtime contract stays unambiguous.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MnnBackend {
    Vulkan,
}

impl MnnBackend {
    pub fn id(&self) -> &'static str {
        "mnn_vulkan"
    }
    pub fn priority(&self) -> i32 {
        99
    }
    #[cfg(feature = "mnn")]
    fn to_sys(self) -> MnnBackendType {
        MnnBackendType::Vulkan
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
    /// `[1,2,H,W/2]` INT16 — packed u16 pairs split into even/odd **i16**
    /// lanes on CPU. The graph input was declared INT16 and MNN kept the
    /// width, so the split stays 16-bit: half the scratch memory of
    /// [`Self::SplitInt32`] and no widening pass.
    SplitInt16,
    /// `[1,2,H,W/2]` FLOAT32 — the same two lanes, widened to f32 on the host.
    ///
    /// Reached only when the converted model really declares a float input with
    /// two channels. The width the graph ends up with is decided by the block's
    /// ONNX `value_info` (INT16 stays INT16 / INT32 stays INT32), not by the
    /// converter widening anything, so a two-lane float model is a deliberate
    /// declaration rather than an accidental upcast.
    SplitFloat,
    /// `[1,1,H,W]` FLOAT32 — native int16 buffer converted to f32 on CPU
    /// (unpack_blc16 / UnpackCfa-style heads).
    RawFloat,
    /// `[1,1,H,W]` integer — caller buffer passed through as-is, at the
    /// width the graph declared (INT16 or INT32).
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
    /// Store the graph's float tensors as fp16 on the GPU when the device
    /// supports it (`Precision_Normal` instead of `Precision_High`).
    ///
    /// Every tensor in the ISP chain is float, and the Vulkan backend allocates
    /// 4 bytes per element when this is off and 2 when it is on. At 2592x1940
    /// the RGBA display tensor alone is 77 MB -> 38 MB per session, which is
    /// what makes a multi-session pool fit on a phone at all.
    fp16_storage: bool,
    /// Whether the graph divides by `sensor_max` at its head (a
    /// `NormalizeBlock`). Defaults to true, the shape of every profile that
    /// keeps the block.
    normalizes_at_input: bool,
    /// Measure the 3A statistics every N frames instead of every frame.
    ///
    /// Every statistics tensor is a full-frame reduction followed by a
    /// host-side readback, and the readback is a GPU stall. The controller
    /// smooths these numbers anyway, so a per-frame reduction buys accuracy
    /// nobody uses — AWB at 1/8 the frame rate is indistinguishable in the
    /// picture. `1` means every frame (the old behaviour).
    stats_interval: u32,
    /// Frames since the last measurement. `0` means the next frame measures.
    stats_countdown: AtomicU32,
    /// Whether to fetch the statistics at all.
    ///
    /// Read per frame, so a caller can turn it off for a burst of frames without
    /// rebuilding the session. Atomic rather than a plain bool because the camera
    /// thread owns the engine and a control thread flips this.
    stats_enabled: AtomicBool,
    /// Expected input tensor element count from model (for validation).
    expected_input_elements: Option<u32>,
    /// ISP controller for AWB/AE/CCM/tone parameter estimation.
    /// After each inference, stats output tensors are read and fed
    /// into the controller to update state for the next frame.
    ///
    /// Shared by both workers in dual-engine mode: one engine measures stats
    /// and advances the controller, the other only reads its tuned params.
    pub controller: Arc<Mutex<IspController>>,
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
    /// Reused even/odd lane-split buffer for the packed INT32 input path.
    /// MNN's Vulkan backend cannot run integer elementwise ops, so the
    /// engine splits the two u16 lanes on CPU before feeding `[1,2,H,W/2]`.
    split_scratch: Mutex<Vec<i32>>,
    /// Same split, 16-bit lanes, for a graph whose input stayed INT16.
    ///
    /// The model's declared width is not always 32: when the converter keeps
    /// INT16 the tensor is half the size, so widening to i32 would double the
    /// dominant per-frame buffer for no reason.
    split_scratch_i16: Mutex<Vec<i16>>,
    /// Reused f32 scratch: u16→f32 lane conversion for native-int16 models
    /// (SplitFloat) and full-frame conversion for raw FLOAT32 models (RawFloat).
    scratch_f32: Mutex<Vec<f32>>,
    /// Cached default values for extra inputs (formerly baked as initializers).
    /// Populated at build() from each block's `extra_input_defaults()`.
    #[cfg(feature = "mnn")]
    default_inputs: Mutex<Vec<(String, Vec<u8>)>>,
    /// Names whose defaults are not session inputs in this graph. Logged once
    /// per build; emitting a warning every frame costs more than the upload it
    /// describes and hides the tensors that actually carry the value.
    #[cfg(feature = "mnn")]
    missing_inputs_reported: Mutex<HashSet<String>>,
    /// Whether the chain contains fused `isp.display_*` ops.
    /// Computed at build() and used in process() for output decoding.
    #[cfg(feature = "mnn")]
    has_fused_display: bool,
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
            frame_input_name: None,
            input_mode: None,
            preserve_input_type: false,
            fp16_storage: true,
            normalizes_at_input: true,
            stats_interval: 1,
            stats_countdown: AtomicU32::new(0),
            stats_enabled: AtomicBool::new(true),
            expected_input_elements: None,
            controller: Arc::new(Mutex::new(IspController::new())),
            pool_size,
            #[cfg(feature = "mnn")]
            pool: None,
            #[cfg(feature = "mnn")]
            buf_pool: Mutex::new(crate::mnn_buffer::OutputBufferPool::new(3, 1)),
            watchdog: crate::gpu_watchdog::GpuWatchdog::new(
                crate::gpu_watchdog::WatchdogConfig::default(),
            ),
            split_scratch: Mutex::new(Vec::new()),
            split_scratch_i16: Mutex::new(Vec::new()),
            scratch_f32: Mutex::new(Vec::new()),
            #[cfg(feature = "mnn")]
            default_inputs: Mutex::new(Vec::new()),
            #[cfg(feature = "mnn")]
            missing_inputs_reported: Mutex::new(HashSet::new()),
            #[cfg(feature = "mnn")]
            has_fused_display: false,
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

    /// Replace this engine's ISP controller with `ctrl`.
    ///
    /// Prefer [`Self::share_controller`] when two engines must see the same
    /// AWB/AE/CCM/tone state: this method takes ownership and no other engine
    /// can hold the same controller.
    pub fn set_controller(&mut self, ctrl: Mutex<IspController>) {
        self.controller = Arc::new(ctrl);
    }

    /// Point this engine at a controller shared with another engine.
    ///
    /// Dual-engine mode has one worker measure 3A statistics and advance the
    /// controller while the other skips the readback and still consumes the
    /// tuned params. Only the controller state is shared; each engine owns its
    /// sessions and GPU buffers.
    pub fn share_controller(&mut self, ctrl: &Arc<Mutex<IspController>>) {
        self.controller = Arc::clone(ctrl);
    }

    /// Clone this engine's controller handle, for wiring a second engine.
    pub fn controller_handle(&self) -> Arc<Mutex<IspController>> {
        Arc::clone(&self.controller)
    }

    /// Choose fp16 (`true`, the default) or fp32 (`false`) tensor storage.
    ///
    /// Applies to the whole graph, not just the frame: the Vulkan backend's
    /// `getTensorSize` halves every float tensor's allocation when precision is
    /// not `Precision_High`. Turn it off only to compare output numerics; it is
    /// what keeps a 4-session pool inside a phone's GPU memory budget.
    pub fn set_fp16_storage(&mut self, fp16: bool) {
        self.fp16_storage = fp16;
    }

    /// Whether fp16 tensor storage is requested.
    pub fn fp16_storage(&self) -> bool {
        self.fp16_storage
    }

    /// Measure the 3A statistics every `n` frames instead of every frame.
    ///
    /// `n <= 1` restores the per-frame behaviour. The statistics are a
    /// full-frame reduction plus a host readback (a GPU stall), and the
    /// controller smooths them — at 1/8 the frame rate the picture is the same.
    /// A frame that skips measurement reports `stats_measured == false` and
    /// carries no statistics, so a consumer can tell "nothing new" from "the
    /// readback failed".
    pub fn set_stats_interval(&mut self, n: u32) {
        self.stats_interval = n.max(1);
        self.stats_countdown.store(0, Ordering::Relaxed);
    }

    /// How often the statistics are measured, in frames.
    pub fn stats_interval(&self) -> u32 {
        self.stats_interval
    }

    /// Turn the 3A statistics on or off at run time.
    ///
    /// Off means no readback at all: every tensor the statistics produce stays on
    /// the device, nothing is copied to the host, and the controller is not moved.
    /// The graph still contains the statistics nodes — they run, their results are
    /// simply never fetched — so switching back on needs no rebuild and no
    /// re-conversion.
    ///
    /// A frame produced while this is off reports
    /// `IspAuxOutput::stats_measured == false` and carries no statistics, which is
    /// what a consumer uses to skip its own work instead of reading a tensor that
    /// was never fetched.
    ///
    /// Default is **on**, because a pipeline whose controller never receives
    /// feedback is not doing 3A at all and that should be a deliberate choice.
    pub fn set_stats_enabled(&self, on: bool) {
        self.stats_enabled.store(on, Ordering::Relaxed);
    }

    /// Whether the 3A statistics are being measured.
    pub fn stats_enabled(&self) -> bool {
        self.stats_enabled.load(Ordering::Relaxed)
    }

    /// Declare whether the graph divides by `sensor_max` at its head.
    ///
    /// True (the default) when the pipeline carries a `NormalizeBlock`, whose
    /// `max_val` input `set_extra_inputs` writes. False when the pipeline was
    /// built with `PipelineProfile::normalize_at_display`, where the divide is
    /// folded into the display Conv and the engine must therefore scale the
    /// unit-bearing parameters by `sensor_max` itself.
    pub fn set_normalizes_at_input(&mut self, yes: bool) {
        self.normalizes_at_input = yes;
    }

    /// See [`Self::set_normalizes_at_input`].
    pub fn normalizes_at_input(&self) -> bool {
        self.normalizes_at_input
    }

    /// Narrow the graph's `f32 [0,255]` output to `u8` bytes.
    ///
    /// `DisplayBlock` emits every non-`Float*` format as FLOAT: the Conv(1×1)
    /// already applies the 255 scale and the channel permutation, but ONNX has
    /// no way to hand a `u8` consumer its bytes. The narrowing happens here so
    /// the returned buffer is exactly `W*H*channels` bytes, which is what
    /// `WindowTarget::present` and the HAL bridges read.
    ///
    /// Values are clamped to `[0,255]` before truncation: Vulkan FP16 kernels
    /// can overshoot slightly on saturated pixels, and an unclamped cast
    /// wraps negatives to ~255 instead of 0.
    fn f32_to_u8(raw: &[u8]) -> Vec<u8> {
        // A `Vec<u8>` only guarantees byte alignment, so reading it as `[f32]`
        // would be UB. Decode each element by value instead — the compiler
        // lowers `from_le_bytes` to a single unaligned load on aarch64.
        raw.as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b).clamp(0.0, 255.0) as u8)
            .collect()
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

    /// Register the Vulkan MNN engine factory.
    /// Called automatically by `cam_isp::init()`.
    #[cfg(feature = "mnn")]
    pub fn register_factories() {
        // Skip GPU benchmark — rapid-fire inference can crash Vulkan drivers
        // on some devices. Use static priorities instead.
        Self::register_with_defaults();

        // Ensure Vulkan backend is registered (forces MNNVulkanFuseRegister and
        // manual runtime creator registration). This is a no-op if already called.
        unsafe { crate::mnn_sys::ensure_mnn_vulkan_registered() };
    }

    /// Fallback: register with static default priorities (no benchmark).
    #[cfg(feature = "mnn")]
    fn register_with_defaults() {
        use crate::engine::register_engine;
        use crate::engine::EngineFactory;
        // Vulkan is the only supported MNN backend. Register two factory
        // instances so `select_engine_by_name("mnn_vulkan_a")` and
        // `select_engine_by_name("mnn_vulkan_b")` hand out two independent
        // engines for dual-GPU ping-pong.
        let b = MnnBackend::Vulkan;
        let create_fn_a = Box::new(move || Box::new(MnnEngine::new(b)) as Box<dyn IspEngine>);
        let create_fn_b = Box::new(move || Box::new(MnnEngine::new(b)) as Box<dyn IspEngine>);
        register_engine(EngineFactory {
            name: "mnn_vulkan_a",
            priority: b.priority(),
            create_fn: create_fn_a,
        });
        register_engine(EngineFactory {
            name: "mnn_vulkan_b",
            priority: b.priority(),
            create_fn: create_fn_b,
        });
        info!("Registered MNN engine factories mnn_vulkan_a and mnn_vulkan_b (default priority)");
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
        let (chunks, _) = buf.as_chunks::<2>();
        chunks
            .iter()
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
        &self,
        pool: &[(String, crate::mnn_sys::MnnTensorSafe)],
        ccm: Option<&[f32; 9]>,
        tone: &ToneParams,
        bayer: Option<&[f32; 4]>,
        _awb: Option<&[f32; 3]>,
        bayer_pattern: i32,
        isp_params: Option<&crate::isp_params::IspParams>,
        warp_grid: Option<&[f32]>,
        warp_shading: Option<&[f32]>,
        input_scale: f32,
        white_level: f32,
        exposure_gain: f32,
    ) {
        /// Look up a cached tensor handle by name prefix (fast — no CString alloc).
        fn find<'a>(
            pool: &'a [(String, crate::mnn_sys::MnnTensorSafe)],
            name: &str,
        ) -> Option<&'a crate::mnn_sys::MnnTensorSafe> {
            pool.iter().find(|(n, _)| n == name).map(|(_, t)| t)
        }

        // Defaults and per-frame overrides frequently target the same tensor.
        // Queue writes by name and upload each tensor once with its FINAL
        // value: on Vulkan every copyFromHostTensor is a separate host→device
        // transfer with its own barrier, and that per-call latency dominated
        // the frame budget (~60 uploads, ~174 ms/frame at 800x600).
        enum PendingWrite {
            /// Uploaded with an explicit `[1]` shape — MNN reports a rank-0
            /// tensor's shape as empty, so an empty-shape upload carries no
            /// elements.
            Scalar(f32),
            Array(Vec<f32>),
            /// `extra_input_defaults` are serialized f32 values.
            Raw(Vec<u8>),
        }
        type Pending = HashMap<String, PendingWrite>;

        fn write_f32(pending: &mut Pending, name: &str, val: f32) {
            pending.insert(name.to_owned(), PendingWrite::Scalar(val));
        }

        fn write_f32_array(pending: &mut Pending, name: &str, vals: &[f32]) {
            pending.insert(name.to_owned(), PendingWrite::Array(vals.to_vec()));
        }

        fn write_raw(pending: &mut Pending, name: &str, data: &[u8]) {
            pending.insert(name.to_owned(), PendingWrite::Raw(data.to_vec()));
        }

        let mut pending: Pending = HashMap::new();

        // ── Block-provided defaults (formerly baked Const initializers) ──
        // Queue raw tensor bytes from each block's `extra_input_defaults()`.
        // These are queued FIRST so the per-frame writes below take precedence.
        //
        // `input_scale` is 1.0 unless the pipeline deferred the divide: then
        // the frame is still in raw counts, so the display Conv — whose weights
        // carry the [0,1] to [0,255] scaling — must be divided by the same
        // factor or every pixel saturates on the first clamp.
        let defaults = self.default_inputs.lock().unwrap();
        for (name, data) in defaults.iter() {
            if input_scale != 1.0 && name == "DisplayBlock/conv_w" {
                let scaled: Vec<u8> = data
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .flat_map(|b| {
                        (f32::from_le_bytes(*b) / input_scale)
                            .to_le_bytes()
                            .to_vec()
                    })
                    .collect();
                write_raw(&mut pending, name, &scaled);
                continue;
            }
            write_raw(&mut pending, name, data);
        }
        drop(defaults);

        // DemosaicCcmBlock/w [3,4,1,1] — fused CCM × demosaic weights
        //
        // The demosaic operation extracts 2×2 Bayer quadrants into 3 RGB
        // output channels.  The weights depend on the sensor's Bayer pattern:
        //
        //   Pattern 0 (RGGB):  R=[1,0,0,0]  G=[0,½,½,0]  B=[0,0,0,1]
        //   Pattern 1 (GRBG):  R=[0,1,0,0]  G=[½,0,0,½]  B=[0,0,1,0]
        //   Pattern 2 (GBRG):  R=[0,0,1,0]  G=[½,0,0,½]  B=[0,1,0,0]
        //   Pattern 3 (BGGR):  R=[0,0,0,1]  G=[0,½,½,0]  B=[1,0,0,0]
        //
        // fused[i,j] = Σₖ ccm[i,k] * demo[k,j]   (i=RGB, j=quadrant)
        const DEMO_RGGB: [f32; 12] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0];
        const DEMO_BGGR: [f32; 12] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.5, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0];
        const DEMO_GRBG: [f32; 12] = [0.0, 1.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0];
        const DEMO_GBRG: [f32; 12] = [0.0, 0.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0];

        // Select demosaic weights by pattern.
        //
        // The encoding is Android's `SENSOR_INFO_COLOR_FILTER_ARRANGEMENT`:
        // 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR (`cam_camera_app::camera::cfa_to_bayer`).
        // This match previously answered 1->BGGR, 2->GRBG, 3->GBRG, which
        // rotates the three non-RGGB patterns by one — so a GRBG sensor got
        // BGGR weights and swapped red and blue outright. The pair chosen must
        // be the one whose planes hold the colours the table assumes, i.e. the
        // table's own name.
        let demo: &[f32; 12] = match bayer_pattern & 3 {
            1 => &DEMO_GRBG,
            2 => &DEMO_GBRG,
            3 => &DEMO_BGGR,
            _ => &DEMO_RGGB,
        };

        // White balance has several possible sinks. Pick the one that survived
        // conversion from the *session*, not from the profile declaration: a
        // fused demosaic+CCM renames both the demosaic and CCM tensors, while a
        // primitive chain keeps them separate, and either may omit BayerWbBlock.
        //
        //   BayerWbBlock/gains present  -> write gains there, fold nothing.
        //   DemosaicCcmBlock/w present  -> fold gains into the fused 3x4 matrix.
        //   DemosaicBlock/w present     -> fold gains into its 3x4 columns.
        //   CcmBlock_ccm/matrix only    -> fold a per-colour gain into CCM.
        let wb_gains: [f32; 4] = bayer.copied().unwrap_or([1.0; 4]);
        let wb_folded = find(pool, "BayerWbBlock/gains").is_none();
        let fused_ccm_sink = find(pool, "DemosaicCcmBlock/w").is_some();
        let primitive_demosaic_sink = find(pool, "DemosaicBlock/w").is_some();
        let ccm_base: [f32; 9] = ccm
            .copied()
            .unwrap_or([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);

        // Is a separate tone stage present?
        //
        // The demosaic weights below can absorb contrast and brightness, but only
        // when nothing else will apply them. On the fused chain the `ToneBlock`
        // is still there, and it reads `ToneBlock/contrast` /
        // `ToneBlock/brightness` from its own tensors further down — so folding
        // tone into the weights would apply it twice. On a flat grey test frame
        // with contrast 1.0 and brightness 0.0 that is invisible; on a real scene
        // with the controller at contrast 1.3 it multiplies the channels by 1.69
        // and clips.
        let tone_stage_present = find(pool, "ToneBlock/contrast").is_some();

        // Fused CCM weights: w[i,j] = Σₖ ccm[i,k] · demo[k,j] · wb[j].
        // The WB fold belongs here only when this fused tensor is the real sink;
        // when the primitive demosaic survives, folding it below into
        // DemosaicBlock/w keeps the same exact per-plane scale.
        if fused_ccm_sink {
            let mut fused = [0.0f32; 12];
            for i in 0..3 {
                for j in 0..4 {
                    let mut s = 0.0;
                    for k in 0..3 {
                        s += ccm_base[i * 3 + k] * demo[k * 4 + j];
                    }
                    if wb_folded {
                        s *= wb_gains[j];
                    }
                    fused[i * 4 + j] = if tone_stage_present {
                        s
                    } else {
                        s * tone.contrast
                    };
                }
            }
            write_f32_array(&mut pending, "DemosaicCcmBlock/w", &fused);
            // DemosaicCcmBlock/b [3] — CCM bias, plus tone brightness when the
            // tone stage is not there to add it itself.
            let demosaic_bias = if tone_stage_present {
                [0.0f32; 3]
            } else {
                [tone.brightness; 3]
            };
            write_f32_array(&mut pending, "DemosaicCcmBlock/b", &demosaic_bias);
        }

        // ── Coefficient buffers for the fused isp.* ops ──
        //
        // The Extra ops are positional: coefficients arrive as descriptor
        // buffers at bindings 5+, never as node attributes. VulkanFuse reads 12
        // attribute keys and none of them is a coefficient, so a block that
        // emits `kernel_shape`/`blc_offsets`/`rotate_mode` is talking to nobody.
        // These three writes are what actually reach the shaders.
        //
        // EeBlock: depthwise unsharp kernel [3,1,3,3] + bias [3]. Values match
        // `EeBlock::extra_input_defaults` (centre 3.0, orthogonal -0.5, sum 1.0)
        // so the fused path and the primitive Conv agree exactly.
        let ee_kernel: [f32; 27] = [
            0.0, -0.5, 0.0, //
            -0.5, 3.0, -0.5, //
            0.0, -0.5, 0.0,
        ]
        .iter()
        .cycle()
        .take(27)
        .copied()
        .collect::<Vec<f32>>()
        .try_into()
        .unwrap_or([0.0; 27]);
        write_f32_array(&mut pending, "EeBlock/kernel_ee", &ee_kernel);
        write_f32_array(&mut pending, "EeBlock/bias_ee", &[0.0f32; 3]);

        // The fused RAW heads — `isp.unpack_blc_normal_cfa` (packed INT32),
        // `isp.unpack16_blc_normal_cfa` (INT16), `isp.unpack32_blc_normal_cfa`
        // (packed INT32, W/2 words per row) — all read the same three
        // coefficient tensors, each under its own block namespace.
        //
        // The head subtracts black level by *colour* but emits planes in
        // physical quad order, so it needs the phase to know which plane is
        // which. Both are written from `ProcessParams`, not from the block's
        // construction-time defaults, so a profile cannot carry a stale phase or
        // white level into a different sensor.
        //
        // Writing only the packed-form name was a real defect: a profile that
        // fused its head to one of the other two kept the block default of
        // 65535, so a 10-bit sensor's 1023-count frame came out at 0.016 — a
        // nearly black picture at the end of an otherwise correct chain.
        for ns in ["isp_unpack_blc", "isp_unpack16_blc", "isp_unpack32_blc"] {
            // `int bayer_pattern[]` in every one of these shaders — four bytes of
            // int, not four bytes of float bits. Uploading `2.0f` put 0x40000000
            // in the buffer, which matches none of the four phases and fell
            // through to the BGGR `else`, rotating every plane.
            write_raw(
                &mut pending,
                &format!("{ns}/bayer_pattern"),
                &bayer_pattern.to_le_bytes(),
            );
            write_f32(
                &mut pending,
                &format!("{ns}/max_val"),
                if input_scale > 1.0 {
                    input_scale
                } else {
                    white_level
                }
                .max(1.0),
            );
        }

        // DisplayBlock/gamma — `isp.display` computes `pow(x, 1/gamma)` and reads
        // gamma from binding 0. Left at zero the shader's `pow(x, 0)` yields 1.0
        // everywhere, i.e. a white frame. The primitive path uses 1/2.4 as an
        // exponent, so the shader wants its reciprocal.
        write_f32(&mut pending, "DisplayBlock/gamma", 2.4);
        // BayerWbBlock/gains [1,4,1,1] — white balance per-channel gains.
        // Written unconditionally; the flush below drops names this session does
        // not have. When the block is absent the gains are folded into whichever
        // demosaic/CCM tensor exists instead, so 3A feedback still reaches the
        // picture.
        write_f32_array(&mut pending, "BayerWbBlock/gains", &wb_gains);
        // DemosaicBlock/w + b [3,4,1,1]/[3] — production demosaic conv weights
        // (depend on the sensor Bayer pattern). When WB has no block of its own
        // and no fused demosaic+CCM exists, its per-plane scale belongs in these
        // columns: demo_wb[k,j] = demo[k,j] * wb[j]. This is exact even though
        // a colour's row can cover two same-colour CFA sites.
        let mut demo_wb = *demo;
        if wb_folded && !fused_ccm_sink {
            for k in 0..3 {
                for j in 0..4 {
                    demo_wb[k * 4 + j] *= wb_gains[j];
                }
            }
        }
        write_f32_array(&mut pending, "DemosaicBlock/w", &demo_wb);
        write_f32_array(&mut pending, "DemosaicBlock/b", &[0.0f32; 3]);
        // CcmBlock_ccm/matrix [3,3]. Last-resort fold: only when neither
        // demosaic sink exists. Each output colour's rows from the demosaic
        // table touch same-colour sites, so their weighted WB gain is the right
        // per-colour scale for the CCM column.
        let mut ccm_mat = ccm_base;
        if wb_folded && !fused_ccm_sink && !primitive_demosaic_sink {
            for i in 0..3 {
                for k in 0..3 {
                    let mut weighted = 0.0f32;
                    let mut total = 0.0f32;
                    for j in 0..4 {
                        let w = demo[k * 4 + j];
                        weighted += w * wb_gains[j];
                        total += w;
                    }
                    if total > 0.0 {
                        ccm_mat[i * 3 + k] *= weighted / total;
                    }
                }
            }
        }
        write_f32_array(&mut pending, "CcmBlock_ccm/matrix", &ccm_mat);
        // ToneBlock/contrast [1]
        write_f32(&mut pending, "ToneBlock/contrast", tone.contrast);
        // ToneBlock/brightness [1]
        write_f32(&mut pending, "ToneBlock/brightness", tone.brightness);
        // ToneBlock/gamma_recip [1]
        write_f32(&mut pending, "ToneBlock/gamma_recip", tone.gamma_recip);
        // SaturationBlock/scale [3] — per-channel saturation, from the tone
        // controller. Its `tone_saturation` was previously ignored here, so the
        // stage always ran at 1.0 and the controller's tuning never reached the
        // picture.
        let sat = tone.saturation.clamp(0.0, 2.0);
        write_f32_array(&mut pending, "saturation/scale", &[sat; 3]);
        // ToneBlock/exposure [1] — AE gain, applied before contrast/brightness.
        write_f32(&mut pending, "ToneBlock/exposure", exposure_gain.max(0.05));
        // Sharpen/strength [1] — sharpening strength
        write_f32(&mut pending, "Sharpen/strength", 1.0);
        // LdciBlock/strength [1] — local contrast strength
        write_f32(&mut pending, "LdciBlock/strength", 1.0);
        // FcsBlock/gain [3] — per-channel gain (default: identity)
        write_f32_array(&mut pending, "FcsBlock/gain", &[1.0f32; 3]);
        // FcsBlock/bias [3] — per-channel bias (default: zero)
        write_f32_array(&mut pending, "FcsBlock/bias", &[0.0f32; 3]);
        // NormalizeBlock/max_val [1] — the divisor that maps sensor counts to
        // [0,1].
        //
        // This used to be a fixed 65535. A 10-bit sensor reports a white level
        // of 1023, so a fully exposed pixel arrived at 1023/65535 = 0.0156 and
        // the display — whose Conv carries the [0,1]→[0,255] scale — handed
        // back 4/255. Every frame rendered essentially black while every stage
        // reported success, which is exactly what the log showed.
        //
        // `input_scale` is 1.0 when the pipeline defers the divide (no
        // NormalizeBlock then), so `white_level` is the sensor's own scale.
        let divisor = if input_scale > 1.0 {
            input_scale
        } else {
            white_level
        };
        write_f32(&mut pending, "NormalizeBlock/max_val", divisor.max(1.0));
        // GammaBlock / AutoContrastBlock / DisplayBlock — always write
        // defaults so pure-input tensors are never left uninitialized when
        // no isp_params are supplied; the isp_params branch below overrides.
        write_f32(&mut pending, "Gamma/inv_gamma", 1.0 / 2.2);
        write_f32(&mut pending, "Gamma/min", 0.0);
        write_f32(&mut pending, "Gamma/max", 1.0);
        write_f32(&mut pending, "Gamma/lift", 0.0);
        write_f32(&mut pending, "Gamma/norm", 1.0);
        write_f32(&mut pending, "AutoContrast/lift", 0.0);
        write_f32(&mut pending, "AutoContrast/half", 0.5);
        write_f32(&mut pending, "AutoContrast/contrast_w", 1.0);
        write_f32(&mut pending, "AutoContrast/zero", 0.0);
        write_f32(&mut pending, "AutoContrast/one", 1.0);
        write_f32(&mut pending, "DisplayBlock/scale", 1.0);
        write_f32(&mut pending, "DisplayBlock/gamma_exp", 1.0 / 2.4);
        write_f32(&mut pending, "DisplayBlock/zero", 0.0);
        write_f32(&mut pending, "DisplayBlock/one", input_scale);
        // ── The whole-pipeline op's own coefficients ──
        //
        // `isp.pipeline` has no ONNX attributes to carry these: VulkanFuse is
        // positional and reads none of the attribute keys it knows as
        // coefficients, so each value has to be a tensor the engine writes before
        // every frame. The names are the ones `PipelineAllBlock` declares.
        //
        // Gated on the session actually having the tensors, so a per-stage chain
        // pays nothing for this.
        if find(pool, "isp_pipeline/stages").is_some() {
            // `hw` is deliberately not written: it is the source and target rect,
            // which only the block knows at build time. Its
            // `extra_input_defaults` already carry the right numbers, and
            // overwriting them from here — where there is no geometry — would
            // replace them with zeroes and make the shader return immediately.
            //
            // White level: the shader divides by it, and it is also the scale a
            // black level or a bad-pixel threshold means something against.
            write_f32(&mut pending, "isp_pipeline/white", white_level.max(1.0));
            // Bayer gains per plane, in the plane order the phase table uses.
            if let Some(g) = bayer {
                write_f32_array(&mut pending, "isp_pipeline/wb", g);
            }
            if let Some(m) = ccm {
                write_f32_array(&mut pending, "isp_pipeline/ccm", m);
            }
            // TONE_CURVE/GAMMA_RECIP, the same value `isp.display` consumes: the
            // shader takes its reciprocal, so this is the display gamma.
            write_f32(
                &mut pending,
                "isp_pipeline/gamma",
                tone.gamma_recip.max(1e-4),
            );
            write_f32(
                &mut pending,
                "isp_pipeline/lift",
                tone.shadow_lift.clamp(0.0, 1.0),
            );
            write_f32(
                &mut pending,
                "isp_pipeline/pattern",
                bayer_pattern.clamp(0, 3) as f32,
            );
            write_f32_array(
                &mut pending,
                "isp_pipeline/sat",
                &[tone.saturation.clamp(0.0, 2.0)],
            );
            // A bad pixel is one further from its 3x3 median than this, in raw
            // counts — scaled off the white level so a 10-bit and a 12-bit sensor
            // get the same *relative* threshold.
            write_f32(
                &mut pending,
                "isp_pipeline/dpc",
                white_level.max(1.0) * 0.02,
            );
        }

        // Every `Clip(x, <ns>/zero, <ns>/one)` in the chain assumes a [0,1]
        // frame. With the divide deferred it must allow the full white level,
        // or highlights saturate at a single count.
        if input_scale != 1.0 {
            for ns in [
                "BlcBlock_dpc",
                "BlcBlock",
                "BayerWbBlock",
                "CcmBlock_ccm",
                "ToneBlock",
                "SaturationBlock",
            ] {
                write_f32(&mut pending, &format!("{ns}/one"), input_scale);
                write_f32(&mut pending, &format!("{ns}/zero"), 0.0);
            }
        }
        // GammaBlock — inv_gamma, min, max, lift, norm
        if let Some(isp) = isp_params {
            let inv_gamma = if isp.tone.gamma > 0.0 {
                1.0 / isp.tone.gamma
            } else {
                1.0
            };
            write_f32(&mut pending, "Gamma/inv_gamma", inv_gamma);
            write_f32(&mut pending, "Gamma/min", isp.tone.black_crush);
            write_f32(&mut pending, "Gamma/max", isp.tone.white_clip);
            write_f32(&mut pending, "Gamma/lift", isp.tone.black_crush);
            write_f32(&mut pending, "Gamma/norm", 1.0);
            // AutoContrastBlock — lift, half, contrast_w, zero, one
            write_f32(&mut pending, "AutoContrast/lift", isp.tone.black_crush);
            write_f32(&mut pending, "AutoContrast/half", 0.5);
            write_f32(&mut pending, "AutoContrast/contrast_w", isp.tone.contrast);
            write_f32(&mut pending, "AutoContrast/zero", 0.0);
            write_f32(&mut pending, "AutoContrast/one", 1.0);
            // DisplayBlock — scale, gamma_exp, zero, one (FloatRgb path)
            write_f32(&mut pending, "DisplayBlock/scale", 1.0);
            write_f32(&mut pending, "DisplayBlock/gamma_exp", 1.0 / 2.4);
            write_f32(&mut pending, "DisplayBlock/zero", 0.0);
            write_f32(&mut pending, "DisplayBlock/one", input_scale);
            // SaturationBlock/scale [3] — from saturation.factor
            write_f32_array(
                &mut pending,
                "saturation/scale",
                &[isp.saturation.factor; 3],
            );
            // Sharpen/strength [1] — from sharpen.amount
            write_f32(&mut pending, "Sharpen/strength", isp.sharpen.amount);
            // LdciBlock/strength [1] — from denoise params (use sharpen radius as proxy)
            write_f32(
                &mut pending,
                "LdciBlock/strength",
                isp.sharpen.radius.max(0.1),
            );
            // FcsBlock/gain [3] — per-channel gain from CCM diagonal
            let fcs_gain = if let Some(ccm) = ccm {
                [ccm[0], ccm[4], ccm[8]]
            } else {
                [1.0f32; 3]
            };
            write_f32_array(&mut pending, "FcsBlock/gain", &fcs_gain);
            // FcsBlock/bias [3] — per-channel bias
            write_f32_array(&mut pending, "FcsBlock/bias", &[isp.tone.brightness; 3]);
            // NormalizeBlock/max_val [1] — the sensor's white level, same
            // value as the default path above. Deriving it from `blc.r` was
            // wrong: black level and white level are unrelated quantities.
            let divisor = if input_scale > 1.0 {
                input_scale
            } else {
                white_level
            };
            write_f32(&mut pending, "NormalizeBlock/max_val", divisor.max(1.0));
        }

        // ── Warp grid + shading LUT (runtime extra_inputs) ────────
        // These are declared as graph inputs (not initializers) to prevent
        // MNN from DCE-ing the ISP graph. The engine writes the actual
        // per-frame data here.
        if let Some(grid) = warp_grid {
            write_f32_array(&mut pending, "WarpGrid/grid", grid);
        }
        if let Some(lut) = warp_shading {
            write_f32_array(&mut pending, "WarpGrid/shading_lut", lut);
        }

        // One upload per tensor, using the value that won. A fused/converted
        // graph legitimately lacks many primitive-sink names: warn once per
        // build (with the reason) instead of paying a file write per frame.
        let mut missing = Vec::new();
        for (name, value) in pending {
            let Some(t) = find(pool, &name) else {
                missing.push(name);
                continue;
            };
            match value {
                PendingWrite::Scalar(val) => {
                    t.upload_f32_shaped(&[1], &[val]);
                }
                PendingWrite::Array(vals) => {
                    // A failed upload leaves the tensor at whatever it was
                    // allocated with — usually zeros. Say so instead of
                    // swallowing the bool into a silently black frame.
                    if !t.upload_f32(&vals) {
                        warn!(
                            "{name}: upload of {} f32 rejected (tensor wants a different size)",
                            vals.len()
                        );
                    }
                }
                PendingWrite::Raw(data) => {
                    if !t.upload_bytes(&data, 2, 32) {
                        warn!("default for {name} ({} B) failed to upload", data.len());
                    }
                }
            }
        }
        if !missing.is_empty() {
            let mut reported = self.missing_inputs_reported.lock().unwrap();
            for name in missing {
                if reported.insert(name.clone()) {
                    debug!(
                        "extra input '{name}' is not a session input; skipped \
                         (fused into another tensor or its block is absent)"
                    );
                }
            }
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
        lsc_strength: f32,
    ) {
        // Two placements share this path: lens shading on the Bayer quad
        // (`[1,4,H/2,W/2]`) and vignetting on the RGB tail (`[1,3,H',W']`).
        // Each instance owns its own gain-map tensor, so anything the profile
        // did not add is simply absent from the pool.
        for (name, t) in pool {
            let Some((channels, scale)) = Self::gain_map_layout(name) else {
                continue;
            };
            let gh = (h / scale).max(1);
            let gw = (w / scale).max(1);
            let shape = vec![1i32, channels as i32, gh as i32, gw as i32];
            let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &shape);
            // Strength is the controller's tuned value (it raises it in dark
            // scenes); falloff stays at the lens model's 2.0. A strength of 0
            // means "no correction", which the map expresses as a flat 1.0.
            let gain = if lsc_strength <= 0.001 {
                vec![1.0f32; (gh * gw) as usize * channels as usize]
            } else {
                crate::blocks::VignettingBlock::gain_map_channels(
                    gh,
                    gw,
                    lsc_strength,
                    2.0,
                    channels,
                )
            };
            if !t.upload_f32(&gain) {
                warn!("{name}: gain map upload failed ({} f32)", gain.len());
            }
        }
    }

    /// `(channels, downsample factor)` for a gain-map tensor name.
    ///
    /// The RGB tail and the Bayer quad both live on the demosaiced grid, which
    /// is half the sensor in each axis; the channel count is what differs.
    fn gain_map_layout(name: &str) -> Option<(u32, u32)> {
        match name {
            // Tail vignetting (`VignettingBlock` with no instance).
            "vignetting/gain_map" => Some((3, 2)),
            // Lens shading on the post-CFA quad.
            "vignetting_lsc/gain_map" => Some((4, 2)),
            _ => None,
        }
    }
}

/// Whether any block in `chain` will emit a fused `isp.*` Extra op.
///
/// Two conditions, and both are needed:
///
///   `custom_op_types().is_some()`  — the block *can* emit a fused op. This is
///       static metadata: the five fusion-capable blocks declare it whether or
///       not they are currently fused.
///   `opset_mode() == MnnIspMin`    — the block is *set* to do so. Only those
///       same five blocks override `opset_mode`; everything else inherits the
///       `Custom` default.
///
/// Testing either alone gets it wrong. `custom_op_types()` alone calls every
/// chain fused, including a primitive one. `opset_mode()` alone calls every
/// chain fused too, because of that `Custom` default — which is what silently
/// forced fp32 tensor storage for the app's ordinary pipeline. Four fp32
/// sessions at 2592x1940 do not fit: the process stalled for seconds while
/// allocating sessions and then died mid-frame.
pub fn chain_has_fused_op(chain: &[Box<dyn IspBlock>]) -> bool {
    chain.iter().any(|b| {
        b.custom_op_types().is_some()
            && b.opset_mode() == crate::pipeline::BlockOpsetMode::MnnIspMin
    })
}

/// Check if the chain contains fused `isp.display_*` ops.
/// These ops write packed INT32 BGRA pixels, which the engine must
/// return as raw bytes without f32 reinterpretation.
pub fn chain_has_fused_op_for_display(chain: &[Box<dyn IspBlock>]) -> bool {
    chain.iter().any(|b| {
        if let Some(types) = b.custom_op_types() {
            types.iter().any(|t| t.starts_with("isp.display"))
        } else {
            false
        }
    })
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
        self.controller.as_ref()
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

            #[allow(clippy::type_complexity)]
            let mnn_result: (Vec<u8>, Option<Vec<(String, Vec<u8>)>>) = match &self.model_path {
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
                    let bytes = std::fs::read(p).map_err(|e| {
                        error!("Failed to read MNN model: {}", p);
                        crate::error::IspError::Io(format!("read {}: {}", p, e))
                    })?;
                    (bytes, None)
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

                    // ── Precision is a correctness constraint, not a knob ──
                    //
                    // Every `isp.*` shader declares `float[]` storage buffers,
                    // but `VulkanBackend::getTensorSize` allocates two bytes per
                    // element when `mUseFP16`. A fused op therefore reads halves
                    // as floats: the same fused CCM gives mean 0.8007 at fp32 and
                    // 0.0000 at fp16, with nothing else changed.
                    //
                    // So a chain that contains a fused op cannot use fp16
                    // storage. Detect that here rather than making every caller
                    // know — and say so, because it halves the memory headroom.
                    //
                    // The test is `custom_op_types()`, not `opset_mode()`.
                    // `opset_mode()` defaults to `Custom` for every block that
                    // does not override it — Identity, Blc, Unary and the rest —
                    // so asking it made *every* chain look fused. That silently
                    // forced fp32 for the app's ordinary primitive pipeline, and
                    // four fp32 sessions at 2592x1940 do not fit: the process
                    // grew a 5 s gap allocating sessions and then died mid-frame.
                    //
                    // `custom_op_types()` is the precise signal: exactly the five
                    // blocks that can emit an `isp.*` Extra op declare it.
                    let has_fused = chain_has_fused_op(&all);
                    if has_fused && self.fp16_storage {
                        warn!(
                            "chain contains fused isp.* ops — forcing fp32 tensor storage \
                             (their shaders declare float buffers; fp16 would hand them halves)"
                        );
                        self.fp16_storage = false;
                    }

                    // Record whether the chain has fused display ops for output decoding.
                    self.has_fused_display = crate::mnnengine::chain_has_fused_op_for_display(&all);

                    // Collect default values for extra inputs from every block
                    // before we discard the block list.  These replace the
                    // former baked Const initializers: the engine writes them
                    // into MNN Input tensors at inference time.
                    let mut block_defaults: Vec<(String, Vec<u8>)> = Vec::new();
                    for b in &all {
                        block_defaults.extend(b.extra_input_defaults());
                    }

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
                    let bytes = convert_onnx_buffer(&onnx).map_err(|e| {
                        crate::error::IspError::Conversion(format!("convert: {}", e))
                    })?;
                    (bytes, Some(block_defaults))
                }
            };

            if mnn_result.0.is_empty() {
                error!("MNN model bytes are empty");
                return Err(crate::error::IspError::Mnn("MNN model bytes empty".into()));
            }
            let interp = MnnInterpreterSafe::from_buffer(&mnn_result.0).ok_or_else(|| {
                error!(
                    "Failed to load MNN model from buffer ({} bytes)",
                    mnn_result.0.len()
                );
                crate::error::IspError::Mnn(format!("load fail ({} bytes)", mnn_result.0.len()))
            })?;

            // Use first session to probe model input type (before building pool)
            let probe_sess = interp
                .create_session(self.backend.to_sys(), 4, self.fp16_storage)
                .ok_or(crate::error::IspError::Mnn(
                    "probe session create fail (Vulkan backend unavailable)".into(),
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
                    // to halide codes (0=int/1=uint/2=float) so
                    // model_input_type stays compatible with the FFI buffer
                    // type codes used by mnn_run_with_output.
                    //
                    // The bit width has to come from the tensor: `data_type()`
                    // reports the *code* only, so an INT16 graph input would
                    // otherwise be recorded as 32-bit and fed a buffer twice
                    // the size MNN expects.
                    let halide_code = match code {
                        4 => 2, // halide_type_float
                        5 => 0, // halide_type_int
                        6 => 1, // halide_type_uint
                        _ => code,
                    };
                    let bits = match t.data_bits() {
                        8 | 16 | 32 | 64 => t.data_bits(),
                        _ => 32,
                    };
                    (halide_code, bits, ndim, dims4)
                }
                None => (0, 0, 0, [0; 4]),
            };
            // Record the type whenever the tensor existed. The guard used to be
            // `input_code != 0`, which is wrong for exactly the case it matters
            // most: `halide_type_int` *is* code 0, so every INT32 model input was
            // recorded as "unknown" and `InputMode::RawInt` then fell back to
            // `(0, 16)` — a 16-bit buffer handed to a 32-bit input:
            //
            //     input type mismatch: model code=0 bits=32, buffer code=0 bits=16
            //
            // `input_ndim` is the real "we found a tensor" signal.
            self.model_input_type = if input_ndim > 0 {
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
                } else if input_bits == 16 {
                    // The converter kept the declared INT16, so the lanes stay
                    // 16-bit: same values, half the scratch.
                    InputMode::SplitInt16
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

            // ── Enumerate ALL model inputs to build the extra-inputs list ──
            // The model may declare frame + many runtime-fed weight inputs.
            let all_input_names: Vec<String> = (0..interp.get_input_count(&probe_sess))
                .filter_map(|i| interp.get_input_name(&probe_sess, i))
                .collect();
            let extra_names: Vec<String> = all_input_names
                .into_iter()
                .filter(|n| Some(n.as_str()) != self.frame_input_name.as_deref())
                .collect();
            info!(
                "MNN model inputs: {} total (1 frame + {} extras)",
                1 + extra_names.len(),
                extra_names.len()
            );
            debug!("MNN model input names: {}", extra_names.join(", "));
            // Don't need probe session any more — pool will create its own.
            drop(probe_sess);

            // Build session pool (N sessions for parallel inference)
            // Backend was already validated by probe session above.
            //
            // Session count is a *memory* budget, not a preference. Each
            // session allocates its own Vulkan buffers for the whole graph, so
            // at 5 Mpx the default four need several hundred MB. `SessionPool`
            // halves the count on failure rather than taking the process down
            // (on device the log simply stopped at "MNN model inputs: N total"
            // with no panic line at all).
            let pool = SessionPool::new(
                interp,
                self.backend.to_sys(),
                self.pool_size.max(1),
                4,
                self.fp16_storage,
                extra_names,
            )?;
            self.pool_size = pool.len();
            info!(
                "MNN engine loaded ({} bytes, backend={:?}, {} sessions, {} extra inputs)",
                mnn_result.0.len(),
                self.backend,
                self.pool_size,
                pool.tensor_count()
            );
            self.pool = Some(pool);
            self.missing_inputs_reported.lock().unwrap().clear();
            // Cache block-provided default values for extra inputs (formerly
            // baked as Const initializers).  These are written before each
            // inference; isp_params per-frame values take precedence.
            if let Some(defaults) = mnn_result.1 {
                let mut cache = self.default_inputs.lock().unwrap();
                *cache = defaults;
            }
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
        // Needed by the 3A snapshot below (per-plane gain mapping).
        let bayer_pattern = p.bayer_pattern;

        // ── Close the 3A loop ──
        //
        // The stats phase below calls `update_channel_stats` /
        // `update_tone_stats`, which move `awb_gains` and `smoothed_ccm` on the
        // controller. But the graph is fed from `ProcessParams`, and no caller
        // has to fill those in — the app passes `ccm_matrix: None`,
        // `bayer_gains: None`, `awb_gains: None`. The controller then computes
        // gains that nothing applies, and with a static scene the picture is
        // byte-identical frame after frame while `awb_gains` walks.
        //
        // Explicit values from the caller still win; only the slots left empty
        // are taken from the controller.
        let ctrl = self.controller.lock().ok().map(|c| ControllerState {
            awb: c.get_awb_gains(),
            ccm: c.get_ccm(),
            tone: c.get_tone_params(),
            frames: c.frame_count,
            exposure_gain: c.get_effective_exposure_gain(),
        });

        // Per-plane gains.
        //
        // The CFA stage emits planes in physical quad order (TL, TR, BL, BR), so
        // which plane holds red depends on the sensor's Bayer pattern. Writing
        // the controller's `[R, G, B]` straight into planes 0..3 only happens to
        // be right for RGGB; on the GBRG sensor this device reports it puts the
        // red gain on a *green* plane and the blue gain on red, which is a
        // permanent colour cast no white balance can remove.
        let ctrl_bayer: [f32; 4] = match &ctrl {
            Some(c) => crate::bayer_planes::gains_by_plane(&c.awb, bayer_pattern),
            None => [1.0; 4],
        };
        let bayer_src: Option<&[f32; 4]> = p.bayer_gains.as_ref().or(Some(&ctrl_bayer));
        let ccm_src: Option<&[f32; 9]> = p.ccm_matrix.as_ref().or(ctrl.as_ref().map(|c| &c.ccm));
        // Only trust the controller's tone once it has seen frames; before that
        // it is the untouched default.
        let tone_base: &cam_types::ToneParams = match &ctrl {
            Some(c) if c.frames > 0 => &c.tone,
            _ => &p.tone_params,
        };

        let awb = p.awb_gains.as_ref();

        // `sensor_max` is normally applied by `NormalizeBlock` at the head of
        // the chain, and `set_extra_inputs` feeds it the divisor. When the
        // pipeline was instead built with the divide folded into the display,
        // that block is absent: the graph works in raw sensor counts and the
        // values that carry physical meaning have to be scaled here instead.
        let input_scale = if self.normalizes_at_input || smax <= 1.0 {
            1.0
        } else {
            smax
        };

        // If isp_params is available, override individual fields
        // This allows per-frame controller params to flow through
        let mut isp_plane_gains = [1.0f32; 4];
        let (override_ccm, override_tone, _no_flat_wb) = if let Some(ref params) = p.isp_params {
            let tone = cam_types::ToneParams {
                contrast: params.tone.contrast,
                brightness: params.tone.brightness,
                gamma_recip: params.tone.gamma,
                ..Default::default()
            };
            // WB gains arrive as [R, G, B] and every consumer is plane-ordered:
            // `isp.pipeline` indexes its gains by CFA plane, so a flat
            // [R, G, G, B] only happens to be right for RGGB. On a GRBG sensor it
            // puts the red gain on a green plane and the blue gain on red, which
            // renders as a cast no downstream correction can remove.
            //
            // Bound outside the tuple so it lives as long as the borrow.
            (Some(&params.ccm.matrix), Some(tone), None::<&[f32; 4]>)
        } else {
            (None, None, None)
        };
        // In the deferred scheme the frame is still in raw sensor counts.
        // Brightness is additive against the frame so it scales by `smax`;
        // contrast is multiplicative and means the same thing either way.
        let scaled_tone;
        let tone = if self.normalizes_at_input {
            override_tone.as_ref().unwrap_or(tone_base)
        } else {
            let base = override_tone.as_ref().unwrap_or(tone_base);
            scaled_tone = cam_types::ToneParams {
                contrast: base.contrast,
                brightness: base.brightness * input_scale,
                gamma_recip: base.gamma_recip,
                ..*base
            };
            &scaled_tone
        };
        // Map the callers [R, G, B] onto the sensors planes, which is what the
        // blocks and the fused op both index by.
        if let Some(wb) = p.isp_params.as_ref() {
            isp_plane_gains =
                crate::bayer_planes::gains_by_plane(&[wb.wb.r, wb.wb.g, wb.wb.b], bayer_pattern);
        }
        let override_bayer = p.isp_params.as_ref().map(|_| &isp_plane_gains);
        let ccm = override_ccm.or(ccm_src);
        let tone = override_tone.as_ref().unwrap_or(tone);
        let bayer = override_bayer.or(bayer_src);

        if !self.initialized {
            return Err(crate::error::IspError::Config("not init".into()));
        }

        #[cfg(feature = "mnn")]
        {
            let pool = self
                .pool
                .as_ref()
                .ok_or(crate::error::IspError::Config("no pool".into()))?;
            let t_enter = Instant::now();
            let mut slot = pool.acquire();
            let interp = &pool.interp;

            let t_start = Instant::now();
            debug!("pipeline timer pool_acquire={:?}", t_start - t_enter);

            debug!(
                "pipeline stage=arrive raw={}×{} bayer={}B sensor_max={}",
                w,
                h,
                buf.len(),
                smax
            );

            // ── Pre-allocate output buffer (zero-copy target) ──
            //
            // When the caller supplied `output_buffer`, MNN writes directly into
            // it via `mnn_run_true_zero_copy` — no temporary Vec, no memcpy.
            // Otherwise we use the pre-allocated OutputBufferPool to avoid
            // per-frame 132 MB zero-fill at 4K.
            let zero_copy = p.output_buffer.is_some();
            let (out_ptr, max_out, buf_id_for_release) =
                if let Some(&(buf_ptr, buf_len)) = p.output_buffer.as_ref() {
                    // The caller's buffer must hold at least max_out f32 elements.
                    let max_out = (buf_len / 4) as i32;
                    if max_out <= 0 {
                        return Err(crate::error::IspError::Config(
                            "output_buffer too small for zero-copy inference".into(),
                        ));
                    }
                    (buf_ptr as *mut f32, max_out, None)
                } else {
                    // Fallback: use pre-allocated OutputBufferPool to avoid per-frame
                    // 132 MB zero-fill at 4K (H×W×4 f32 elements = 64 MB f32 = 132 MB u8).
                    //
                    // The element count must match what the FFI hands to MNN as
                    // `max_out`: a 4K frame writes ~33M f32 through `out_ptr`, and a
                    // smaller pool buffer is silently overflowed by `runSession`
                    // (heap corruption → SIGSEGV). The graph's own output geometry
                    // (display downscale etc.) is only known AFTER inference, so the
                    // worst case — full sensor area × 4 channels — is the safe
                    // ceiling; acquire() grows the pool once, lazily.
                    let needed_capacity = (tw * h * 4) as usize;
                    let mut pool = self.buf_pool.lock().unwrap();
                    let (buf_id, buf) = pool.acquire(needed_capacity);
                    // Capacity is now guaranteed >= needed_capacity f32 elements.
                    let max_out = buf.len() as i32;
                    (buf.as_mut_ptr(), max_out, Some(buf_id))
                };
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
            // The extent the FRAME input takes. It has to match what the feed
            // below hands over, or `resize` propagates a shape twice the width of
            // the buffer:
            //   split lanes          [1,2,H,W/2]
            //   packed-word RAW      [1,1,H,W/2]  (u16 samples seen as int32 words)
            //   one sample per pixel [1,1,H,W]
            let packed_words = matches!(self.model_input_type, Some((0, 32)));
            let shape = if split {
                vec![1, 2, h as i32, (w / 2).max(1) as i32]
            } else if packed_words {
                vec![1, 1, h as i32, (w / 2).max(1) as i32]
            } else {
                vec![1, 1, h as i32, w as i32]
            };
            // Resize decision is PER SLOT.
            //
            // `Tensor::set_shape` calls `Interpreter::resizeTensor`, which
            // takes an extent change as a signal to `setNeedResize()` on the
            // session that owns the tensor. `Session::run()` then returns
            // COMPUTE_SIZE_ERROR until that session is resized again. Sessions
            // are handed out round-robin, so with a single engine-wide "last
            // shape" key only the first session was ever resized: the other
            // three failed every time with exactly the COMPUTE_SIZE_ERROR the
            // camera saw.
            let needs_resize = slot.resized_for != Some((w, h));
            if needs_resize {
                // Vignetting gain map is a runtime extra input (not a baked
                // initializer): resize it to the post-demosaic dims (h/2 × w/2)
                // and fill with computed gains BEFORE resizeSession so the Mul
                // shape check passes during propagation. A baked [1,1,H,W] gain
                // would break resizeSession for any input size differing from
                // the model's baked dims.
                // The LSC strength is a controller output; ask it once per
                // resize rather than locking again in the hot path.
                let lsc_strength = self
                    .controller
                    .lock()
                    .map(|c| c.get_lsc_strength())
                    .unwrap_or(0.0);
                Self::write_vignetting_gain(
                    &slot.tensor_pool,
                    interp,
                    &slot.sess,
                    h,
                    w,
                    lsc_strength,
                );
                // Resolve the frame input by NAME (never getSessionInput(
                // nullptr), which is the alphabetically-first input in
                // multi-input models) and give it this frame's extent.
                let frame_t = interp
                    .get_input(&slot.sess, self.frame_input_name.as_deref().unwrap_or(""))
                    .or_else(|| interp.get_first_input(&slot.sess));
                if let Some(t) = frame_t {
                    let _ = t.set_shape(interp.as_ptr(), slot.sess.as_ptr(), &shape);
                }
                // Surface the real reason instead of deferring to the FFI's
                // opaque `n=-3`: this call is where a graph that cannot take
                // the geometry actually fails.
                if let Err(e) = slot.sess.resize() {
                    return Err(crate::error::IspError::Mnn(format!(
                        "session resize failed for {w}x{h} (mode={mode:?}): {e}. \
                         The graph cannot be resized to this input geometry — check \
                         that every shape-dependent block got concrete_h/concrete_w \
                         at build time."
                    )));
                }
                slot.resized_for = Some((w, h));
                // `resize()` re-allocates the session tensors, so the handles
                // cached in the pool (and used above for the gain map) are stale
                // for everything that follows. Re-resolve them before writing
                // the per-frame weights, or those writes land in freed memory
                // and the frame comes back black.
                slot.refresh_tensors(interp);
            }
            let sess = &slot.sess;
            // MNN consumes input tensors on every run; their host buffers are
            // not guaranteed to retain the previous frame's values, so every
            // runtime input is re-uploaded unconditionally.
            self.set_extra_inputs(
                &slot.tensor_pool,
                ccm,
                tone,
                bayer,
                awb,
                bayer_pattern,
                p.isp_params.as_ref(),
                p.warp_grid,
                p.warp_shading,
                input_scale,
                smax,
                ctrl.as_ref().map_or(1.0, |c| c.exposure_gain),
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
            let split_scratch_16: Option<std::sync::MutexGuard<'_, Vec<i16>>>;
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
                    split_scratch_16 = None;
                    scratch_f32 = None;
                }
                InputMode::SplitInt16 => {
                    let pairs = ((w / 2).max(1) as usize) * h as usize;
                    let mut guard = self.split_scratch_i16.lock().unwrap();
                    if guard.len() != pairs * 2 {
                        guard.resize(pairs * 2, 0);
                    }
                    let u16s: &[u16] = unsafe {
                        std::slice::from_raw_parts(buf.as_ptr() as *const u16, buf.len() / 2)
                    };
                    for i in 0..pairs {
                        guard[i] = u16s[2 * i] as i16;
                        guard[pairs + i] = u16s[2 * i + 1] as i16;
                    }
                    split_scratch_i32 = None;
                    split_scratch_16 = Some(guard);
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
                    split_scratch_16 = None;
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
                    split_scratch_16 = None;
                    scratch_f32 = Some(guard);
                }
                InputMode::RawInt => {
                    split_scratch_i32 = None;
                    split_scratch_16 = None;
                    scratch_f32 = None;
                }
            }

            // Which tensor is the result? `DisplayBlock/frame` for a full
            // chain, the tail block's own name otherwise. Resolved once here
            // instead of assumed, because a miss reads a null tensor and faults
            // inside MNN with no message that names the cause.
            let output_name = if interp.get_output(sess, "DisplayBlock/frame").is_some() {
                "DisplayBlock/frame".to_string()
            } else {
                match interp.output_names(sess).first() {
                    Some(n) => n.clone(),
                    None => "DisplayBlock/frame".to_string(),
                }
            };
            let output_name_ptr = std::ffi::CString::new(output_name.clone())
                .unwrap_or_else(|_| c"DisplayBlock/frame".to_owned());

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
                InputMode::SplitInt16 => {
                    let v = split_scratch_16.as_ref().unwrap();
                    (
                        v.as_ptr() as *const c_void,
                        0, // halide_type_int
                        16,
                        vec![1, 2, h as i32, (w / 2).max(1) as i32],
                        "packed_split_i16",
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
                    // The graph head decides the wire dtype, not the enum name:
                    //   isp.unpack16_blc_normal_cfa → INT16 input → zero-copy u16 buffer
                    //   isp.unpack32_blc_normal_cfa → INT32 input → zero-copy u16 buffer
                    //                                  (W/2 int32 words per row; the shader
                    //                                   splits each word into its two int16
                    //                                   lanes, then converts to float)
                    // RawInt is only reached when the converter declared a single
                    // non-packed channel, so one sample (or one packed word per two
                    // samples) per lane.
                    let (code, bits) = self.model_input_type.unwrap_or((0, 16));
                    // A 32-bit integer RAW input is the packed-word form: W*H
                    // contiguous u16 samples are W/2 int32 words per row, so the
                    // row extent halves and the byte count stays W*H*2. Feeding
                    // the un-halved shape declares twice the elements the buffer
                    // holds and the resize/upload is rejected.
                    let packed_words = code == 0 && bits == 32;
                    let raw_shape = if packed_words {
                        [1, 1, h as i32, (w / 2).max(1) as i32]
                    } else {
                        [1, 1, h as i32, w as i32]
                    };
                    // Zero-copy input: if the caller supplied input_buffer,
                    // pass its pointer directly instead of p.buf.  This
                    // eliminates the AHardwareBuffer → Vec<u8> memcpy on the
                    // input side.
                    let input_ptr = if let Some(&(ptr, _)) = p.input_buffer.as_ref() {
                        ptr as *const c_void
                    } else {
                        buf.as_ptr() as *const c_void
                    };
                    (input_ptr, code, bits, raw_shape.to_vec(), "raw_zero_copy")
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
            let t_infer_end: std::time::Instant;
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
                // Choose the inference path: true zero-copy (caller's buffer)
                // or the standard copy-in/copy-out path.
                if zero_copy {
                    // Import AHardwareBuffers as Vulkan device memory before
                    // inference, so MNN's Vulkan backend reads/writes GPU
                    // memory directly instead of host<->GPU copies. The input
                    // name MUST be the frame input — the old code used the
                    // output name here and silently bound the wrong tensor.
                    let mut imported_input = false;
                    if let Some(&(ahb_ptr, _)) = p.input_ahb.as_ref() {
                        if !frame_name_ptr.is_null() {
                            let rc = crate::mnn_sys::mnn_import_ahb_as_device_memory(
                                interp.as_ptr(),
                                sess.as_ptr(),
                                ahb_ptr as *const c_void,
                                frame_name_ptr,
                                1, // read_only: input tensor
                            );
                            if rc != 0 {
                                warn!("AHB import for frame input failed: rc={rc}");
                            } else {
                                imported_input = true;
                            }
                        } else {
                            warn!("AHB input import skipped: frame input name is unknown");
                        }
                    }
                    let mut imported_output = false;
                    if let Some(&(ahb_ptr, _)) = p.output_ahb.as_ref() {
                        let rc = crate::mnn_sys::mnn_import_ahb_as_device_memory(
                            interp.as_ptr(),
                            sess.as_ptr(),
                            ahb_ptr as *const c_void,
                            output_name_ptr.as_ptr(),
                            0, // read_only: output tensor
                        );
                        if rc != 0 {
                            warn!("AHB import for output '{output_name}' failed: rc={rc}");
                        } else {
                            imported_output = true;
                        }
                    }

                    n = crate::mnn_sys::mnn_run_true_zero_copy(
                        interp.as_ptr(),
                        sess.as_ptr(),
                        frame_name_ptr,
                        output_name_ptr.as_ptr(),
                        buffer_ptr,
                        buffer_type_code,
                        buffer_type_bits,
                        input_shape.as_ptr(),
                        input_shape.len() as i32,
                        out_ptr,
                        max_out,
                    );

                    // Release the AHB device-memory bindings after inference.
                    if imported_input {
                        let rc = crate::mnn_sys::mnn_release_ahb_device_memory(
                            interp.as_ptr(),
                            sess.as_ptr(),
                            frame_name_ptr,
                        );
                        if rc != 0 {
                            warn!("AHB release for frame input failed: rc={rc}");
                        }
                    }
                    if imported_output {
                        let rc = crate::mnn_sys::mnn_release_ahb_device_memory(
                            interp.as_ptr(),
                            sess.as_ptr(),
                            output_name_ptr.as_ptr(),
                        );
                        if rc != 0 {
                            warn!("AHB release for output '{output_name}' failed: rc={rc}");
                        }
                    }
                } else {
                    n = crate::mnn_sys::mnn_run_with_output(
                        interp.as_ptr(),
                        sess.as_ptr(),
                        frame_name_ptr,
                        buffer_ptr,
                        buffer_type_code,
                        buffer_type_bits,
                        input_shape.as_ptr(),
                        input_shape.len() as i32,
                        output_name_ptr.as_ptr(),
                        out_ptr,
                        max_out,
                    );
                }
                t_infer_end = Instant::now();
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
            let t_after_profiling = Instant::now();

            if n <= 0 {
                // The FFI's negative code alone cannot say *why*: -6/-7/-8 are
                // shape problems, a negative MNN code is a backend error. The
                // C side records the reason alongside the code.
                let detail = unsafe {
                    let p = crate::mnn_sys::mnn_last_error();
                    if p.is_null() {
                        String::new()
                    } else {
                        CStr::from_ptr(p).to_string_lossy().into_owned()
                    }
                };
                error!(
                    "MNN inference failed: n={} (input={}x{}, output_name='{}', path={})",
                    n, w, h, output_name, path
                );
                if detail.is_empty() {
                    error!("  This usually means the MNN IspChainFusion pass computed wrong output dimensions.");
                    error!(
                        "  Check that RawInputBlock has concrete_h and concrete_w set (not -1)."
                    );
                } else {
                    error!("  MNN reported: {detail}");
                }
                let reason = if detail.is_empty() {
                    format!("MNN inference failed: n={n} (input={w}x{h})")
                } else {
                    format!("MNN inference failed: n={n} (input={w}x{h}): {detail}")
                };
                return Err(crate::error::IspError::Mnn(reason));
            }

            debug!(
                "pipeline stage=infer_done path={} ({}x{} -> {} elts) infer={:?}",
                path,
                w,
                h,
                n,
                t_infer_end - t_infer_start
            );

            // ── Determine output size and encoding from the output tensor ──
            //
            // The frame must be described by what the graph actually produced,
            // not by the requested `target_width`/`target_height`: the display
            // stage may resize, and a mismatch here makes the engine hand back
            // a buffer whose length disagrees with `frame.width * height *
            // channels` (silently truncating or mis-slicing the image).
            let out_tensor = interp.get_output(sess, &output_name);
            let out_type = out_tensor.as_ref().map_or(4, |t| t.data_type());
            let mut oh = p.target_height as usize;
            let mut ow = tw as usize;
            let mut graph_channels = p.output_format.channel_count();
            if let Some(t) = &out_tensor {
                let shape = t.shape();
                // Trust any concrete `[1,C,H,W]` shape, including PackedRgb:
                // the fused `isp.display_uint32_bgra` op declares one uint32
                // per pixel (this was fixed alongside the shader's packing
                // semantics), so its last dim is the real output width.
                // Reporting the *requested* geometry here instead makes the
                // caller size a full-size BGRA buffer for a half-size packed
                // frame; `present()` then rejects it as "source frame too
                // small" and the preview stays black.
                if shape.len() == 4 && shape[2] > 0 && shape[3] > 0 {
                    graph_channels = shape[1].max(1) as usize;
                    oh = shape[2] as usize;
                    ow = shape[3] as usize;

                    // The graph's channel count and the caller's expectation
                    // must agree: the buffer is decoded by interpreting channel
                    // 0 as the start of the image. A `[1,4,H,W]` display Conv
                    // fed to a `FloatRgb` caller is silently wrong — every
                    // plane lands 8x too large in the wrong place — so say so
                    // instead of rendering garbage.
                    // A fused `isp.display_*` shader writes one packed BGRA
                    // word per pixel, so its `[1,1,H,W]` shape is correct even
                    // for a four-byte u8 format. Compare against one channel
                    // there; the host expands the word afterwards.
                    let packed_u8_tail = !p.output_format.is_raw_output()
                        && !matches!(p.output_format, OutputFormat::PackedRgb)
                        && graph_channels == 1
                        && (out_type != 4 || self.has_fused_display);
                    let expected = if packed_u8_tail {
                        1
                    } else {
                        p.output_format.channel_count()
                    };
                    if graph_channels != expected
                        && !CHANNEL_MISMATCH_REPORTED.swap(true, Ordering::Relaxed)
                    {
                        // A configuration error, not a frame error: it holds
                        // for every frame, so report it exactly once.
                        warn!(
                            "output channel mismatch: graph produced {} channels, {:?} expects {}. \
                             The profile's `output_format` and `ProcessParams::output_format` \
                             must agree, or every plane lands in the wrong place.",
                            graph_channels, p.output_format, expected
                        );
                    }
                }
            }
            let pixels = oh * ow;

            // Two encodings can come back, and only the tensor itself says
            // which — by its element type:
            //
            //  * FLOAT — the plain ONNX graph. `DisplayBlock` emits
            //    `f32 [1,C,H,W]` in `[0,255]` for every u8 format; ONNX cannot
            //    hand a u8 consumer its bytes, so the host narrows.
            //  * INT32/UINT32 — MNN's Vulkan fusion rewrote the display Conv
            //    into an `isp.display_*` shader that writes one packed word
            //    per pixel (`a<<24 | b<<16 | g<<8 | r`, i.e. R,G,B,A in
            //    little-endian byte order — exactly the BGRA bytes consumers
            //    read). Those words are already the answer.
            //
            // Inferring the encoding from the element *count* instead would
            // misfire, because a 1×1 Conv output of `pixels/channels` lands on
            // the same count.
            let packed_display_by_type = !p.output_format.is_raw_output() && out_type != 4;

            // `isp.display_uint32_bgra` writes one BGRA uint32 per pixel. MNN
            // reports its element type as UINT32 (6), or INT32 (5) when the
            // shader's unsigned bits are preserved through the Vulkan path.
            // `is_raw_output()` alone cannot tell that from a float tail, so
            // decide from the tensor type as well and take the packed path.
            let packed_u32 =
                matches!(p.output_format, OutputFormat::PackedRgb) && matches!(out_type, 5 | 6);

            // Robust fallback: MNN may report FLOAT (type 4) for fused
            // `isp.display_*` outputs even though the shader writes INT32.
            let has_fused_display = self.has_fused_display;
            let is_u8_format = !p.output_format.is_raw_output()
                && !matches!(p.output_format, OutputFormat::PackedRgb);
            let packed_display =
                packed_u32 || packed_display_by_type || (has_fused_display && is_u8_format);
            let t_output_meta = Instant::now();

            let n_valid = n.max(0) as usize;
            let out_elems = n_valid.min(pixels * graph_channels);
            let graph_elem = if p.output_format.is_fp16() { 2 } else { 4 };
            let _raw_bytes = out_elems * graph_elem;

            // Narrow f32 `[0,255]` → u8 only for the graph-native float path.
            // The graph does all colour work; this is only the type cast the
            // ONNX graph cannot express for a u8 consumer.
            //
            // Only convert `out_elems` f32 values (not the whole pool buffer):
            // for `FloatRgb` (3 channels) the valid output is `pixels*3` f32,
            // but the pool holds `tw*h*4` f32 elements. Passing the full buffer
            // would decode trailing garbage as u8 → near-zero mean → blank frame.
            let valid_bytes = out_elems * graph_elem;
            let data_out = if zero_copy {
                // MNN wrote directly into the caller's buffer via
                // mnn_run_true_zero_copy. Return empty data; the real
                // pixels live in `external_data` (set below).
                Vec::new()
            } else if packed_u32 {
                // Fused packed display: little-endian BGRA bytes, four per
                // pixel. Return them verbatim — the uint32 words are already
                // the final pixel layout.
                let buf_id = buf_id_for_release.expect("pool buffer id");
                let mut pool = self.buf_pool.lock().unwrap();
                let buf = pool.buffer_mut(buf_id);
                let buf_bytes =
                    unsafe { std::slice::from_raw_parts(buf.as_ptr() as *const u8, valid_bytes) };
                buf_bytes.to_vec()
            } else if p.output_format.is_raw_output() {
                // Float graph output (e.g. FloatRgb): return raw bytes as-is.
                // The graph already emits the final layout (f32 planar for
                // FloatRgb), so no narrowing is needed — the consumer
                // reinterprets these bytes as f32 values.
                let buf_id = buf_id_for_release.expect("pool buffer id");
                let mut pool = self.buf_pool.lock().unwrap();
                let buf = pool.buffer_mut(buf_id);
                let buf_bytes =
                    unsafe { std::slice::from_raw_parts(buf.as_ptr() as *const u8, valid_bytes) };
                buf_bytes.to_vec()
            } else if packed_display {
                // Fused `isp.display_*` shader wrote packed INT32 words — one
                // BGRA u8 pixel per word, little-endian. These bytes are *already*
                // the final answer; do NOT reinterpret as f32 (that collapses
                // every pixel to ~0 → blank frame).
                let buf_id = buf_id_for_release.expect("pool buffer id");
                let mut pool = self.buf_pool.lock().unwrap();
                let buf = pool.buffer_mut(buf_id);
                let buf_bytes =
                    unsafe { std::slice::from_raw_parts(buf.as_ptr() as *const u8, valid_bytes) };
                buf_bytes.to_vec()
            } else {
                // SplitInt32 path: read f32 from pool buffer, clamp to u8.
                let buf_id = buf_id_for_release.expect("pool buffer id");
                let mut pool = self.buf_pool.lock().unwrap();
                let buf = pool.buffer_mut(buf_id);
                let buf_bytes = unsafe {
                    std::slice::from_raw_parts_mut(buf.as_mut_ptr() as *mut u8, valid_bytes)
                };
                Self::f32_to_u8(buf_bytes)
            };
            let t_output_copy = Instant::now();

            // Black-frame triage: an all-zero frame means either the graph
            // tail ran on zeroed inputs or the readback branch disagreed with
            // the tensor's real encoding. Sample (not scan) the bytes, and on
            // a black frame compare against the session's own DisplayBlock
            // output to tell the two apart. Allocations happen only when a
            // black frame is already detected.
            if !data_out.is_empty() {
                // Odd step: an even (especially multiple-of-4) stride would
                // always land on the zero low byte of a packed BGRA word and
                // report an opaque-black frame as "all zero".
                let step = (data_out.len() / 128).max(1) | 1;
                if data_out.iter().step_by(step).all(|&b| b == 0) {
                    let mut probe = vec![0.0f32; (oh * ow * graph_channels).max(4)];
                    let disp_n = interp.read_output_floats(sess, "DisplayBlock/frame", &mut probe);
                    let disp_max = probe.iter().fold(0.0f32, |m, v| m.max(v.abs()));
                    warn!(
                        "black frame: bytes={} branch={} out_type={} out_elems={} graph={}x{} \
                         needs_resize={} session_display={:?} session_max_abs={:.4}",
                        data_out.len(),
                        if zero_copy {
                            "zero_copy"
                        } else if packed_u32 {
                            "packed_u32"
                        } else if p.output_format.is_raw_output() {
                            "raw"
                        } else if packed_display {
                            "packed"
                        } else {
                            "narrow"
                        },
                        out_type,
                        out_elems,
                        ow,
                        oh,
                        needs_resize,
                        disp_n,
                        disp_max
                    );
                }
            }

            // ── Read stats output tensors from MNN session and feed to controller ──
            let mut calib_vals: Option<[f32; 24]> = None;
            // After inference, all output tensors are available in the session.
            // Try reading known stats output names.  If a tensor doesn't exist
            // in the graph (block not enabled in this profile), get_output returns None.
            // Measure the statistics on the first frame and every `n` frames
            // after it. This is decided per frame, before any tensor is read, so a
            // skipped frame costs no readback at all — the readback is the stall,
            // not the reduction.
            // Two reasons to skip, and they mean the same thing to a consumer: the
            // switch is off, or this frame is not the one in `stats_interval`.
            //
            // Both are decided here, before any tensor is touched, so a skipped
            // frame costs no readback at all — the readback (a `copyToHostTensor`
            // that waits for the queue) is the stall, not the reduction.
            let measure_stats = self.stats_enabled.load(Ordering::Relaxed)
                && if self.stats_countdown.load(Ordering::Relaxed) == 0 {
                    self.stats_countdown
                        .store(self.stats_interval.max(1) - 1, Ordering::Relaxed);
                    true
                } else {
                    self.stats_countdown.fetch_sub(1, Ordering::Relaxed);
                    false
                };

            #[cfg(feature = "mnn")]
            if measure_stats {
                // ── Phase 1: Read raw stats tensors into local variables ──
                // (Avoid borrow conflicts with ctrl write_stats() above)
                let mut cm_vals: Option<[f32; 3]> = None;
                let mut ts_vals: Option<[f32; 6]> = None;
                let mut hist_vals: Option<[f32; 16]> = None;
                let mut zone_data: Option<ZoneData> = None;

                /// Read a named output tensor and reinterpret bytes as `f32`.
                /// Returns up to `max_n` floats, or `None` if the tensor is missing.
                /// Read a named output tensor as floats.
                ///
                /// Goes through `mnn_session_read_output` (`copyToHostTensor`)
                /// rather than `host<float>()`. On Vulkan a tensor's data lives
                /// in a device buffer and the host pointer is null, so the
                /// direct read returned `None` for every stats tensor — which
                /// is why 3A never received any feedback.
                fn read_output_f32(
                    interp: &crate::mnn_sys::MnnInterpreterSafe,
                    sess: &crate::mnn_sys::MnnSessionSafe,
                    name: &str,
                    max_n: usize,
                ) -> Option<Vec<f32>> {
                    let mut buf = vec![0.0f32; max_n];
                    let n = interp.read_output_floats(sess, name, &mut buf)?;
                    let n = n.min(max_n);
                    if n == 0 {
                        None
                    } else {
                        Some(buf.into_iter().take(n).collect())
                    }
                }

                // ChannelMeansBlock/frame → [1, 3] or [3]
                // The stats tensors are registered as graph outputs by the
                // composer, but the converted model does not always keep them:
                // `oplists` shows the nodes while `get_output` returns None for
                // the name. When that happens the controller silently sees no
                // data and AWB/AE stop adapting, which looks like "3A does
                // nothing" rather than a missing tensor. Say so explicitly.
                // Report the session's actual output list once per session build
                // prefix. `get_output(name)` returning null for a tensor that is
                // in `oplists` is otherwise indistinguishable from "the profile
                // did not enable that block".
                {
                    let names = interp.output_names(sess);
                    debug!(
                        "session exposes {} outputs: {}",
                        names.len(),
                        names.join(", ")
                    );
                }

                // The tap's width depends on where it sits in the chain, and
                // both placements are legitimate:
                //
                //   [1, 4]  after CFA, before demosaic — one mean per Bayer
                //           site (R, Gr, Gb, B). No white balance has been
                //           applied yet, so these describe the sensor and the
                //           AWB loop cannot chase its own output.
                //   [1, 3]  after demosaic — already RGB.
                //
                // The tensor's own shape is the only safe way to tell them
                // apart: guessing from the value count rejected the 4-site form
                // outright, and assuming 3 sites silently mixed Gb in as if it
                // were B.
                match read_output_f32(interp, sess, "ChannelMeansBlock/frame", 8) {
                    Some(vals) => {
                        // Odd lengths are impossible for a reduction over a
                        // whole channel axis; treat anything but 3 or 4 as
                        // unusable rather than folding a truncated read.
                        match vals.len() {
                            4 => {
                                // Planes are physical quad positions, so fold
                                // through the pattern rather than assuming
                                // plane 0 is red.
                                let planes: [f32; 4] =
                                    [vals[0], vals[1], vals[2], vals[3]];
                                let rgb = crate::bayer_planes::planes_to_rgb(
                                    &planes,
                                    bayer_pattern,
                                );
                                debug!(
                                    "stats: planes [{:.4}, {:.4}, {:.4}, {:.4}] (pattern {}) -> rgb [{:.4}, {:.4}, {:.4}]",
                                    vals[0], vals[1], vals[2], vals[3], bayer_pattern,
                                    rgb[0], rgb[1], rgb[2]
                                );
                                cm_vals = Some(rgb);
                            }
                            3 => {
                                debug!(
                                    "stats: rgb channel means [{:.4}, {:.4}, {:.4}]",
                                    vals[0], vals[1], vals[2]
                                );
                                cm_vals = Some([vals[0], vals[1], vals[2]]);
                            }
                            n => debug!(
                                "stats: ChannelMeansBlock/frame has {n} values; expected 3 (RGB) \
                                 or 4 (CFA sites R,Gr,Gb,B)"
                            ),
                        }
                    }
                    None => debug!(
                        "stats: ChannelMeansBlock/frame unreadable - the controller gets no feedback"
                    ),
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
            let t_stats_done = Instant::now();
            let t_frame_done = Instant::now();

            debug!(
                "pipeline timer path={} setup={:?} tensor_assign={:?} input={:?} infer={:?} \
                 profiling={:?} output_meta={:?} output_copy={:?} stats={:?} frame={:?} total={:?}",
                path,
                t_tensor_before - t_start,
                t_tensor_after - t_tensor_before,
                t_prep_end - t_tensor_after,
                t_infer_end - t_infer_start,
                t_after_profiling - t_infer_end,
                t_output_meta - t_after_profiling,
                t_output_copy - t_output_meta,
                t_stats_done - t_output_copy,
                t_frame_done - t_stats_done,
                t_frame_done - t_enter
            );
            debug!(
                "pipeline stage=output_frame {}×{} frame={}B",
                ow,
                oh,
                data_out.len()
            );
            let total_duration_ns = (t_frame_done - t_enter).as_nanos() as u64;
            let mut frame = IspFrame::new(ow as u32, oh as u32, FrameFormat::Rgba8888);
            if zero_copy {
                // MNN wrote directly into the caller's buffer; hand back the
                // external pointer so the consumer can display without a memcpy.
                if let Some(&(ptr, _)) = p.output_buffer.as_ref() {
                    frame.data = Vec::new(); // data is empty; external_data owns the pixels
                    frame.external_data = Some(ExternalBuffer {
                        ptr,
                        len: out_elems * graph_elem,
                    });
                } else {
                    frame.data = data_out;
                }
            } else {
                frame.data = data_out;
            }
            // Propagate controller params from ProcessParams to output frame
            frame.params = p.isp_params.clone().unwrap_or_default();
            // Stats are fed into the controller above (not returned in IspAuxOutput).
            // The controller modifies its internal state for the next frame.
            frame.aux = Some(IspAuxOutput {
                calibration_stats: calib_vals,
                stats_measured: measure_stats,
                ..Default::default()
            });
            frame.timestamp_ns = p.timestamp_ns;
            // prep = everything before the MNN call (pool + setup + uploads)
            // infer = the MNN call only; post-inference readback/controller work
            // is excluded here and reported by `pipeline timer` at debug level.
            // total = enter -> frame ready, including post-inference.
            let prep_ns = (t_prep_end - t_enter).as_nanos() as u64;
            frame.prep_duration_ns = prep_ns;
            frame.inference_duration_ns = (t_infer_end - t_infer_start).as_nanos() as u64;
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

#[cfg(all(test, feature = "mnn"))]
mod tests {
    use super::*;

    /// Whether a Vulkan device is available to run the MNN graph on.
    fn gpu_available() -> bool {
        crate::gpu_probe::vulkan_available()
    }

    #[test]
    fn test_mnn_backend_id() {
        assert_eq!(MnnBackend::Vulkan.id(), "mnn_vulkan");
    }

    #[test]
    fn test_mnn_backend_priority() {
        assert_eq!(MnnBackend::Vulkan.priority(), 99);
    }

    #[test]
    fn test_mnn_engine_new() {
        let engine = MnnEngine::new(MnnBackend::Vulkan);
        assert_eq!(engine.backend_name(), "mnn_vulkan");
    }

    #[test]
    fn test_mnn_engine_with_pool_size() {
        let engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 4);
        assert_eq!(engine.backend_name(), "mnn_vulkan");
    }

    #[test]
    fn test_preserve_input_type() {
        let mut engine = MnnEngine::new(MnnBackend::Vulkan);
        assert!(!engine.preserve_input_type());
        engine.set_preserve_input_type(true);
        assert!(engine.preserve_input_type());
    }

    #[test]
    fn test_query_optimal_workgroup() {
        if !gpu_available() {
            return;
        }
        let (wx, wy) = MnnEngine::query_optimal_workgroup();
        assert!(wx > 0 && wy > 0);
    }

    #[test]
    fn test_f32_to_u8_narrows_display_output() {
        // The graph hands back f32 [0,255] for the u8 display formats; the
        // engine must narrow it to one byte per element, not reinterpret it.
        let floats: [f32; 4] = [0.0, 127.5, 255.0, 12.25];
        let raw: Vec<u8> = floats.iter().flat_map(|v| v.to_le_bytes()).collect();
        assert_eq!(MnnEngine::f32_to_u8(&raw), vec![0u8, 127, 255, 12]);
    }

    #[test]
    fn test_f32_to_u8_clamps_out_of_range() {
        // Vulkan FP16 kernels can overshoot on saturated pixels; negatives
        // must clamp to 0 rather than wrap to ~255 on the cast.
        let floats: [f32; 3] = [-8.0, 256.0, 300.5];
        let raw: Vec<u8> = floats.iter().flat_map(|v| v.to_le_bytes()).collect();
        assert_eq!(MnnEngine::f32_to_u8(&raw), vec![0u8, 255, 255]);
    }

    #[test]
    fn test_f32_to_u8_empty() {
        assert!(MnnEngine::f32_to_u8(&[]).is_empty());
    }

    #[test]
    fn test_share_controller_between_engines() {
        let shared = Arc::new(Mutex::new(IspController::new()));
        let mut a = MnnEngine::new(MnnBackend::Vulkan);
        let mut b = MnnEngine::new(MnnBackend::Vulkan);
        a.share_controller(&shared);
        b.share_controller(&shared);

        assert!(Arc::ptr_eq(&a.controller, &shared));
        assert!(Arc::ptr_eq(&b.controller, &shared));
        assert!(Arc::ptr_eq(&a.controller_handle(), &b.controller_handle()));
    }

    #[test]
    fn test_stats_enabled_toggle() {
        let engine = MnnEngine::new(MnnBackend::Vulkan);
        assert!(engine.stats_enabled());
        engine.set_stats_enabled(false);
        assert!(!engine.stats_enabled());
        engine.set_stats_enabled(true);
        assert!(engine.stats_enabled());
    }
}
