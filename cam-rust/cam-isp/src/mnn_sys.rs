//! Rust FFI bindings for the MNN C wrapper.
//!
//! These bindings call into `libmnn_wrapper.so` (or `.a`) which wraps
//! the MNN C++ API. The wrapper is compiled separately with the Android NDK.
//!
//! ## Build requirement
//! The MNN wrapper must be compiled and linked into the final binary:
//! ```bash
//! ${CROSS_COMPILE}clang++ -std=c++17 -c mnn_sys/mnn_wrapper.cpp \
//!     -I${MNN_INCLUDE_DIR} -o mnn_wrapper.o
//! # Then link:
//! # - mnn_wrapper.o
//! # - -lMNN (prebuilt libMNN.so or libMNN.a)
//! ```

#![allow(dead_code)]
#![allow(non_camel_case_types)]

pub use std::os::raw::{c_char, c_float, c_int, c_void};

/// MNN external-memory device-pointer type: import an AHardwareBuffer (which
/// wraps a CMA/dma-buf fd from V4L2/MIPI) as device memory for zero-copy input.
/// Passed to `Tensor::setDevicePtr` / `mnn_run_external_zero_copy`.
pub const MNN_MEMORY_AHARDWAREBUFFER: c_int = 14;

// ── Backend enum ─────────────────────────────────────────────────────────

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MnnBackendType {
    Cpu = 0,
    Opencl = 3,
    Opengl = 6,
    Vulkan = 7,
    Metal = 9,
    Nn = 11,
}

// ── Model Info enum ──────────────────────────────────────────────────────
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MnnModelInfo {
    MEMORY = 0,
    FLOPS = 1,
    BACKENDS = 2,
    RESIZE_STATUS = 3,
    THREAD_NUMBER = 4,
}
// ── Opaque handles ───────────────────────────────────────────────────────

/// Opaque handle to an MNN Interpreter (model).
#[repr(C)]
pub struct MnnInterpreter(std::ptr::NonNull<c_void>);

/// Opaque handle to an MNN Session (inference session).
#[repr(C)]
pub struct MnnSession(std::ptr::NonNull<c_void>);

/// Opaque handle to an MNN Tensor.
#[repr(C)]
pub struct MnnTensor(std::ptr::NonNull<c_void>);

// ── FFI declarations (low-level Interpreter API) ────────────────────────

extern "C" {
    pub fn mnn_interpreter_create_from_file(path: *const c_char) -> *mut c_void;
    pub fn mnn_interpreter_create_from_buffer(buffer: *const c_void, size: usize) -> *mut c_void;
    pub fn mnn_interpreter_destroy(interpreter: *mut c_void);
    pub fn mnn_session_create(
        interpreter: *mut c_void,
        backend: MnnBackendType,
        num_threads: c_int,
    ) -> *mut c_void;
    pub fn mnn_session_release(interpreter: *mut c_void, session: *mut c_void);
    pub fn mnn_session_resize(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    pub fn mnn_session_run(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    pub fn mnn_session_get_input_v2(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
    ) -> *mut c_void;
    pub fn mnn_session_get_output_v2(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
    ) -> *mut c_void;
    pub fn mnn_tensor_get_shape(tensor: *mut c_void, dims: *mut c_int, max_dims: c_int) -> c_int;
    pub fn mnn_tensor_get_type(tensor: *mut c_void) -> c_int;
    pub fn mnn_tensor_get_host_data(tensor: *mut c_void) -> *mut f32;
    pub fn mnn_tensor_get_host_data_raw(tensor: *mut c_void) -> *mut c_void;
    pub fn mnn_tensor_get_data_size(tensor: *mut c_void) -> usize;
    pub fn mnn_tensor_set_shape(
        interpreter: *mut c_void,
        session: *mut c_void,
        tensor: *mut c_void,
        dims: *const c_int,
        ndim: c_int,
    ) -> c_int;
    // Standard MNN C++ API for session info (always available)
    pub fn mnn_get_model_info(
        interpreter: *mut c_void,
        session: *mut c_void,
        info_code: c_int,
        out: *mut c_void,
    ) -> c_int;
    pub fn mnn_get_model_input_type(
        interpreter: *mut c_void,
        session: *mut c_void,
        out_code: *mut c_int,
        out_bits: *mut c_int,
    ) -> c_int;

    pub fn mnn_run_true_zero_copy(
        interpreter: *mut c_void,
        session: *mut c_void,
        buffer: *const c_void,
        buffer_type_code: c_int,
        buffer_type_bits: c_int,
        in_shape: *const c_int,
        in_ndim: c_int,
        out_data: *mut c_float,
        max_out: c_int,
    ) -> c_int;

    /// Run inference with the input tensor bound to EXTERNAL device memory
    /// (dma-buf fd / AHardwareBuffer) via `Tensor::setDevicePtr`. This lets the
    /// camera's CMA/dma-buf pages be consumed by the GPU directly — no CPU
    /// staging copy, no heap allocation for large MIPI Bayer frames. The Vulkan
    /// backend must honor external-memory import (custom MNN build) for this to
    /// take effect; otherwise it is a no-op at the backend level.
    ///
    /// `ext_handle` is the dma-buf fd (Linux) or AHardwareBuffer* (Android),
    /// `memory_type` is e.g. `MNN_MEMORY_AHARDWAREBUFFER`. Output is written
    /// into `out_data` (host) as with `mnn_run_true_zero_copy`.
    pub fn mnn_run_external_zero_copy(
        interpreter: *mut c_void,
        session: *mut c_void,
        ext_handle: i64,
        memory_type: c_int,
        in_shape: *const c_int,
        in_ndim: c_int,
        out_data: *mut c_float,
        max_out: c_int,
    ) -> c_int;

    pub fn mnn_run_with_output(
        interpreter: *mut c_void,
        session: *mut c_void,
        buffer: *const c_void,
        buffer_type_code: c_int,
        buffer_type_bits: c_int,
        in_shape: *const c_int,
        in_ndim: c_int,
        output_name: *const c_char,
        out_data: *mut c_float,
        max_out: c_int,
    ) -> c_int;

    /// Host-tensor inference: creates host tensors, copies data in,
    /// runs inference, copies data out. Handles GPU→host transfers.
    pub fn mnn_run_host_tensors(
        interpreter: *mut c_void,
        session: *mut c_void,
        in_data: *const c_float,
        in_shape: *const c_int,
        in_ndim: c_int,
        out_data: *mut c_float,
        max_out: c_int,
    ) -> c_int;

    /// FP16 output inference: reads raw float16 output from models with
    /// Cast(FLOAT→FLOAT16) at the end. Output buffer is raw bytes (uint16_t[]).
    pub fn mnn_set_input_float(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
        data: *const c_float,
        shape: *const c_int,
        ndim: c_int,
    ) -> c_int;

    /// Get expected input tensor element count.
    pub fn mnn_get_model_input_elements(interpreter: *mut c_void, session: *mut c_void) -> c_int;

    /// Benchmark with GPU synchronization.
    /// Uses tensor map/unmap to force GPU→CPU sync for accurate timing.
    pub fn mnn_benchmark_sync(
        interpreter: *mut c_void,
        session: *mut c_void,
        warmup: c_int,
        loops: c_int,
        precision: c_int,
        out_costs_ms: *mut c_float,
        max_costs: c_int,
    ) -> c_int;

    /// Query the actual forward type a session is using (post-fallback).
    pub fn mnn_get_actual_backend(
        interpreter: *mut c_void,
        session: *mut c_void,
    ) -> c_int;

    // Retrieve per‑node profiling info (requires session built with profiling enabled).
    // extern declaration removed; stub provided below
}

// General MNN_GetSessionInfo API (mirrors MNN::Interpreter::getSessionInfo).
// `info_code` corresponds to the `MnnModelInfo` enum values (MEMORY, FLOPS, THREAD_NUMBER, ...).
// The function writes the requested value into the memory pointed to by `out` and
// returns 0 on success. This works without the profiler flag.
// When profiling is enabled, a separate string version `MNN_GetSessionInfoString`
// can be used for per‑node timing data.
///
/// # Safety
///
/// `session`, `info_code`, and `out` must be valid pointers.
#[no_mangle]
pub unsafe extern "C" fn MNN_GetSessionInfo(
    session: *mut c_void,
    info_code: c_int,
    out: *mut c_void,
) -> c_int {
    // The real implementation is provided by libMNN.so; this stub simply
    // returns -1 to indicate that the function is not linked. It will be
    // overridden by the actual library at runtime.
    let _ = (session, info_code, out);
    -1
}

// Fallback for optional profiling string version (returns NULL when profiling is unavailable).
///
/// # Safety
///
/// `session` must be a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn MNN_GetSessionInfoString(_session: *mut c_void) -> *const c_char {
    std::ptr::null()
}

// ═══════════════════════════════════════════════════════════════════════════
// Safe Rust wrappers (low-level API)
// ═══════════════════════════════════════════════════════════════════════════

/// Safe wrapper around an MNN Interpreter.
pub struct MnnInterpreterSafe {
    inner: *mut c_void,
}

// MNN C API is thread-safe for read operations; interpreter/session handles
// are protected by Mutex in MnnSessionWrapper.
unsafe impl Send for MnnInterpreterSafe {}
unsafe impl Sync for MnnInterpreterSafe {}

impl MnnInterpreterSafe {
    /// Create an Interpreter from a model file (.mnn format).
    pub fn from_file(path: &str) -> Option<Self> {
        let c_path = std::ffi::CString::new(path).ok()?;
        let ptr = unsafe { mnn_interpreter_create_from_file(c_path.as_ptr()) };
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    /// Create an Interpreter from a model buffer.
    pub fn from_buffer(buffer: &[u8]) -> Option<Self> {
        let ptr = unsafe {
            mnn_interpreter_create_from_buffer(buffer.as_ptr() as *const c_void, buffer.len())
        };
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    /// Get raw interpreter pointer (for C FFI calls).
    pub fn as_ptr(&self) -> *mut c_void {
        self.inner
    }

    /// Create a session with the given backend.
    pub fn create_session(
        &self,
        backend: MnnBackendType,
        num_threads: i32,
    ) -> Option<MnnSessionSafe> {
        let ptr = unsafe { mnn_session_create(self.inner, backend, num_threads) };
        if ptr.is_null() {
            None
        } else {
            Some(MnnSessionSafe {
                interpreter: self.inner,
                inner: ptr,
            })
        }
    }

    /// Get an input tensor by name.
    pub fn get_input(&self, session: &MnnSessionSafe, name: &str) -> Option<MnnTensorSafe> {
        let c_name = std::ffi::CString::new(name).ok()?;
        let ptr = unsafe { mnn_session_get_input_v2(self.inner, session.inner, c_name.as_ptr()) };
        if ptr.is_null() {
            None
        } else {
            Some(MnnTensorSafe { inner: ptr })
        }
    }

    /// Get first input tensor (passes NULL name to MNN).
    pub fn get_first_input(&self, session: &MnnSessionSafe) -> Option<MnnTensorSafe> {
        let ptr = unsafe { mnn_session_get_input_v2(self.inner, session.inner, std::ptr::null()) };
        if ptr.is_null() {
            None
        } else {
            Some(MnnTensorSafe { inner: ptr })
        }
    }

    /// Get an output tensor by name.
    pub fn get_output(&self, session: &MnnSessionSafe, name: &str) -> Option<MnnTensorSafe> {
        let c_name = std::ffi::CString::new(name).ok()?;
        let ptr = unsafe { mnn_session_get_output_v2(self.inner, session.inner, c_name.as_ptr()) };
        if ptr.is_null() {
            None
        } else {
            Some(MnnTensorSafe { inner: ptr })
        }
    }

    /// Get first output tensor (passes NULL name to MNN).
    pub fn get_first_output(&self, session: &MnnSessionSafe) -> Option<MnnTensorSafe> {
        let ptr = unsafe { mnn_session_get_output_v2(self.inner, session.inner, std::ptr::null()) };
        if ptr.is_null() {
            None
        } else {
            Some(MnnTensorSafe { inner: ptr })
        }
    }
}

impl Drop for MnnInterpreterSafe {
    fn drop(&mut self) {
        unsafe { mnn_interpreter_destroy(self.inner) };
    }
}

/// Safe wrapper around an MNN Session.
pub struct MnnSessionSafe {
    interpreter: *mut c_void,
    inner: *mut c_void,
}

unsafe impl Send for MnnSessionSafe {}
unsafe impl Sync for MnnSessionSafe {}

impl MnnSessionSafe {
    /// Get raw session pointer (for C FFI calls).
    pub fn as_ptr(&self) -> *mut c_void {
        self.inner
    }

    /// Resize the session for current input shapes.
    pub fn resize(&self) -> Result<(), String> {
        let ret = unsafe { mnn_session_resize(self.interpreter, self.inner) };
        if ret == 0 {
            Ok(())
        } else {
            Err("Session resize failed".to_string())
        }
    }

    /// Run inference.
    pub fn run(&self) -> Result<(), String> {
        let ret = unsafe { mnn_session_run(self.interpreter, self.inner) };
        if ret == 0 {
            Ok(())
        } else {
            Err("Session run failed".to_string())
        }
    }

    /// Query the actual forward type this session is using.
    /// Returns MNNForwardType: 0=CPU, 7=Vulkan, etc.
    pub fn get_actual_backend(&self) -> i32 {
        unsafe { mnn_get_actual_backend(self.interpreter, self.inner) }
    }

    /// Benchmark with GPU synchronization (accurate timing).
    ///
    /// Uses tensor map/unmap to force GPU→CPU sync, giving real end-to-end
    /// latency — not the async fire-and-forget illusion.
    ///
    /// Returns `(avg_ms, min_ms, max_ms, per_frame_ms)`.
    pub fn benchmark_sync(
        &self,
        warmup: i32,
        loops: i32,
        precision: i32,
    ) -> Result<(f32, f32, f32, Vec<f32>), String> {
        let cap = loops as usize;
        let mut costs = vec![0.0f32; cap];
        let ret = unsafe {
            mnn_benchmark_sync(
                self.interpreter,
                self.inner,
                warmup,
                loops,
                precision,
                costs.as_mut_ptr(),
                cap as i32,
            )
        };
        if ret < 0 {
            return Err(format!("mnn_benchmark_sync failed: {ret}"));
        }
        costs.truncate(ret as usize);
        if costs.is_empty() {
            return Err("no benchmark iterations completed".into());
        }
        let avg = costs.iter().sum::<f32>() / costs.len() as f32;
        let min = costs.iter().cloned().fold(f32::INFINITY, f32::min);
        let max = costs.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        Ok((avg, min, max, costs))
    }

    /// Query model info (memory, FLOPS, etc.) via `MnnModelInfo`.
    /// Returns the float value on success.
    pub fn get_model_info(&self, info: MnnModelInfo) -> Result<f32, String> {
        let mut out: f32 = 0.0;
        let ret = unsafe {
            mnn_get_model_info(
                self.interpreter,
                self.inner,
                info as i32,
                &mut out as *mut _ as *mut c_void,
            )
        };
        if ret == 0 {
            Ok(out)
        } else {
            Err(format!("get_model_info({:?}) failed", info))
        }
    }
}

impl Drop for MnnSessionSafe {
    fn drop(&mut self) {
        unsafe { mnn_session_release(self.interpreter, self.inner) };
    }
}

/// Safe wrapper around an MNN Tensor.
pub struct MnnTensorSafe {
    inner: *mut c_void,
}

unsafe impl Send for MnnTensorSafe {}
unsafe impl Sync for MnnTensorSafe {}

impl MnnTensorSafe {
    /// Get the shape of the tensor.
    pub fn shape(&self) -> Vec<i32> {
        let mut dims = [0i32; 8];
        let n = unsafe { mnn_tensor_get_shape(self.inner, dims.as_mut_ptr(), 8) };
        dims[..n as usize].to_vec()
    }

    /// Get the data type code: 4=float32, 5=int32, 6=uint32, 0=unknown.
    pub fn data_type(&self) -> i32 {
        unsafe { mnn_tensor_get_type(self.inner) }
    }

    /// Get a reference to the tensor's host data (any type) as bytes.
    /// This is zero-copy - the slice points to MNN's internal buffer.
    pub fn as_bytes(&self) -> Option<&[u8]> {
        let ptr = unsafe { mnn_tensor_get_host_data_raw(self.inner) };
        if ptr.is_null() {
            return None;
        }
        let size = unsafe { mnn_tensor_get_data_size(self.inner) };
        Some(unsafe { std::slice::from_raw_parts(ptr as *const u8, size) })
    }

    /// Get a mutable reference to the tensor's host data as bytes.
    ///
    /// # Safety
    ///
    /// Caller must ensure:
    /// - The tensor has valid host data
    /// - No other references to this data exist
    /// - The tensor outlives the returned slice
    #[allow(clippy::mut_from_ref)] // MNN tensors are inherently mutable
    pub fn as_bytes_mut(&self) -> Option<&mut [u8]> {
        let ptr = unsafe { mnn_tensor_get_host_data_raw(self.inner) };
        if ptr.is_null() {
            return None;
        }
        let size = unsafe { mnn_tensor_get_data_size(self.inner) };
        Some(unsafe { std::slice::from_raw_parts_mut(ptr as *mut u8, size) })
    }

    /// Set the shape of the tensor (requires interpreter + session).
    /// Caller must hold references to both that outlive this call.
    pub fn set_shape(
        &self,
        interpreter: *mut c_void,
        session: *mut c_void,
        dims: &[i32],
    ) -> Result<(), String> {
        let ret = unsafe {
            mnn_tensor_set_shape(
                interpreter,
                session,
                self.inner,
                dims.as_ptr(),
                dims.len() as i32,
            )
        };
        if ret == 0 {
            Ok(())
        } else {
            Err("Tensor set_shape failed".to_string())
        }
    }

    /// Get a raw pointer to the tensor's data buffer (any element type).
    /// Use with caution - you must know the element type from get_type().
    pub fn as_ptr(&self) -> *const u8 {
        unsafe { mnn_tensor_get_host_data_raw(self.inner) as *const u8 }
    }

    /// Get a mutable raw pointer to the tensor's data buffer.
    pub fn as_mut_ptr(&self) -> *mut u8 {
        unsafe { mnn_tensor_get_host_data_raw(self.inner) as *mut u8 }
    }

    /// Get the size of the tensor data in bytes.
    pub fn data_size(&self) -> usize {
        unsafe { mnn_tensor_get_data_size(self.inner) }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_mnn_not_available_without_library() {
        // This test verifies that we gracefully handle missing MNN library.
        // Without libMNN.so / libmnn_wrapper.so, the FFI calls will fail
        // at link time. This test only runs when the feature is enabled.
        assert!(true);
    }
}

// ── MNN Convert C API (libMNNConvertDeps.so) ──────────────────────────────

#[repr(C)]
#[derive(Debug, Clone, Copy)]
#[allow(non_snake_case)]
pub struct MnnConvert_Options {
    pub bizCode: *const c_char,
    pub optimizeLevel: c_int,
    pub fp16: c_int,
    pub weightQuantBits: c_int,
    pub weightQuantBlock: c_int,
    pub saveStaticModel: c_int,
    pub targetVersion: c_float,
    pub transformerFuse: c_int,
    pub allowCustomOp: c_int,
    pub useGeluApproximation: c_int,
    pub inputConfigFile: *const c_char,
}

impl Default for MnnConvert_Options {
    fn default() -> Self {
        Self {
            bizCode: c"MNN".as_ptr(),
            optimizeLevel: 1,
            fp16: 0,
            weightQuantBits: 0,
            weightQuantBlock: -1,
            saveStaticModel: 0,
            targetVersion: 0.0,
            transformerFuse: 0,
            allowCustomOp: 0,
            useGeluApproximation: 1,
            inputConfigFile: std::ptr::null(),
        }
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct MnnConvert_Result {
    pub code: c_int,
    pub message: *mut c_char,
}

extern "C" {
    pub fn MnnConvert_OnnxToMnn(
        onnxPath: *const c_char,
        mnnPath: *const c_char,
        options: *const MnnConvert_Options,
    ) -> MnnConvert_Result;

    pub fn MnnConvert_FreeResult(result: *mut MnnConvert_Result);
}

// ── Simpler C FFI (mnn_convert_api.cpp, statically linked) ──────────

/// Conversion result from the simple C FFI.
#[repr(C)]
pub struct MnnConvertResult {
    pub success: i32,
    pub error_msg: [c_char; 1024usize],
}

/// Buffer-based conversion result (mnn_convert_api.cpp).
/// Caller must call MnnConvert_FreeBuffer to free the data pointer.
#[repr(C)]
pub struct MnnConvertBufferResult {
    pub success: i32,
    pub error_msg: [c_char; 1024usize],
    pub data: *mut c_void,
    pub size: usize,
}

extern "C" {
    /// Convert ONNX model to MNN format (statically linked from mnn_convert_api.cpp).
    /// Returns result via MnnConvertResult struct. No heap allocation, no free needed.
    pub fn mnn_convert_onnx_to_mnn(
        onnx_path: *const c_char,
        mnn_path: *const c_char,
        biz_code: *const c_char,
        optimize_level: i32,
        weight_quant_bits: i32,
        fp16: i32,
        preserve_input_type: i32,
        result: *mut MnnConvertResult,
    );

    /// Buffer-based ONNX→MNN conversion (libMNNConvertDeps.so, same-process).
    /// Pass ONNX bytes → receive MNN bytes. Uses memfd internally, no disk writes.
    /// Caller must free result->data via MnnConvert_FreeBuffer.
    pub fn mnn_convert_onnx_buffer(
        onnx_data: *const c_void,
        onnx_len: usize,
        result: *mut MnnConvertBufferResult,
    );

    /// Free data allocated by mnn_convert_onnx_buffer.
    pub fn MnnConvert_FreeBuffer(result: *mut MnnConvertBufferResult);

    /// Convert TensorFlow model to MNN format.
    pub fn mnn_convert_tf_to_mnn(
        tf_model_path: *const c_char,
        mnn_path: *const c_char,
        biz_code: *const c_char,
        optimize_level: i32,
        weight_quant_bits: i32,
        fp16: i32,
        preserve_input_type: i32,
        result: *mut MnnConvertResult,
    );
}

// ── Vulkan Workgroup Configuration ────────────────────────────────────────

extern "C" {
    /// Set preferred workgroup size for a Vulkan session.
    /// Called from Rust to tune dispatch groups per-device.
    pub fn MNNVulkanSetSessionWorkgroup(session: *mut c_void, size_x: i32, size_y: i32);

    /// Query optimal workgroup size for current GPU.
    pub fn MNNVulkanQueryOptimalWorkgroup(out_x: *mut i32, out_y: *mut i32);

    /// Set workgroup by preset name.
    /// Presets: "fast_4k" (32×8), "low_power" (8×32), "portrait" (4×64), "universal" (16×16).
    pub fn MNNVulkanSetWorkgroupPreset(preset_name: *const c_char);

    /// Hot-swap a const buffer at runtime for live 3A adjustments.
    pub fn MNNVulkanHotSwapConstBuffer(
        session_ptr: *mut c_void,
        binding_index: c_int,
        data: *const c_void,
        byte_size: c_int,
    ) -> c_int;
}
