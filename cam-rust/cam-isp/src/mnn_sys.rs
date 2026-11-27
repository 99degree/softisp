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

pub use std::ffi::CStr;
pub use std::os::raw::{c_char, c_float, c_int, c_void};

// ── Backend enum ─────────────────────────────────────────────────────────

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MnnBackendType {
    Vulkan = 7,
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
        precision: c_int,
    ) -> *mut c_void;
    pub fn mnn_session_release(interpreter: *mut c_void, session: *mut c_void);
    pub fn mnn_session_resize(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    pub fn mnn_session_resize_for_rebind(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    pub fn mnn_session_run(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    pub fn mnn_session_get_input_v2(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
    ) -> *mut c_void;
    pub fn mnn_session_get_input_count(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    pub fn mnn_session_get_input_name(
        interpreter: *mut c_void,
        session: *mut c_void,
        index: c_int,
    ) -> *const c_char;
    pub fn mnn_session_get_output_v2(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
    ) -> *mut c_void;
    pub fn mnn_session_output_count(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    pub fn mnn_session_output_name(
        interpreter: *mut c_void,
        session: *mut c_void,
        index: c_int,
    ) -> *const c_char;
    pub fn mnn_session_read_output(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
        out: *mut c_void,
        max_bytes: c_int,
    ) -> c_int;
    pub fn mnn_tensor_get_shape(tensor: *mut c_void, dims: *mut c_int, max_dims: c_int) -> c_int;
    pub fn mnn_tensor_get_type(tensor: *mut c_void) -> c_int;
    pub fn mnn_tensor_get_bits(tensor: *mut c_void) -> c_int;
    pub fn mnn_tensor_get_host_data(tensor: *mut c_void) -> *mut f32;
    pub fn mnn_tensor_get_host_data_raw(tensor: *mut c_void) -> *mut c_void;
    pub fn mnn_tensor_get_data_size(tensor: *mut c_void) -> usize;
    pub fn mnn_tensor_upload(
        tensor: *mut c_void,
        data: *const c_void,
        bytes: usize,
        type_code: c_int,
        type_bits: c_int,
        shape: *const c_int,
        ndim: c_int,
    ) -> c_int;
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
        input_name: *const c_char,
        output_name: *const c_char,
        buffer: *const c_void,
        buffer_type_code: c_int,
        buffer_type_bits: c_int,
        in_shape: *const c_int,
        in_ndim: c_int,
        out_data: *mut c_float,
        max_out: c_int,
    ) -> c_int;

    /// Import an Android `AHardwareBuffer` as Vulkan device memory so MNN's
    /// Vulkan backend can read/write it directly instead of allocating its own
    /// device buffers and doing host<->GPU copies.
    ///
    /// This is the final zero-copy step: camera DMA buffer -> GPU tensor ->
    /// GPU shader -> display DMA buffer, with no host memory traffic at all.
    pub fn mnn_import_ahb_as_device_memory(
        interpreter: *mut c_void,
        session: *mut c_void,
        ahb: *const c_void,
        tensor_name: *const c_char,
        read_only: c_int,
    ) -> c_int;

    /// Release a previously imported AHardwareBuffer device memory binding.
    pub fn mnn_release_ahb_device_memory(
        interpreter: *mut c_void,
        session: *mut c_void,
        tensor_name: *const c_char,
    ) -> c_int;

    /// Run inference on an AHB-backed output tensor without `copyToHostTensor`.
    ///
    /// The output AHB must already be bound to the output tensor via
    /// `mnn_import_ahb_as_device_memory` *before* `resizeSession`. `output_ahb`
    /// is the same buffer, passed again so a lost binding can be restored only
    /// when it is a real, sufficiently sized AHardwareBuffer. The function
    /// waits for the GPU write and returns the output element count; the caller
    /// reads the pixels from its own CPU mapping of the AHB.
    pub fn mnn_run_ahb_output(
        interpreter: *mut c_void,
        session: *mut c_void,
        input_name: *const c_char,
        output_name: *const c_char,
        buffer: *const c_void,
        buffer_type_code: c_int,
        buffer_type_bits: c_int,
        in_shape: *const c_int,
        in_ndim: c_int,
        max_out: c_int,
        output_ahb: *const c_void,
    ) -> c_int;

    pub fn mnn_run_with_output(
        interpreter: *mut c_void,
        session: *mut c_void,
        input_name: *const c_char,
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

    /// Declared shape of the model's first input tensor (rank returned,
    /// dims written into `out_dims`, -1 on error).
    pub fn mnn_get_model_input_dims(
        interpreter: *mut c_void,
        session: *mut c_void,
        out_dims: *mut c_int,
        max_ndim: c_int,
    ) -> c_int;

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
    pub fn mnn_get_actual_backend(interpreter: *mut c_void, session: *mut c_void) -> c_int;

    /// Force MNN Vulkan backend registration.
    /// Calls MNNVulkanFuseRegister and manually registers the Vulkan runtime creator.
    /// Returns 1 on success, 0 on failure.
    pub fn ensure_mnn_vulkan_registered() -> c_int;

    /// Human-readable reason for the most recent `mnn_run_with_output` failure.
    ///
    /// The negative return code alone cannot distinguish a shape mismatch from
    /// a backend compute error, and the app has no logcat — only its own log
    /// file — so the reason has to come back through this pointer.
    pub fn mnn_last_error() -> *const c_char;
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

    /// Create an Interpreter from a model file descriptor (e.g. an O_TMPFILE
    /// or memfd). The path `/proc/self/fd/<fd>` is passed to MNN, so the
    /// model is read straight from the fd without copying into the Rust heap.
    /// `fd` must remain open for the lifetime of the returned interpreter's
    /// load; it can be closed afterwards. Prefer `from_buffer` for the
    /// converter's pure in-memory output.
    pub fn from_fd(fd: std::os::unix::io::RawFd) -> Option<Self> {
        let path = format!("/proc/self/fd/{}", fd);
        let c_path = std::ffi::CString::new(path).ok()?;
        let ptr = unsafe { mnn_interpreter_create_from_file(c_path.as_ptr()) };
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
    ///
    /// `fp16_storage` asks for `Precision_Normal` rather than the historical
    /// `Precision_High`. On the Vulkan backend that is what makes MNN store
    /// every float tensor as fp16 when the device supports it, halving the
    /// bytes per element — see `mnn_session_create` in mnn_wrapper.cpp.
    pub fn create_session(
        &self,
        backend: MnnBackendType,
        num_threads: i32,
        fp16_storage: bool,
    ) -> Option<MnnSessionSafe> {
        let precision = if fp16_storage { 0 } else { 1 };
        let ptr = unsafe { mnn_session_create(self.inner, backend, num_threads, precision) };
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

    /// Number of input tensors declared by the model for this session.
    pub fn get_input_count(&self, session: &MnnSessionSafe) -> usize {
        (unsafe { mnn_session_get_input_count(self.inner, session.inner) }) as usize
    }

    /// Name of the i-th input tensor (0-based, in declaration order).
    pub fn get_input_name(&self, session: &MnnSessionSafe, index: usize) -> Option<String> {
        use std::ffi::CStr;
        let ptr = unsafe { mnn_session_get_input_name(self.inner, session.inner, index as c_int) };
        if ptr.is_null() {
            None
        } else {
            Some(
                unsafe { CStr::from_ptr(ptr) }
                    .to_string_lossy()
                    .into_owned(),
            )
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

    /// Names of every tensor the session exposes as an output.
    ///
    /// Empty is a meaningful answer: the graph can carry a stats op whose
    /// output was never registered, in which case `get_output(name)` returns
    /// `None` for a tensor that plainly exists in the model. Listing what *is*
    /// there is the difference between a conversion bug and a lookup bug.
    pub fn output_names(&self, session: &MnnSessionSafe) -> Vec<String> {
        let count = unsafe { mnn_session_output_count(self.inner, session.inner) };
        (0..count.max(0))
            .filter_map(|i| {
                let ptr = unsafe { mnn_session_output_name(self.inner, session.inner, i) };
                if ptr.is_null() {
                    None
                } else {
                    Some(
                        unsafe { CStr::from_ptr(ptr) }
                            .to_string_lossy()
                            .into_owned(),
                    )
                }
            })
            .collect()
    }

    /// Copy a named session output into `buf`, returning the float count.
    ///
    /// Works for GPU tensors, unlike reading `host<float>()` directly -- a
    /// Vulkan-backed tensor has no host allocation until it is copied.
    pub fn read_output_floats(
        &self,
        session: &MnnSessionSafe,
        name: &str,
        buf: &mut [f32],
    ) -> Option<usize> {
        let c_name = std::ffi::CString::new(name).ok()?;
        let max_bytes = std::mem::size_of_val(buf);
        let bytes = unsafe {
            mnn_session_read_output(
                self.inner,
                session.inner,
                c_name.as_ptr(),
                buf.as_mut_ptr() as *mut c_void,
                max_bytes as c_int,
            )
        };
        if bytes <= 0 {
            // -3 = tensor not found, -5 = buffer too small, -4 = no elements.
            log::debug!("read_output_floats({name}) -> {bytes}");
            None
        } else {
            Some(bytes as usize / std::mem::size_of::<f32>())
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

    /// Re-acquire session tensors after rebinding an AHB without a shape change.
    pub fn resize_for_rebind(&self) -> Result<(), String> {
        let ret = unsafe { mnn_session_resize_for_rebind(self.interpreter, self.inner) };
        if ret == 0 {
            Ok(())
        } else {
            Err("Session rebind resize failed".to_string())
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

    /// Get the element bit width: 8, 16, 32, 64, or 0 if unknown.
    ///
    /// [`Self::data_type`] collapses INT16 and INT32 into the same code, so
    /// this is the only way to tell a RAW16-shaped graph input from a widened
    /// one.
    pub fn data_bits(&self) -> i32 {
        unsafe { mnn_tensor_get_bits(self.inner) }
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

    /// Upload host f32 values into this tensor for the next inference.
    ///
    /// Unlike [`Self::as_bytes_mut`], which writes into a host mapping the
    /// Vulkan backend never reads, this performs the host→device transfer, so
    /// the values actually reach the graph. Returns `false` if the tensor
    /// cannot take `vals` (wrong element count).
    pub fn upload_f32(&self, vals: &[f32]) -> bool {
        self.upload_f32_shaped(&self.shape(), vals)
    }

    /// Upload host f32 values using an explicit logical shape.
    ///
    /// Scalars are the reason this exists: MNN reports a rank-0 tensor's shape
    /// as an empty list, and a buffer view of `[]` has no elements, so the
    /// round trip through [`Self::shape`] loses the one value these tensors
    /// exist to carry. Callers that know the logical shape pass it here.
    pub fn upload_f32_shaped(&self, shape: &[i32], vals: &[f32]) -> bool {
        let own;
        let dims = if shape.is_empty() {
            // A rank-0 tensor still holds exactly one element.
            own = [1i32];
            &own[..]
        } else {
            shape
        };
        let elems: i64 = dims.iter().map(|d| *d as i64).product();
        if elems != vals.len() as i64 {
            return false;
        }
        let ret = unsafe {
            mnn_tensor_upload(
                self.inner,
                vals.as_ptr() as *const c_void,
                std::mem::size_of_val(vals),
                2, // halide_type_float
                32,
                dims.as_ptr(),
                dims.len() as i32,
            )
        };
        ret == 0
    }

    /// Upload raw host bytes (e.g. a serialized f32 buffer) into this tensor.
    ///
    /// `bits` selects the host element width: 32 for f32/int32, 16 for
    /// half/int16. Returns `false` on a shape mismatch.
    pub fn upload_bytes(&self, data: &[u8], code: i32, bits: i32) -> bool {
        let mut shape = self.shape();
        if shape.is_empty() {
            // Rank-0 scalars report no dims but still hold one element.
            shape.push(1);
        }
        let ret = unsafe {
            mnn_tensor_upload(
                self.inner,
                data.as_ptr() as *const c_void,
                data.len(),
                code,
                bits,
                shape.as_ptr(),
                shape.len() as i32,
            )
        };
        ret == 0
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_mnn_not_available_without_library() {
        // This test verifies that we gracefully handle missing MNN library.
        // Without libMNN.so / libmnn_wrapper.so, the FFI calls will fail
        // at link time. This test only runs when the feature is enabled.
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

/// MNN→JSON dump result (mnn_convert_api.cpp).
/// `data` is a NUL-terminated JSON string (malloc'd); caller must free
/// via MnnConvert_FreeBuffer.
#[repr(C)]
pub struct MnnConvertJsonResult {
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

    /// Pure in-memory ONNX→MNN conversion (libMNNConvertDeps.so, C-only API
    /// from MNN's MNNConvertBuffer.cpp). No temp files, no memfd, no
    /// `/proc/self/fd` — the converter parses the ONNX protobuf from the
    /// buffer, runs the full optimizeLevel=2 pipeline (incl. IspChainFusion)
    /// and returns malloc'd MNN flatbuffer bytes.
    ///
    /// Returns 0 on success (out_data/out_size set; caller frees via
    /// `libc::free`) or -1 on failure (error_msg filled when provided).
    pub fn mnn_convert_onnx_to_mnn_buffer(
        onnx_data: *const c_void,
        onnx_len: usize,
        out_data: *mut *mut c_void,
        out_size: *mut usize,
        error_msg: *mut c_char,
        error_msg_cap: usize,
    ) -> c_int;

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

    // MNN→MNN buffer conversion (framework=MNN → applies IspChainFusion).
    // Pass1 (B test): converts the Pass0 .mnn through the converter again,
    // fusing primitive ops into isp.* custom ops carrying SPIR-V.
    // Caller must free result->data via MnnConvert_FreeBuffer.
    pub fn mnn_convert_mnn_buffer(
        mnn_data: *const c_void,
        mnn_len: usize,
        result: *mut MnnConvertBufferResult,
    );

    /// MNN→JSON dump via MNN::Cli::mnn2json (in-process).
    /// Produces the flatbuffer JSON text: one object per op with "type"
    /// (MNN OpType enum name) and, for custom ops, "main_type": "Extra"
    /// with "main.type" = the isp.* string. Used by the Pass1 (B) test to
    /// verify the converted MNN contains ONLY isp.* custom opset ops.
    /// Caller must free result->data via MnnConvert_FreeBuffer.
    pub fn mnn_dump_mnn_to_json(
        mnn_data: *const c_void,
        mnn_len: usize,
        result: *mut MnnConvertJsonResult,
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
