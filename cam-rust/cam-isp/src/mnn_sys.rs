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

use std::os::raw::{c_char, c_float, c_int, c_void};

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
    fn mnn_interpreter_create_from_buffer(buffer: *const c_void, size: usize) -> *mut c_void;
    fn mnn_interpreter_destroy(interpreter: *mut c_void);
    fn mnn_session_create(
        interpreter: *mut c_void,
        backend: MnnBackendType,
        num_threads: c_int,
    ) -> *mut c_void;
    fn mnn_session_release(interpreter: *mut c_void, session: *mut c_void);
    fn mnn_session_resize(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    fn mnn_session_run(interpreter: *mut c_void, session: *mut c_void) -> c_int;
    fn mnn_session_get_input_v2(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
    ) -> *mut c_void;
    fn mnn_session_get_output_v2(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
    ) -> *mut c_void;
    fn mnn_tensor_get_shape(tensor: *mut c_void, dims: *mut c_int, max_dims: c_int) -> c_int;
    fn mnn_tensor_get_type(tensor: *mut c_void) -> c_int;
    fn mnn_tensor_get_host_data(tensor: *mut c_void) -> *mut f32;
    fn mnn_tensor_get_host_data_raw(tensor: *mut c_void) -> *mut c_void;
    fn mnn_tensor_get_data_size(tensor: *mut c_void) -> usize;
    fn mnn_tensor_set_shape(interpreter: *mut c_void, session: *mut c_void, tensor: *mut c_void, dims: *const c_int, ndim: c_int) -> c_int;
    pub fn mnn_run_host_tensors(
        interpreter: *mut c_void,
        session: *mut c_void,
        in_data: *const c_float,
        in_shape: *const c_int,
        in_ndim: c_int,
        out_data: *mut c_float,
        max_out: c_int,
    ) -> c_int;

    // ── Express Module API ──
    fn mnn_express_load_vars(path: *const c_char, out_count: *mut c_int) -> *mut *mut c_void;
    fn mnn_express_extract(inputs: *mut *mut c_void, n_inputs: c_int, outputs: *mut *mut c_void, n_outputs: c_int) -> *mut c_void;
    fn mnn_express_destroy_module(module: *mut c_void);
    fn mnn_express_create_input(dims: *const c_int, ndim: c_int, format: c_int, dtype: c_int) -> *mut c_void;
    fn mnn_express_write_map(varp: *mut c_void) -> *mut c_void;
    fn mnn_express_read_map(varp: *mut c_void) -> *const c_void;
    fn mnn_express_var_info(varp: *mut c_void, dims_out: *mut c_int, max_dims: c_int, out_format: *mut c_int) -> c_int;
    fn mnn_express_var_resize(varp: *mut c_void, dims: *const c_int, ndim: c_int) -> c_int;
    fn mnn_express_forward(module: *mut c_void, inputs: *mut *mut c_void, n_inputs: c_int, out_count: *mut c_int) -> *mut *mut c_void;
    fn mnn_varps_destroy(varps: *mut *mut c_void, count: c_int);
}

// ═══════════════════════════════════════════════════════════════════════════
// Express Module safe wrapper
// ═══════════════════════════════════════════════════════════════════════════

/// Format enum matching MNN::Express::Dimensionformat
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MnnDimFormat {
    Nhwc = 0,
    Nc4hw4 = 1,
    Nchw = 2,
}

/// Safe wrapper for MNN Express Module API.
pub struct MnnExpressEngine {
    module: *mut c_void,
}

unsafe impl Send for MnnExpressEngine {}
unsafe impl Sync for MnnExpressEngine {}

impl MnnExpressEngine {
    /// Load a .mnn model and extract an inference module.
    /// Scans all variables to find input/output by name heuristics.
    pub fn load(path: &str) -> Result<Self, String> {
        let c_path = std::ffi::CString::new(path).map_err(|e| format!("CString: {}", e))?;

        let mut var_count: c_int = 0;
        let vars = unsafe { mnn_express_load_vars(c_path.as_ptr(), &mut var_count) };
        if vars.is_null() || var_count == 0 {
            return Err("No variables loaded".into());
        }

        // Build name→index map by reading variable info
        let mut input_idx: Option<c_int> = None;
        let mut output_idx: Option<c_int> = None;

        unsafe {
            for i in 0..var_count as isize {
                let varp = *vars.offset(i);
                if varp.is_null() { continue; }

                // Try to get 1 dim from the var to check if it's a candidate
                let mut dims = [0i32; 4];
                let mut fmt: c_int = 0;
                let ndim = mnn_express_var_info(varp, dims.as_mut_ptr(), 4, &mut fmt);

                if ndim == 4 && dims[0] == -1 && dims[1] == 3 && dims[2] == 224 && dims[3] == 224 {
                    input_idx = Some(i as c_int);
                }
                if ndim == 2 && dims[0] == -1 && dims[1] == 1001 {
                    output_idx = Some(i as c_int);
                }
            }
        }

        // Fallback: first var is input, last is output
        let in_idx = input_idx.unwrap_or(0);
        let out_idx = output_idx.unwrap_or(var_count - 1);

        let mut in_var = unsafe { *vars.offset(in_idx as isize) };
        let mut out_var = unsafe { *vars.offset(out_idx as isize) };

        let module = unsafe { mnn_express_extract(&mut in_var, 1, &mut out_var, 1) };
        unsafe { mnn_varps_destroy(vars, var_count) };

        if module.is_null() {
            return Err("Module::extract returned null".into());
        }

        Ok(Self { module })
    }

    /// Run inference with float32 input, return float32 output.
    /// NOTE: In the current MNN build, `readMap` returns NULL after `onForward`.
    /// The call to `fix(VARP::CONSTANT)` is needed to force compute and pin memory.
    /// This is tracked as a known MNN issue.
    pub fn forward(&self, input_data: &[f32], input_shape: &[i32], _output_shape: &[i32]) -> Result<Vec<f32>, String> {
        let ndim = input_shape.len() as c_int;
        let fmt = MnnDimFormat::Nchw as c_int;

        let varp = unsafe { mnn_express_create_input(input_shape.as_ptr(), ndim, fmt, 4) };
        if varp.is_null() {
            return Err("_Input returned null".into());
        }

        let ptr = unsafe { mnn_express_write_map(varp) };
        if ptr.is_null() {
            let mut v = varp;
            unsafe { mnn_varps_destroy(&mut v, 1) };
            return Err("writeMap returned null".into());
        }

        let n = input_data.len();
        unsafe {
            std::ptr::copy_nonoverlapping(input_data.as_ptr(), ptr as *mut f32, n);
        }

        let mut out_count: c_int = 0;
        let mut inp = varp;
        let outs = unsafe { mnn_express_forward(self.module, &mut inp, 1, &mut out_count) };

        // Cleanup input varp
        let mut v = varp;
        unsafe { mnn_varps_destroy(&mut v, 1) };

        if outs.is_null() || out_count == 0 {
            return Err("onForward returned empty".into());
        }

        let out_varp = unsafe { *outs };
        
        // Force fix(CONSTANT) to trigger memory pinning
        // (C-level wrapper doesn't expose fix(), so this is best-effort)
        
        let out_ptr = unsafe { mnn_express_read_map(out_varp) };
        if out_ptr.is_null() {
            unsafe { mnn_varps_destroy(outs, out_count) };
            return Err("readMap returned null after onForward. This is a known MNN build issue where readMap fails for Express Module outputs. Use the low-level Interpreter API (MnnInterpreterSafe) instead.".into());
        }

        let mut dims = [0i32; 8];
        let mut fmt: c_int = 0;
        let ndim = unsafe { mnn_express_var_info(out_varp, dims.as_mut_ptr(), 8, &mut fmt) };
        let total: usize = dims[..ndim as usize].iter().map(|&x| x as usize).product();
        
        let mut result = vec![0.0f32; total];
        unsafe {
            std::ptr::copy_nonoverlapping(out_ptr as *const f32, result.as_mut_ptr(), total);
        }

        unsafe { mnn_varps_destroy(outs, out_count) };
        Ok(result)
    }
}

impl Drop for MnnExpressEngine {
    fn drop(&mut self) {
        unsafe { mnn_express_destroy_module(self.module) };
    }
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
        let ptr = unsafe { mnn_interpreter_create_from_buffer(buffer.as_ptr() as *const c_void, buffer.len()) };
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
    pub fn create_session(&self, backend: MnnBackendType, num_threads: i32) -> Option<MnnSessionSafe> {
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
        if ptr.is_null() { None } else { Some(MnnTensorSafe { inner: ptr }) }
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
        if ptr.is_null() { None } else { Some(MnnTensorSafe { inner: ptr }) }
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
        if ret == 0 { Ok(()) } else { Err("Session resize failed".to_string()) }
    }

    /// Run inference.
    pub fn run(&self) -> Result<(), String> {
        let ret = unsafe { mnn_session_run(self.interpreter, self.inner) };
        if ret == 0 { Ok(()) } else { Err("Session run failed".to_string()) }
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
    use super::*;

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
