//! Minimal MNN C FFI declarations for the ISP Rectifier inference path.
//!
//! These mirror the declarations in `cam-isp/src/mnn_sys.rs` but are kept
//! separate so `isp-rectifier` does not depend on `cam-isp` (which would be a
//! circular dependency: cam-isp optionally depends on isp-rectifier).
//!
//! Only the functions needed for rectifier inference are declared:
//!   - Interpreter creation from buffer
//!   - Session create/release
//!   - Session resize/run
//!   - Input tensor get + upload
//!   - Output tensor read
//!   - Interpreter destroy
//!   - Last error

#![allow(dead_code)]
#![allow(non_camel_case_types)]

pub use std::os::raw::{c_char, c_int, c_void};

/// Opaque handle to an MNN Interpreter (model).
pub struct MnnInterpreter {
    inner: *mut c_void,
}

/// Opaque handle to an MNN Session (inference session).
pub struct MnnSession {
    inner: *mut c_void,
    interpreter: *mut c_void,
}

/// Opaque handle to an MNN Tensor.
pub struct MnnTensor {
    inner: *mut c_void,
}

/// Forward types matching MNNForwardType.h
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MnnBackendType {
    Vulkan = 7,
}

extern "C" {
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

    pub fn mnn_session_read_output(
        interpreter: *mut c_void,
        session: *mut c_void,
        name: *const c_char,
        out: *mut c_void,
        max_bytes: c_int,
    ) -> c_int;

    pub fn mnn_tensor_get_shape(tensor: *mut c_void, dims: *mut c_int, max_dims: c_int) -> c_int;
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

    /// Convert ONNX bytes to MNN flatbuffer bytes (libMNNConvertDeps.so).
    /// Returns 0 on success; out_data/out_size are set (caller frees via libc::free).
    pub fn mnn_convert_onnx_to_mnn_buffer(
        onnx_data: *const c_void,
        onnx_len: usize,
        out_data: *mut *mut c_void,
        out_size: *mut usize,
        error_msg: *mut c_char,
        error_msg_cap: usize,
    ) -> c_int;

    pub fn mnn_last_error() -> *const c_char;
}

/// Safe wrapper around an MNN Interpreter.
impl MnnInterpreter {
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

    pub fn create_session(
        &self,
        backend: MnnBackendType,
        num_threads: i32,
        fp16_storage: bool,
    ) -> Option<MnnSession> {
        let precision = if fp16_storage { 0 } else { 1 };
        let ptr = unsafe {
            mnn_session_create(self.inner, backend, num_threads, precision)
        };
        if ptr.is_null() {
            None
        } else {
            Some(MnnSession {
                inner: ptr,
                interpreter: self.inner,
            })
        }
    }

    pub fn get_input(
        &self,
        session: &MnnSession,
        name: &str,
    ) -> Option<MnnTensor> {
        let c_name = std::ffi::CString::new(name).ok()?;
        let ptr = unsafe {
            mnn_session_get_input_v2(self.inner, session.inner, c_name.as_ptr())
        };
        if ptr.is_null() {
            None
        } else {
            Some(MnnTensor { inner: ptr })
        }
    }

    pub fn get_output(
        &self,
        session: &MnnSession,
        name: &str,
    ) -> Option<MnnTensor> {
        let c_name = std::ffi::CString::new(name).ok()?;
        let ptr = unsafe {
            mnn_session_get_output_v2(self.inner, session.inner, c_name.as_ptr())
        };
        if ptr.is_null() {
            None
        } else {
            Some(MnnTensor { inner: ptr })
        }
    }

    pub fn read_output_floats(
        &self,
        session: &MnnSession,
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
            None
        } else {
            Some(bytes as usize / std::mem::size_of::<f32>())
        }
    }

    pub fn as_ptr(&self) -> *mut c_void {
        self.inner
    }

    pub fn interpreter_ptr(&self) -> *mut c_void {
        self.inner
    }
}

impl Drop for MnnInterpreter {
    fn drop(&mut self) {
        unsafe { mnn_interpreter_destroy(self.inner) };
    }
}

/// Safe wrapper around an MNN Session.
impl MnnSession {
    pub fn resize(&self) -> Result<(), String> {
        let ret = unsafe { mnn_session_resize(self.interpreter, self.inner) };
        if ret == 0 {
            Ok(())
        } else {
            Err("Session resize failed".to_string())
        }
    }

    pub fn run(&self) -> Result<(), String> {
        let ret = unsafe { mnn_session_run(self.interpreter, self.inner) };
        if ret == 0 {
            Ok(())
        } else {
            Err("Session run failed".to_string())
        }
    }

    pub fn as_ptr(&self) -> *mut c_void {
        self.inner
    }

    pub fn interpreter_ptr(&self) -> *mut c_void {
        self.interpreter
    }
}

impl Drop for MnnSession {
    fn drop(&mut self) {
        unsafe { mnn_session_release(self.interpreter, self.inner) };
    }
}

/// Safe wrapper around an MNN Tensor.
impl MnnTensor {
    pub fn shape(&self) -> Vec<i32> {
        let mut dims = [0i32; 8];
        let n = unsafe { mnn_tensor_get_shape(self.inner, dims.as_mut_ptr(), 8) };
        dims[..n as usize].to_vec()
    }

    /// Set the shape of the tensor (requires interpreter + session).
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
                dims.len() as c_int,
            )
        };
        if ret == 0 {
            Ok(())
        } else {
            Err("Tensor set_shape failed".to_string())
        }
    }

    pub fn data_size(&self) -> usize {
        unsafe { mnn_tensor_get_data_size(self.inner) }
    }

    /// Upload host f32 values using an explicit logical shape.
    pub fn upload_f32_shaped(&self, shape: &[i32], vals: &[f32]) -> bool {
        let own;
        let dims = if shape.is_empty() {
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
}
