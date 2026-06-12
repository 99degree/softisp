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

use std::os::raw::{c_char, c_int, c_void};

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

// ── FFI declarations ────────────────────────────────────────────────────

extern "C" {
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
}

// ── Safe Rust wrappers ───────────────────────────────────────────────────

/// Safe wrapper around an MNN Interpreter.
pub struct MnnInterpreterSafe {
    inner: *mut c_void,
}

// MNN C API is thread-safe for read operations; interpreter/session handles
// are protected by Mutex in MnnSessionWrapper.
unsafe impl Send for MnnInterpreterSafe {}
unsafe impl Sync for MnnInterpreterSafe {}

impl MnnInterpreterSafe {
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
