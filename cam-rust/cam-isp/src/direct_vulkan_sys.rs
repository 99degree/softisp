//! Minimal FFI for the standalone Vulkan RAW16→BGRA compute module.
//!
//! The module is intentionally independent of MNN. It imports AHardwareBuffer
//! handles as Vulkan external memory and binds them to `VkBuffer` objects, so
//! the input and output remain opaque byte containers.

use std::ffi::{c_char, c_int, c_void, CStr};
use std::ptr;

extern "C" {
    fn dv_context_create() -> *mut c_void;
    fn dv_context_destroy(ctx: *mut c_void);
    fn dv_run_ahb(
        ctx: *mut c_void,
        input_ahb: *mut c_void,
        input_len: usize,
        output_ahb: *mut c_void,
        output_len: usize,
        width: u32,
        height: u32,
        mode: u32,
    ) -> c_int;
    fn dv_last_error() -> *const c_char;
}

fn last_error() -> String {
    unsafe {
        let ptr = dv_last_error();
        if ptr.is_null() {
            return "unknown direct Vulkan error".to_string();
        }
        CStr::from_ptr(ptr).to_string_lossy().into_owned()
    }
}

/// One Vulkan device/pipeline owned by a single ISP worker.
pub struct DirectVulkanContext {
    ctx: *mut c_void,
}

// The context is touched only by the worker that created it. The marker is
// needed because `IspEngine` requires `Send + Sync` and the engine moves to its
// worker thread after construction.
unsafe impl Send for DirectVulkanContext {}
unsafe impl Sync for DirectVulkanContext {}

impl DirectVulkanContext {
    pub fn new() -> Result<Self, String> {
        let ctx = unsafe { dv_context_create() };
        if ctx.is_null() {
            return Err(format!(
                "Vulkan compute context unavailable: {}",
                last_error()
            ));
        }
        Ok(Self { ctx })
    }

    /// Dispatch one raw-word → packed-BGRA pass between two AHBs.
    ///
    /// Both buffers stay GPU-owned for the whole call. Imports are cached by
    /// the C++ layer, so a steady-state frame only rewrites descriptors when
    /// the camera slot or output slot pointer changes.
    pub fn run_ahb(
        &self,
        input: *mut c_void,
        input_len: usize,
        output: *mut c_void,
        output_len: usize,
        width: u32,
        height: u32,
        mode: u32,
    ) -> Result<(), String> {
        if input.is_null() || output.is_null() {
            return Err("null AHardwareBuffer in direct Vulkan run".to_string());
        }
        let rc = unsafe {
            dv_run_ahb(
                self.ctx, input, input_len, output, output_len, width, height, mode,
            )
        };
        if rc == 0 {
            Ok(())
        } else {
            Err(last_error())
        }
    }
}

impl Drop for DirectVulkanContext {
    fn drop(&mut self) {
        if !self.ctx.is_null() {
            unsafe { dv_context_destroy(self.ctx) };
            self.ctx = ptr::null_mut();
        }
    }
}
