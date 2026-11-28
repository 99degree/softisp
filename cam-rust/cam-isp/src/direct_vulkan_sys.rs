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
    fn dv_run_ahb_image(
        ctx: *mut c_void,
        input_ahb: *mut c_void,
        input_len: usize,
        output_ahb: *mut c_void,
        width: u32,
        height: u32,
        mode: u32,
    ) -> c_int;
    fn dv_probe_image_ahb(
        ctx: *mut c_void,
        ahb: *mut c_void,
        width: u32,
        height: u32,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn dv_probe_raw16_image_ahb(
        ctx: *mut c_void,
        ahb: *mut c_void,
        width: u32,
        height: u32,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn dv_probe_external_memory_extensions(
        ctx: *mut c_void,
        ahb: *mut c_void,
        out: *mut c_char,
        out_len: usize,
    ) -> c_int;
    fn dv_probe_dmabuf_ahb_global(ahb: *mut c_void, out: *mut c_char, out_len: usize) -> c_int;
    fn dv_probe_opaque_fd_ahb_global(ahb: *mut c_void, out: *mut c_char, out_len: usize) -> c_int;
    fn dv_supports_dma_buf_external_memory(ctx: *mut c_void) -> c_int;
    fn dv_run_ahb_fd_image(
        ctx: *mut c_void,
        input_ahb: *mut c_void,
        input_len: usize,
        output_ahb: *mut c_void,
        width: u32,
        height: u32,
        mode: u32,
    ) -> c_int;
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

fn probe_ahb_fd(
    probe: unsafe extern "C" fn(*mut c_void, *mut c_char, usize) -> c_int,
    ahb: *mut c_void,
) -> (bool, String) {
    let mut buf = [0 as c_char; 1024];
    let rc = unsafe { probe(ahb, buf.as_mut_ptr(), buf.len()) };
    let text = unsafe { CStr::from_ptr(buf.as_ptr()) }
        .to_string_lossy()
        .into_owned();
    (rc == 0, text)
}

/// Probe AHB → `VkBuffer` import through `VK_EXT_external_memory_dma_buf`.
///
/// This works without `VK_ANDROID_external_memory_android_hardware_buffer`, so
/// it can be used with ICDs such as Mesa Turnip that expose dma_buf import but
/// not AHardwareBuffer import.
pub fn probe_dmabuf_ahb(ahb: *mut c_void) -> (bool, String) {
    probe_ahb_fd(dv_probe_dmabuf_ahb_global, ahb)
}

/// Probe AHB fd import as a Vulkan opaque fd.
///
/// The camera AHB fd is a dma_buf fd, so vendor drivers that expose only
/// `VK_KHR_external_memory_fd` may reject it. This probe decides whether the
/// built-in Adreno HAL can consume camera buffers without Turnip.
pub fn probe_opaque_fd_ahb(ahb: *mut c_void) -> (bool, String) {
    probe_ahb_fd(dv_probe_opaque_fd_ahb_global, ahb)
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

    /// Dispatch one RAW16 word pass into an AHB-backed storage `VkImage`.
    ///
    /// No CPU mapping is opened on the output AHB, so the same handle can be
    /// handed to SurfaceFlinger immediately after the fence wait.
    pub fn run_ahb_image(
        &self,
        input: *mut c_void,
        input_len: usize,
        output: *mut c_void,
        width: u32,
        height: u32,
        mode: u32,
    ) -> Result<(), String> {
        if input.is_null() || output.is_null() {
            return Err("null AHardwareBuffer in direct Vulkan image run".to_string());
        }
        let rc =
            unsafe { dv_run_ahb_image(self.ctx, input, input_len, output, width, height, mode) };
        if rc == 0 {
            Ok(())
        } else {
            Err(last_error())
        }
    }

    /// One-shot probe: can this AHB be imported as a storage `VkImage`?
    pub fn probe_image_ahb(&self, ahb: *mut c_void, width: u32, height: u32) -> (bool, String) {
        let mut buf = [0 as c_char; 512];
        let rc = unsafe {
            dv_probe_image_ahb(self.ctx, ahb, width, height, buf.as_mut_ptr(), buf.len())
        };
        let text = unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        (rc == 0, text)
    }

    /// Which external-memory fd extensions the device exposes.
    pub fn probe_external_memory_extensions(&self, ahb: *mut c_void) -> String {
        let mut buf = [0 as c_char; 1024];
        let rc = unsafe {
            dv_probe_external_memory_extensions(self.ctx, ahb, buf.as_mut_ptr(), buf.len())
        };
        if rc != 0 {
            return "probe failed".to_string();
        }
        unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned()
    }

    /// Probe explicit 16-bit Vulkan formats against a RAW16 AHardwareBuffer.
    ///
    /// The AHB property query reports `VK_FORMAT_UNDEFINED` on this device, but
    /// some drivers still accept an explicit `R16_UINT`/`R16_UNORM` import.
    pub fn probe_raw16_image_ahb(
        &self,
        ahb: *mut c_void,
        width: u32,
        height: u32,
    ) -> (bool, String) {
        let mut buf = [0 as c_char; 1024];
        let rc = unsafe {
            dv_probe_raw16_image_ahb(self.ctx, ahb, width, height, buf.as_mut_ptr(), buf.len())
        };
        let text = unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        (rc == 0, text)
    }

    /// Whether this Vulkan device can import camera AHB dma_buf fds directly.
    pub fn supports_dma_buf(&self) -> bool {
        unsafe { dv_supports_dma_buf_external_memory(self.ctx) != 0 }
    }

    /// Dispatch a strided RAW16 dma_buf fd into an AHB-backed storage `VkImage`.
    ///
    /// The input AHB stays opaque to the CPU: its native fd is imported as
    /// `VkBuffer` device memory and the shader follows the plane's row stride.
    pub fn run_ahb_fd_image(
        &self,
        input: *mut c_void,
        input_len: usize,
        output: *mut c_void,
        width: u32,
        height: u32,
        mode: u32,
    ) -> Result<(), String> {
        if input.is_null() || output.is_null() {
            return Err("null AHardwareBuffer in direct Vulkan fd run".to_string());
        }
        let rc =
            unsafe { dv_run_ahb_fd_image(self.ctx, input, input_len, output, width, height, mode) };
        if rc == 0 {
            Ok(())
        } else {
            Err(last_error())
        }
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
