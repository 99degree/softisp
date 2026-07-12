//! Gralloc interop interface for Android camera buffers.
//!
//! Provides:
//! - `native_handle_t` FFI (raw buffer handle from camera framework)
//! - `GrallocInterop` trait — map gralloc handles → CameraBuffer
//! - NDK AHardwareBuffer path (`AHardwareBuffer_fromHardwareBuffer`)
//! - Raw fd path (extract fd → mmap, no NDK needed)

use std::os::raw::c_int;

use cam_hal::buffer::CameraBuffer;

use crate::adapter::ahardware_buffer::{AHardwareBuffer, AHardwareBuffer_Desc};

// ── native_handle_t FFI ────────────────────────────────────────────────────
//
// Matches cutils/native_handle.h exactly.

#[repr(C)]
pub struct native_handle_t {
    pub version: c_int,
    pub numFds: c_int,
    pub numInts: c_int,
    pub data: [c_int; 0], // flexible array: numFds + numInts ints
}

/// Android `buffer_handle_t` = `const native_handle_t*`.
pub type BufferHandleT = *const native_handle_t;

// ── AHardwareBuffer_fromHardwareBuffer (NDK API 26+) ─────────────────────

extern "C" {
    /// Convert a `BufferHandleT` (native_handle) to an `AHardwareBuffer*`.
    /// Returns NULL on failure.
    /// Defined in <android/hardware_buffer.h>, libnativewindow.so.
    pub fn AHardwareBuffer_fromHardwareBuffer(handle: BufferHandleT) -> *mut AHardwareBuffer;

    /// Allocate a new AHardwareBuffer with the given description.
    /// Returns 0 on success, or a negative error code.
    pub fn AHardwareBuffer_allocate(
        desc: *const AHardwareBuffer_Desc,
        out_buffer: *mut *mut AHardwareBuffer,
    ) -> c_int;
}

// ── GrallocInterop trait ────────────────────────────────────────────────────

/// Strategy for importing/allocating gralloc buffers.
///
/// Two implementations exist:
/// - `AHardwareBufferInterop` — uses NDK `AHardwareBuffer_fromHardwareBuffer`
///   (API 26+, requires libnativewindow.so at runtime)
/// - `RawFdInterop` — directly `mmap`s the fd from `native_handle_t`
///   (works on any Android version, no NDK dependency)
pub trait GrallocInterop: Send + Sync {
    /// Import a framework-provided buffer handle and wrap it as a CameraBuffer.
    ///
    /// # Safety
    /// `handle` must be a valid `BufferHandleT` from a `camera3_stream_buffer_t`.
    unsafe fn import(
        &self,
        handle: BufferHandleT,
        format: i32,
        width: u32,
        height: u32,
        frame_number: u64,
    ) -> Result<Box<dyn CameraBuffer>, String>;

    /// Allocate a new gralloc buffer (for internal processing).
    fn allocate(&self, desc: &AHardwareBuffer_Desc) -> Result<Box<dyn CameraBuffer>, String>;

    /// Human-readable name.
    fn name(&self) -> &'static str;
}

// ── AHardwareBufferInterop ──────────────────────────────────────────────────

/// Uses NDK `AHardwareBuffer` API to import/allocate gralloc buffers.
#[derive(Debug)]
pub struct AHardwareBufferInterop;

impl GrallocInterop for AHardwareBufferInterop {
    unsafe fn import(
        &self,
        handle: BufferHandleT,
        format: i32,
        width: u32,
        height: u32,
        frame_number: u64,
    ) -> Result<Box<dyn CameraBuffer>, String> {
        let ahb = AHardwareBuffer_fromHardwareBuffer(handle);
        if ahb.is_null() {
            return Err("AHardwareBuffer_fromHardwareBuffer returned NULL".to_string());
        }
        Ok(Box::new(crate::adapter::AHardwareBufferBacked::new(
            ahb,
            width,
            height,
            format,
            frame_number,
        )))
    }

    fn allocate(&self, desc: &AHardwareBuffer_Desc) -> Result<Box<dyn CameraBuffer>, String> {
        let mut ahb: *mut AHardwareBuffer = std::ptr::null_mut();
        let ret = unsafe { AHardwareBuffer_allocate(desc, &mut ahb) };
        if ret != 0 || ahb.is_null() {
            return Err(format!("AHardwareBuffer_allocate failed: {}", ret));
        }
        unsafe {
            Ok(Box::new(crate::adapter::AHardwareBufferBacked::new(
                ahb,
                desc.width,
                desc.height,
                desc.format as i32,
                0,
            )))
        }
    }

    fn name(&self) -> &'static str {
        "ahardwarebuffer"
    }
}

// ── RawFdInterop ────────────────────────────────────────────────────────────

/// Directly `mmap`s the dma-buf fd from `native_handle_t.data[0]`.
/// No NDK dependency — works on any Android version.
#[derive(Debug)]
pub struct RawFdInterop;

impl GrallocInterop for RawFdInterop {
    unsafe fn import(
        &self,
        handle: BufferHandleT,
        format: i32,
        width: u32,
        height: u32,
        frame_number: u64,
    ) -> Result<Box<dyn CameraBuffer>, String> {
        if handle.is_null() {
            return Err("Null BufferHandleT".to_string());
        }
        let nh = &*handle;

        // Extract first valid fd
        let fd = (0..nh.numFds)
            .map(|i| nh.data[i as usize])
            .find(|&fd| fd != -1)
            .ok_or("No valid fd in native_handle")?;

        let buf = crate::adapter::MmapFdCameraBuffer::new(fd, format, width, height, frame_number)?;
        Ok(Box::new(buf))
    }

    fn allocate(&self, desc: &AHardwareBuffer_Desc) -> Result<Box<dyn CameraBuffer>, String> {
        // Raw fd path can't allocate gralloc buffers directly.
        // Fall back to heap for internal allocations.
        let size = (desc.width as usize) * (desc.height as usize) * 4; // assume RGBA
        Ok(Box::new(cam_hal::buffer::GenericCameraBuffer::new(
            desc.width,
            desc.height,
            desc.stride,
            desc.format as i32,
            Box::new(cam_hal::buffer::HeapBuffer::new(size)),
        )))
    }

    fn name(&self) -> &'static str {
        "rawfd"
    }
}
