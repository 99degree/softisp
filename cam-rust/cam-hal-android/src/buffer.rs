//! Zero-copy HardwareBuffer operations.
//! Ported from com.camhal.HardwareBufferOps and com.camhal.buffer.*

use std::ptr;
use std::slice;
// unused
use ndk_sys::*;
// unused
use crate::error::{Error, Result};

// Define the constant if not available from ndk-sys
pub const AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN: u64 = 0x3;

/// Lock an AHardwareBuffer for CPU read access and return a pointer to its data.
///
/// # Safety
/// The caller must ensure the buffer is valid and not already locked.
pub unsafe fn lock_hardware_buffer(buffer: *mut AHardwareBuffer, size: usize) -> Result<*mut u8> {
    let mut data: *mut u8 = ptr::null_mut();
    let ret = AHardwareBuffer_lock(
        buffer,
        AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN as u64,
        -1,
        ptr::null(),
        &mut data as *mut *mut u8 as *mut *mut std::ffi::c_void,
    );
    if ret != 0 || data.is_null() {
        return Err(Error(format!("Failed to lock HardwareBuffer: error={}", ret)));
    }
    Ok(data)
}

/// Unlock a previously locked AHardwareBuffer.
///
/// # Safety
/// The caller must ensure the buffer was previously locked and is no longer in use.
pub unsafe fn unlock_hardware_buffer(buffer: *mut AHardwareBuffer) {
    let fence: *mut i32 = ptr::null_mut();
    AHardwareBuffer_unlock(buffer, fence);
}

/// Lock a buffer and execute a closure with the data, then unlock.
///
/// # Safety
/// The closure must not leak the data pointer outside its scope.
pub unsafe fn with_locked_buffer<F, T>(buffer: *mut AHardwareBuffer, size: usize, f: F) -> Result<T>
where
    F: FnOnce(&mut [u8]) -> T,
{
    let data = lock_hardware_buffer(buffer, size)?;
    let slice = slice::from_raw_parts_mut(data, size);
    let result = f(slice);
    unlock_hardware_buffer(buffer);
    Ok(result)
}

/// Wrap a native pointer into a ByteBuffer-like view.
pub fn wrap_pointer_to_buffer(ptr: u64, size: i32) -> Result<Vec<u8>> {
    if ptr == 0 || size <= 0 {
        return Err(Error("Invalid pointer or size".to_string()));
    }
    // Safety: we assume the caller guarantees the memory is valid for `size` bytes.
    let slice = unsafe { slice::from_raw_parts(ptr as *const u8, size as usize) };
    Ok(slice.to_vec())
}

/// GpuBuffer for handling hardware-accelerated buffers.
pub struct GpuBuffer {
    native_buffer: *mut AHardwareBuffer,
    width: u32,
    height: u32,
    format: i32,
}

impl GpuBuffer {
    pub fn new(width: u32, height: u32, format: i32) -> Result<Self> {
        // TODO: Use AHardwareBuffer_allocate to create a new buffer.
        Ok(Self {
            native_buffer: ptr::null_mut(),
            width,
            height,
            format,
        })
    }

    pub fn native_buffer(&self) -> *mut AHardwareBuffer {
        self.native_buffer
    }
}

impl Drop for GpuBuffer {
    fn drop(&mut self) {
        if !self.native_buffer.is_null() {
            unsafe {
                // AHardwareBuffer_release(self.native_buffer);
            }
        }
    }
}

/// Native buffer operations for importing/exporting gralloc buffers.
pub struct NativeBufferOps;

impl NativeBufferOps {
    /// Import a buffer from a file descriptor (dmabuf).
    pub fn import_from_fd(_fd: i32, _size: i32) -> Result<GpuBuffer> {
        // TODO: Implement dmabuf import.
        Err(Error("Not implemented".to_string()))
    }
}

// Redefine AHardwareBuffer_lock/unlock if ndk-sys doesn't expose them fully.
// We use raw libc::dlopen/dlsym approach or rely on ndk-sys bindings.

// Since ndk-sys might not have the latest AHardwareBuffer_lock functions,
// we define the FFI ourselves.
extern "C" {
    #[link_name = "AHardwareBuffer_lock"]
    fn AHardwareBuffer_lock(
        buffer: *mut AHardwareBuffer,
        usage: u64,
        fence: i32,
        rect: *const std::ffi::c_void,
        out_virtual_address: *mut *mut std::ffi::c_void,
    ) -> i32;

    #[link_name = "AHardwareBuffer_unlock"]
    fn AHardwareBuffer_unlock(
        buffer: *mut AHardwareBuffer,
        fence: *mut i32,
    ) -> i32;
}
