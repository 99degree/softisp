//! Buffer management for zero-copy operations (Linux + Android)
use crate::{Buffer, IpaError, IpaResult};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};

/// DMA buffer wrapper — works on Linux (V4L2 dmabuf) and Android (SharedFD).
///
/// Maps the fd via mmap on creation so `as_mut_ptr()` returns a valid host
/// pointer for `ProcessParams::input_buffer` / `output_buffer` zero-copy.
pub struct DmaBufBuffer {
    fd: OwnedFd,
    size: usize,
    mmap_ptr: *mut u8,
    mmap_len: usize,
}

// Safety: The mmap pointer is fixed after creation and never changes.
// All reads/writes go through the same pointer, so it is safe to share
// across threads as long as the caller synchronizes access to the buffer
// contents (which the ISP pipeline does via single-frame ownership).
unsafe impl Send for DmaBufBuffer {}
unsafe impl Sync for DmaBufBuffer {}

impl DmaBufBuffer {
    /// Create from raw fd (takes ownership) and mmap the whole buffer.
    pub fn new(fd: RawFd, size: usize) -> IpaResult<Self> {
        if fd < 0 {
            return Err(IpaError::Buffer("Invalid fd".into()));
        }
        // SAFETY: We own this fd.
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };

        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd.as_raw_fd(),
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err(IpaError::Buffer(format!(
                "mmap failed for fd {}",
                fd.as_raw_fd()
            )));
        }

        Ok(Self {
            fd,
            size,
            mmap_ptr: ptr as *mut u8,
            mmap_len: size,
        })
    }

    /// Create from an existing mmap'd pointer (caller retains ownership of fd).
    pub fn from_mmap(fd: RawFd, ptr: *mut u8, len: usize) -> Self {
        // SAFETY: We take ownership of the fd.
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };
        Self {
            fd,
            size: len,
            mmap_ptr: ptr,
            mmap_len: len,
        }
    }

    /// Get the raw dmabuf fd.
    pub fn fd(&self) -> RawFd {
        self.fd.as_raw_fd()
    }
}

impl Buffer for DmaBufBuffer {
    fn as_ptr(&self) -> *const u8 {
        self.mmap_ptr as *const u8
    }

    fn as_mut_ptr(&mut self) -> *mut u8 {
        self.mmap_ptr
    }

    fn len(&self) -> usize {
        self.size
    }

    fn fd(&self) -> Option<RawFd> {
        Some(self.fd.as_raw_fd())
    }
}

impl Drop for DmaBufBuffer {
    fn drop(&mut self) {
        if !self.mmap_ptr.is_null() {
            unsafe {
                libc::munmap(self.mmap_ptr as *mut libc::c_void, self.mmap_len);
            }
        }
    }
}

/// AHardwareBuffer wrapper for Android (Vulkan zero-copy via input_ahb/output_ahb).
#[cfg(target_os = "android")]
pub struct AhwBuffer {
    ahb: *mut std::ffi::c_void,
    size: usize,
    format: u32,
    width: u32,
    height: u32,
    strides: [u32; 4],
}

#[cfg(target_os = "android")]
unsafe impl Send for AhwBuffer {}
#[cfg(target_os = "android")]
unsafe impl Sync for AhwBuffer {}

#[cfg(target_os = "android")]
impl AhwBuffer {
    pub fn new(
        ahb: *mut std::ffi::c_void,
        width: u32,
        height: u32,
        format: u32,
    ) -> IpaResult<Self> {
        if ahb.is_null() {
            return Err(IpaError::Buffer("Null AHardwareBuffer".into()));
        }
        use ndk_sys::*;
        let ahb_ptr = ahb as *const _;
        let mut desc: AHardwareBuffer_Desc = unsafe { std::mem::zeroed() };
        unsafe { AHardwareBuffer_describe(ahb_ptr, &mut desc) };

        Ok(Self {
            ahb,
            size: (desc.width as usize) * (desc.height as usize) * 4, // estimate RGBA
            format: desc.format,
            width: desc.width,
            height: desc.height,
            strides: [desc.stride; 4],
        })
    }
}

#[cfg(target_os = "android")]
impl Buffer for AhwBuffer {
    fn as_ptr(&self) -> *const u8 {
        std::ptr::null() // AHB accessed via Vulkan import, not CPU mapping
    }

    fn as_mut_ptr(&mut self) -> *mut u8 {
        std::ptr::null_mut()
    }

    fn len(&self) -> usize {
        self.size
    }

    fn fd(&self) -> Option<RawFd> {
        None // AHB doesn't have a simple fd
    }

    fn is_ahb(&self) -> bool {
        true
    }

    fn ahb_ptr(&self) -> Option<*mut std::ffi::c_void> {
        Some(self.ahb)
    }
}

/// Create a zero-copy dmabuf using memfd (Linux) for testing without a real camera.
///
/// On Linux, `memfd_create` + `mmap` gives a shareable fd that can be passed
/// between processes and mmap'd for zero-copy ISP processing.
#[cfg(not(target_os = "android"))]
pub fn create_memfd_dmabuf(size: usize) -> IpaResult<(RawFd, DmaBufBuffer)> {
    use std::os::fd::AsRawFd;
    let fd = unsafe { libc::memfd_create(b"softisp_ipa\0".as_ptr(), 0) };
    if fd < 0 {
        return Err(IpaError::Buffer(format!(
            "memfd_create failed: {}",
            std::io::Error::last_os_error()
        )));
    }

    // Size the file
    if unsafe { libc::ftruncate(fd, size as i64) } != 0 {
        let _ = unsafe { libc::close(fd) };
        return Err(IpaError::Buffer(format!(
            "ftruncate failed: {}",
            std::io::Error::last_os_error()
        )));
    }

    let buf = DmaBufBuffer::new(fd, size)?;
    let raw_fd = buf.fd();
    Ok((raw_fd, buf))
}

/// In-memory buffer for testing (no zero-copy, host memory only).
pub struct MemBuffer {
    pub data: Vec<u8>,
}

impl MemBuffer {
    pub fn new(size: usize) -> Self {
        Self {
            data: vec![0u8; size],
        }
    }

    pub fn from_vec(data: Vec<u8>) -> Self {
        Self { data }
    }
}

impl Buffer for MemBuffer {
    fn as_ptr(&self) -> *const u8 {
        self.data.as_ptr()
    }

    fn as_mut_ptr(&mut self) -> *mut u8 {
        self.data.as_mut_ptr()
    }

    fn len(&self) -> usize {
        self.data.len()
    }

    fn fd(&self) -> Option<RawFd> {
        None
    }
}

/// Buffer manager for allocating and tracking buffers.
pub struct BufferManager {
    buffers: Vec<std::sync::Arc<dyn Buffer>>,
}

impl BufferManager {
    pub fn new() -> Self {
        Self {
            buffers: Vec::new(),
        }
    }

    pub fn allocate_dmabuf(
        &mut self,
        fd: RawFd,
        size: usize,
    ) -> IpaResult<std::sync::Arc<dyn Buffer>> {
        let buf = DmaBufBuffer::new(fd, size)?;
        let arc = std::sync::Arc::new(buf);
        self.buffers.push(arc.clone());
        Ok(arc)
    }

    #[cfg(target_os = "android")]
    pub fn allocate_ahb(
        &mut self,
        ahb: *mut std::ffi::c_void,
        width: u32,
        height: u32,
        format: u32,
    ) -> IpaResult<std::sync::Arc<dyn Buffer>> {
        let buf = AhwBuffer::new(ahb, width, height, format)?;
        let arc = std::sync::Arc::new(buf);
        self.buffers.push(arc.clone());
        Ok(arc)
    }

    pub fn allocate_mem(&mut self, size: usize) -> std::sync::Arc<dyn Buffer> {
        let buf = MemBuffer::new(size);
        let arc = std::sync::Arc::new(buf);
        self.buffers.push(arc.clone());
        arc
    }

    pub fn clear(&mut self) {
        self.buffers.clear();
    }
}

impl Default for BufferManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(not(target_os = "android"))]
    #[test]
    fn test_memfd_dmabuf_zero_copy() {
        let size = 4096;
        let (_fd, buf) = create_memfd_dmabuf(size).expect("memfd dmabuf");
        assert_eq!(buf.len(), size);
        assert!(!buf.as_mut_ptr().is_null(), "mmap pointer must be valid");

        // Write and read back through the mmap'd pointer
        unsafe {
            std::ptr::write_bytes(buf.as_mut_ptr(), 0xAA, size);
            assert_eq!(buf.as_ptr().read(), 0xAA);
        }
    }

    #[test]
    fn test_mem_buffer() {
        let mut buf = MemBuffer::new(100);
        assert_eq!(buf.len(), 100);
        assert!(!buf.as_mut_ptr().is_null());
        buf.data[0] = 42;
        assert_eq!(unsafe { buf.as_ptr().read() }, 42);
    }
}
