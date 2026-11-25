//! Buffer management for zero-copy operations

use std::os::fd::{RawFd, AsRawFd, OwnedFd, FromRawFd};
use crate::{Buffer, IpaResult, IpaError};

/// DMA buffer wrapper for libcamera SharedFD
pub struct DmaBufBuffer {
    fd: OwnedFd,
    size: usize,
}

impl DmaBufBuffer {
    /// Create from raw fd (takes ownership)
    pub fn new(fd: RawFd, size: usize) -> IpaResult<Self> {
        if fd < 0 {
            return Err(IpaError::Buffer("Invalid fd".into()));
        }
        // SAFETY: We own this fd
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };
        Ok(Self { fd, size })
    }
    
    /// Get mmap pointer (maps on each access - stateless for Sync safety)
    pub fn mmap_mut(&self) -> IpaResult<*mut u8> {
        use std::os::unix::io::AsRawFd;
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                self.size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED | libc::MAP_PRIVATE,
                self.fd.as_raw_fd(),
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err(IpaError::Buffer("mmap failed".into()));
        }
        Ok(ptr as *mut u8)
    }
}

impl Buffer for DmaBufBuffer {
    fn as_ptr(&self) -> *const u8 {
        // Stateless: no cached mmap pointer
        std::ptr::null()
    }
    
    fn as_mut_ptr(&mut self) -> *mut u8 {
        // Stateless: caller must call mmap_mut() explicitly
        std::ptr::null_mut()
    }
    
    fn len(&self) -> usize {
        self.size
    }
    
    fn fd(&self) -> Option<RawFd> {
        Some(self.fd.as_raw_fd())
    }
}

/// AHardwareBuffer wrapper for Android
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
    pub fn new(ahb: *mut std::ffi::c_void, width: u32, height: u32, format: u32) -> IpaResult<Self> {
        if ahb.is_null() {
            return Err(IpaError::Buffer("Null AHardwareBuffer".into()));
        }
        // Query buffer info
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

/// In-memory buffer for testing
pub struct MemBuffer {
    data: Vec<u8>,
}

impl MemBuffer {
    pub fn new(size: usize) -> Self {
        Self { data: vec![0u8; size] }
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

/// Buffer manager for allocating and tracking buffers
pub struct BufferManager {
    buffers: Vec<std::sync::Arc<dyn Buffer>>,
}

impl BufferManager {
    pub fn new() -> Self {
        Self { buffers: Vec::new() }
    }
    
    pub fn allocate_dmabuf(&mut self, fd: RawFd, size: usize) -> IpaResult<std::sync::Arc<dyn Buffer>> {
        let buf = DmaBufBuffer::new(fd, size)?;
        let arc = std::sync::Arc::new(buf);
        self.buffers.push(arc.clone());
        Ok(arc)
    }
    
    #[cfg(target_os = "android")]
    pub fn allocate_ahb(&mut self, ahb: *mut std::ffi::c_void, width: u32, height: u32, format: u32) -> IpaResult<std::sync::Arc<dyn Buffer>> {
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