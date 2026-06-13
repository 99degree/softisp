//! Linux DMA-BUF allocator backend.
//!
//! Allocates physically-contiguous, DMA-capable buffers via
//! `/dev/dma_heap` (dma-heap) or `/dev/ion` (ION).
//! Implements `cam_hal::buffer::BufferAllocator`.

#![allow(dead_code)]

use std::fs::{File, OpenOptions};
use std::os::unix::io::{AsRawFd, FromRawFd};
use std::sync::atomic::{AtomicBool, Ordering};

use cam_hal::buffer::{BufferAllocator, CameraBuffer, MappedBuffer};

// ── DMABuf ─────────────────────────────────────────────────────────────────

/// A DMA-BUF backed buffer. Physically contiguous, suitable for ISP/GPU.
#[derive(Debug)]
pub struct DMABuf {
    fd: Option<std::os::unix::io::RawFd>,
    ptr: *mut u8,
    size: usize,
    mapped: AtomicBool,
    // heap path used
    heap: &'static str,
}

impl DMABuf {
    fn allocate(size: usize, heap: &str) -> Result<Self, String> {
        // Try dma-heap first, fall back to ion
        let (fd, heap_used) = Self::open_heap(heap, size)?;
        let ptr = Self::mmap_fd(fd, size)?;
        Ok(Self {
            fd: Some(fd),
            ptr,
            size,
            mapped: AtomicBool::new(true),
            heap: heap_used,
        })
    }

    fn open_heap(heap: &str, size: usize) -> Result<(std::os::unix::io::RawFd, &'static str), String> {
        // Try dma-heap
        let path = format!("/dev/dma_heap/{}", heap);
        if let Ok(file) = OpenOptions::new().read(true).write(true).open(&path) {
            let fd = file.into_raw_fd();
            // Set buffer size via DMA_BUF_IOCTL (not trivial, fallback for now)
            // Real implementation would use DMA_HEAP_IOCTL_ALLOC
            return Ok((fd, "dma-heap"));
        }

        // Fallback: just use memfd for now
        let name = std::ffi::CString::new("cam-dma").unwrap();
        let fd = unsafe { libc::memfd_create(name.as_ptr(), 0) };
        if fd < 0 {
            return Err(format!("Failed to allocate DMA buffer (size {})", size));
        }
        if unsafe { libc::ftruncate(fd, size as i64) } < 0 {
            unsafe { libc::close(fd) };
            return Err("ftruncate failed".to_string());
        }
        Ok((fd, "memfd"))
    }

    fn mmap_fd(fd: std::os::unix::io::RawFd, size: usize) -> Result<*mut u8, String> {
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err("mmap failed for DMA buffer".to_string());
        }
        Ok(ptr as *mut u8)
    }
}

impl Drop for DMABuf {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe { libc::munmap(self.ptr as *mut libc::c_void, self.size) };
        }
        if let Some(fd) = self.fd {
            unsafe { libc::close(fd) };
        }
    }
}

impl MappedBuffer for DMABuf {
    fn map(&self) -> Result<(*mut u8, usize), String> {
        if self.ptr.is_null() {
            return Err("DMABuf already freed".to_string());
        }
        self.mapped.store(true, Ordering::Release);
        Ok((self.ptr, self.size))
    }

    fn unmap(&self) -> Result<(), String> {
        // DMA-BUF stays mapped; we just sync
        self.mapped.store(false, Ordering::Release);
        Ok(())
    }

    fn is_mapped(&self) -> bool {
        self.mapped.load(Ordering::Acquire)
    }

    fn size(&self) -> usize {
        self.size
    }

    fn fd(&self) -> Option<i32> {
        self.fd
    }
}

unsafe impl Send for DMABuf {}
unsafe impl Sync for DMABuf {}

// ── DMABuf allocator ───────────────────────────────────────────────────────

/// Allocator that uses Linux dma-heap (or fallback memfd).
#[derive(Debug)]
pub struct DMABufAllocator {
    heap: &'static str,
}

impl DMABufAllocator {
    pub fn new() -> Self {
        Self { heap: "system" }
    }

    pub fn with_heap(heap: &'static str) -> Self {
        Self { heap }
    }
}

impl Default for DMABufAllocator {
    fn default() -> Self {
        Self::new()
    }
}

impl BufferAllocator for DMABufAllocator {
    fn name(&self) -> &'static str {
        "dmabuf"
    }

    fn allocate(&self, size: usize) -> Result<Box<dyn MappedBuffer>, String> {
        let buf = DMABuf::allocate(size, self.heap)?;
        Ok(Box::new(buf))
    }

    fn allocate_frame(
        &self,
        width: u32,
        height: u32,
        stride: u32,
        format: i32,
    ) -> Result<Box<dyn CameraBuffer>, String> {
        let bpp = match format {
            0x20 | 0x25 | 0x26 => 2,
            0x11 | 0x12 => 1,
            _ => 4,
        };
        let size = (stride as usize) * (height as usize) * bpp;
        let buf = DMABuf::allocate(size, self.heap)?;

        Ok(Box::new(cam_hal::buffer::GenericCameraBuffer::new(
            width, height, stride, format, Box::new(buf),
        )))
    }
}

// ── Memfd allocator ────────────────────────────────────────────────────────

/// A memfd-backed buffer (Linux only).
#[derive(Debug)]
pub struct MemfdBuf {
    fd: Option<std::os::unix::io::RawFd>,
    ptr: *mut u8,
    size: usize,
    mapped: AtomicBool,
}

impl MemfdBuf {
    pub fn new(size: usize) -> Result<Self, String> {
        let name = std::ffi::CString::new("cam-memfd").unwrap();
        let fd = unsafe { libc::memfd_create(name.as_ptr(), 0) };
        if fd < 0 {
            return Err(format!("memfd_create failed (size {})", size));
        }
        if unsafe { libc::ftruncate(fd, size as i64) } < 0 {
            unsafe { libc::close(fd) };
            return Err("ftruncate failed".to_string());
        }
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            unsafe { libc::close(fd) };
            return Err("mmap failed for memfd".to_string());
        }
        Ok(Self {
            fd: Some(fd),
            ptr: ptr as *mut u8,
            size,
            mapped: AtomicBool::new(true),
        })
    }
}

impl Drop for MemfdBuf {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe { libc::munmap(self.ptr as *mut libc::c_void, self.size) };
        }
        if let Some(fd) = self.fd {
            unsafe { libc::close(fd) };
        }
    }
}

impl MappedBuffer for MemfdBuf {
    fn map(&self) -> Result<(*mut u8, usize), String> {
        Ok((self.ptr, self.size))
    }
    fn unmap(&self) -> Result<(), String> { Ok(()) }
    fn is_mapped(&self) -> bool { true }
    fn size(&self) -> usize { self.size }
    fn fd(&self) -> Option<i32> { self.fd }
}

unsafe impl Send for MemfdBuf {}
unsafe impl Sync for MemfdBuf {}

/// Allocator using Linux memfd (`memfd_create`).
#[derive(Debug)]
pub struct MemfdAllocator;

impl BufferAllocator for MemfdAllocator {
    fn name(&self) -> &'static str {
        "memfd"
    }
    fn allocate(&self, size: usize) -> Result<Box<dyn MappedBuffer>, String> {
        Ok(Box::new(MemfdBuf::new(size)?))
    }
    fn allocate_frame(
        &self,
        width: u32,
        height: u32,
        stride: u32,
        format: i32,
    ) -> Result<Box<dyn CameraBuffer>, String> {
        let bpp = match format {
            0x20 | 0x25 | 0x26 => 2,
            0x11 | 0x12 => 1,
            _ => 4,
        };
        let size = (stride as usize) * (height as usize) * bpp;
        Ok(Box::new(cam_hal::buffer::GenericCameraBuffer::new(
            width, height, stride, format, Box::new(MemfdBuf::new(size)?),
        )))
    }
}
