//! Unified buffer allocation API.
//!
//! Provides a single set of traits that all allocation backends
//! (heap, AHardwareBuffer, V4L2, CMA, memfd) implement so the
//! pipeline and HAL adapters can allocate/manage buffers without
//! knowing which backend is in use.

use std::fmt::Debug;

// ── MappedBuffer ───────────────────────────────────────────────────────────

/// A buffer whose memory can be mapped for CPU access.
///
/// Implementations wrap platform-specific allocations (heap Vec,
/// AHardwareBuffer, dma-buf, CMA, memfd, etc.).
pub trait MappedBuffer: Send + Sync + Debug {
    /// Map the buffer for CPU read/write access.
    /// Returns (pointer, size_in_bytes).
    fn map(&self) -> Result<(*mut u8, usize), String>;

    /// Unmap after CPU access is done.
    fn unmap(&self) -> Result<(), String>;

    /// Is the buffer currently mapped?
    fn is_mapped(&self) -> bool;

    /// Buffer size in bytes (always valid, even when unmapped).
    fn size(&self) -> usize;

    /// Optional file descriptor (dma-buf, memfd, etc.).
    fn fd(&self) -> Option<i32> { None }
}

// ── CameraBuffer ───────────────────────────────────────────────────────────

/// A frame buffer from a camera source with format metadata.
pub trait CameraBuffer: MappedBuffer {
    fn width(&self) -> u32;
    fn height(&self) -> u32;
    fn stride(&self) -> u32;
    fn format(&self) -> i32; // Android HAL pixel format or custom
    fn timestamp(&self) -> u64;
    fn frame_number(&self) -> u64;
}

// ── BufferAllocator ────────────────────────────────────────────────────────

/// Allocator backend — creates buffers from a concrete strategy.
///
/// Each backend implements this trait. The pipeline selects one at init
/// time and uses it for all buffer allocations.
pub trait BufferAllocator: Send + Sync + Debug {
    /// Human-readable backend name.
    fn name(&self) -> &'static str;

    /// Allocate a plain buffer of `size` bytes.
    fn allocate(&self, size: usize) -> Result<Box<dyn MappedBuffer>, String>;

    /// Allocate a camera frame buffer with format metadata.
    fn allocate_frame(
        &self,
        width: u32,
        height: u32,
        stride: u32,
        format: i32,
    ) -> Result<Box<dyn CameraBuffer>, String>;
}

// ── HeapAllocator (default, always available) ──────────────────────────────

/// Allocator backed by plain `Vec<u8>`.
///
/// Works everywhere. No zero-copy with hardware, but reliable.
#[derive(Debug, Clone)]
pub struct HeapAllocator;

impl HeapAllocator {
    pub fn new() -> Self {
        Self
    }
}

impl BufferAllocator for HeapAllocator {
    fn name(&self) -> &'static str {
        "heap"
    }

    fn allocate(&self, size: usize) -> Result<Box<dyn MappedBuffer>, String> {
        Ok(Box::new(HeapBuffer::new(size)))
    }

    fn allocate_frame(
        &self,
        width: u32,
        height: u32,
        stride: u32,
        format: i32,
    ) -> Result<Box<dyn CameraBuffer>, String> {
        let bpp = match format {
            0x20 | 0x25 | 0x26 => 2,  // RAW16/10/12
            0x11 | 0x12 => 1,          // NV21/NV12
            0x01 | 0x22 => 4,          // RGBA / IMPLEMENTATION_DEFINED
            _ => 4,
        };
        let size = (stride as usize) * (height as usize) * bpp;
        Ok(Box::new(HeapCameraBuffer::new(
            width, height, stride, format,
            HeapBuffer::new(size),
        )))
    }
}

// ── HeapBuffer ─────────────────────────────────────────────────────────────

/// A `Vec<u8>`-backed mapped buffer.
#[derive(Debug)]
pub struct HeapBuffer {
    data: std::cell::UnsafeCell<Vec<u8>>,
    mapped: std::cell::Cell<bool>,
}

impl HeapBuffer {
    pub fn new(size: usize) -> Self {
        Self {
            data: std::cell::UnsafeCell::new(vec![0u8; size]),
            mapped: std::cell::Cell::new(false),
        }
    }
}

impl MappedBuffer for HeapBuffer {
    fn map(&self) -> Result<(*mut u8, usize), String> {
        let v = unsafe { &mut *self.data.get() };
        let ptr = v.as_mut_ptr();
        let len = v.len();
        self.mapped.set(true);
        Ok((ptr, len))
    }

    fn unmap(&self) -> Result<(), String> {
        self.mapped.set(false);
        Ok(())
    }

    fn is_mapped(&self) -> bool {
        self.mapped.get()
    }

    fn size(&self) -> usize {
        unsafe { (*self.data.get()).len() }
    }
}

unsafe impl Send for HeapBuffer {}
unsafe impl Sync for HeapBuffer {}

// ── HeapCameraBuffer ───────────────────────────────────────────────────────

/// A heap-backed camera frame buffer with metadata.
#[derive(Debug)]
pub struct HeapCameraBuffer {
    width: u32,
    height: u32,
    stride: u32,
    format: i32,
    inner: HeapBuffer,
}

impl HeapCameraBuffer {
    pub fn new(width: u32, height: u32, stride: u32, format: i32, inner: HeapBuffer) -> Self {
        Self { width, height, stride, format, inner }
    }
}

impl MappedBuffer for HeapCameraBuffer {
    fn map(&self) -> Result<(*mut u8, usize), String> {
        self.inner.map()
    }
    fn unmap(&self) -> Result<(), String> {
        self.inner.unmap()
    }
    fn is_mapped(&self) -> bool {
        self.inner.is_mapped()
    }
    fn size(&self) -> usize {
        self.inner.size()
    }
}

impl CameraBuffer for HeapCameraBuffer {
    fn width(&self) -> u32 { self.width }
    fn height(&self) -> u32 { self.height }
    fn stride(&self) -> u32 { self.stride }
    fn format(&self) -> i32 { self.format }
    fn timestamp(&self) -> u64 { 0 }
    fn frame_number(&self) -> u64 { 0 }
}

// ── GenericCameraBuffer ─────────────────────────────────────────────────────

/// Wraps any `MappedBuffer` as a `CameraBuffer` with explicit metadata.
/// Useful when the underlying buffer doesn't natively track frame metadata.
#[derive(Debug)]
pub struct GenericCameraBuffer {
    width: u32,
    height: u32,
    stride: u32,
    format: i32,
    inner: Box<dyn MappedBuffer>,
}

impl GenericCameraBuffer {
    pub fn new(
        width: u32,
        height: u32,
        stride: u32,
        format: i32,
        inner: Box<dyn MappedBuffer>,
    ) -> Self {
        Self { width, height, stride, format, inner }
    }
}

impl MappedBuffer for GenericCameraBuffer {
    fn map(&self) -> Result<(*mut u8, usize), String> { self.inner.map() }
    fn unmap(&self) -> Result<(), String> { self.inner.unmap() }
    fn is_mapped(&self) -> bool { self.inner.is_mapped() }
    fn size(&self) -> usize { self.inner.size() }
    fn fd(&self) -> Option<i32> { self.inner.fd() }
}

impl CameraBuffer for GenericCameraBuffer {
    fn width(&self) -> u32 { self.width }
    fn height(&self) -> u32 { self.height }
    fn stride(&self) -> u32 { self.stride }
    fn format(&self) -> i32 { self.format }
    fn timestamp(&self) -> u64 { 0 }
    fn frame_number(&self) -> u64 { 0 }
}

// ── Allocator registry ─────────────────────────────────────────────────────

/// Global allocator that delegates to a registered backend.
use std::sync::OnceLock;

static GLOBAL_ALLOCATOR: OnceLock<Box<dyn BufferAllocator>> = OnceLock::new();

/// Get the global buffer allocator.
pub fn allocator() -> &'static dyn BufferAllocator {
    GLOBAL_ALLOCATOR.get().map(|b| b.as_ref()).unwrap_or(&HeapAllocator)
}

/// Set the global buffer allocator. Panics if already set.
pub fn set_allocator(alloc: Box<dyn BufferAllocator>) {
    GLOBAL_ALLOCATOR.set(alloc).unwrap_or_else(|_| panic!("Global allocator already set"));
}

/// Allocate a buffer from the global allocator.
pub fn allocate(size: usize) -> Result<Box<dyn MappedBuffer>, String> {
    allocator().allocate(size)
}

/// Allocate a camera frame buffer from the global allocator.
pub fn allocate_frame(
    width: u32,
    height: u32,
    stride: u32,
    format: i32,
) -> Result<Box<dyn CameraBuffer>, String> {
    allocator().allocate_frame(width, height, stride, format)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heap_allocator_roundtrip() {
        let alloc = HeapAllocator;
        let buf = alloc.allocate(1024).unwrap();
        assert_eq!(buf.size(), 1024);

        let (ptr, len) = buf.map().unwrap();
        assert_eq!(len, 1024);
        assert!(!ptr.is_null());

        // Write and read
        unsafe {
            std::ptr::write_bytes(ptr, 0xAB, 1024);
            assert_eq!(*ptr, 0xAB);
        }

        buf.unmap().unwrap();
    }

    #[test]
    fn test_heap_camera_buffer() {
        let alloc = HeapAllocator;
        let buf = alloc.allocate_frame(640, 480, 640, 0x20).unwrap();
        assert_eq!(buf.width(), 640);
        assert_eq!(buf.height(), 480);
        assert_eq!(buf.stride(), 640);
        assert_eq!(buf.format(), 0x20);
    }

    #[test]
    fn test_global_allocator() {
        set_allocator(Box::new(HeapAllocator));
        let buf = allocate(64).unwrap();
        assert_eq!(buf.size(), 64);
    }
}
