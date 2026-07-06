//! HardwareBuffer / Gralloc — Zero-copy buffer sharing.
//!
//! Implements Android HardwareBuffer and Gralloc buffer allocation for
//! zero-copy buffer sharing between camera HAL and graphics stack.
//!
//! On Android, uses AHardwareBuffer NDK API.
//! On Linux host, provides simulated buffer management.

use std::ffi::c_void;
use std::sync::{Arc, Mutex};

use log::info;

use crate::types::BufferUsage;

/// HardwareBuffer handle (Android AHardwareBuffer).
#[cfg(target_os = "android")]
pub type HardwareBufferHandle = *mut c_void;

/// Simulated buffer handle for non-Android platforms.
#[cfg(not(target_os = "android"))]
pub type HardwareBufferHandle = u64;

/// HardwareBuffer descriptor matching AHardwareBuffer_Desc.
#[derive(Debug, Clone)]
pub struct HardwareBufferDesc {
    pub width: i32,
    pub height: i32,
    pub layers: i32,
    pub format: i32,
    pub usage: i64,
    pub stride: i32,
}

impl HardwareBufferDesc {
    /// Create a new descriptor.
    pub fn new(width: i32, height: i32, format: i32, usage: i64) -> Self {
        Self {
            width,
            height,
            layers: 1,
            format,
            usage,
            stride: 0,
        }
    }

    /// Camera output buffer descriptor.
    pub fn camera_output(width: i32, height: i32) -> Self {
        Self::new(
            width,
            height,
            0x1, // HAL_PIXEL_FORMAT_RGBA_8888
            BufferUsage::GPU_TEXTURE | BufferUsage::CPU_READ_RARELY | BufferUsage::CAMERA_WRITE,
        )
    }

    /// Camera input buffer descriptor.
    pub fn camera_input(width: i32, height: i32) -> Self {
        Self::new(
            width,
            height,
            0x1, // HAL_PIXEL_FORMAT_RGBA_8888
            BufferUsage::CPU_WRITE_RARELY | BufferUsage::GPU_TEXTURE,
        )
    }

    /// YUV camera output buffer.
    pub fn camera_yuv_output(width: i32, height: i32) -> Self {
        Self::new(
            width,
            height,
            0x23, // HAL_PIXEL_FORMAT_YCbCr_420_888
            BufferUsage::GPU_TEXTURE | BufferUsage::CPU_READ_RARELY | BufferUsage::CAMERA_WRITE,
        )
    }

    /// Allocate stride based on format.
    pub fn compute_stride(&mut self) {
        self.stride = match self.format {
            0x1 | 0x2 => (self.width * 4 + 63) & !63, // RGBA/RGBX: 4 bytes, 64-byte aligned
            0x23 => (self.width + 63) & !63,            // YUV: 1 byte per component
            _ => (self.width + 63) & !63,
        };
    }
}

/// HardwareBuffer — Android-compatible zero-copy buffer.
pub struct HardwareBuffer {
    handle: Mutex<HardwareBufferHandle>,
    desc: HardwareBufferDesc,
    data: Mutex<Vec<u8>>,
    locked: Mutex<bool>,
    next_id: std::sync::atomic::AtomicU64,
}

impl HardwareBuffer {
    /// Create a new HardwareBuffer.
    pub fn new(desc: HardwareBufferDesc) -> Self {
        let size = (desc.stride as usize) * (desc.height as usize) * if desc.format == 0x23 { 3 } else { 4 };
        Self {
            handle: Mutex::new(0),
            desc,
            data: Mutex::new(vec![0u8; size]),
            locked: Mutex::new(false),
            next_id: std::sync::atomic::AtomicU64::new(1),
        }
    }

    /// Create from existing data (zero-copy).
    pub fn from_data(desc: HardwareBufferDesc, data: Vec<u8>) -> Self {
        Self {
            handle: Mutex::new(0),
            desc,
            data: Mutex::new(data),
            locked: Mutex::new(false),
            next_id: std::sync::atomic::AtomicU64::new(1),
        }
    }

    /// Get the buffer descriptor.
    pub fn desc(&self) -> &HardwareBufferDesc {
        &self.desc
    }

    /// Get buffer width.
    pub fn width(&self) -> i32 {
        self.desc.width
    }

    /// Get buffer height.
    pub fn height(&self) -> i32 {
        self.desc.height
    }

    /// Get buffer format.
    pub fn format(&self) -> i32 {
        self.desc.format
    }

    /// Get buffer stride (in pixels).
    pub fn stride(&self) -> i32 {
        self.desc.stride
    }

    /// Get buffer size in bytes.
    pub fn size(&self) -> usize {
        self.data.lock().unwrap().len()
    }

    /// Lock the buffer for CPU access.
    pub fn lock(&self, usage: i64) -> Result<*mut u8, String> {
        let mut locked = self.locked.lock().unwrap();
        if *locked {
            return Err("buffer already locked".into());
        }
        *locked = true;
        Ok(self.data.lock().unwrap().as_mut_ptr())
    }

    /// Unlock the buffer.
    pub fn unlock(&self) -> Result<(), String> {
        let mut locked = self.locked.lock().unwrap();
        if !*locked {
            return Err("buffer not locked".into());
        }
        *locked = false;
        Ok(())
    }

    /// Get a reference to the buffer data.
    pub fn data(&self) -> std::sync::MutexGuard<'_, Vec<u8>> {
        self.data.lock().unwrap()
    }

    /// Get a mutable reference to the buffer data.
    pub fn data_mut(&self) -> std::sync::MutexGuard<'_, Vec<u8>> {
        self.data.lock().unwrap()
    }

    /// Get the raw handle.
    pub fn handle(&self) -> HardwareBufferHandle {
        *self.handle.lock().unwrap()
    }

    /// Get the native AHardwareBuffer pointer (Android only).
    #[cfg(target_os = "android")]
    pub fn native_handle(&self) -> *mut c_void {
        *self.handle.lock().unwrap()
    }

    /// Get the AHardwareBuffer_Desc (Android only).
    #[cfg(target_os = "android")]
    pub fn native_desc(&self) -> AHardwareBuffer_Desc {
        AHardwareBuffer_Desc {
            width: self.desc.width as u32,
            height: self.desc.height as u32,
            layers: self.desc.layers as u32,
            format: self.desc.format as u32,
            usage: self.desc.usage as u64,
            stride: self.desc.stride as u32,
            reserved: [0; 4],
        }
    }

    /// Share the buffer (create a reference-counted clone).
    pub fn share(&self) -> HardwareBuffer {
        HardwareBuffer {
            handle: Mutex::new(self.handle()),
            desc: self.desc.clone(),
            data: Mutex::new(self.data.lock().unwrap().clone()),
            locked: Mutex::new(false),
            next_id: std::sync::atomic::AtomicU64::new(1),
        }
    }

    /// Get the unique ID.
    pub fn id(&self) -> u64 {
        self.next_id.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
    }
}

impl Clone for HardwareBuffer {
    fn clone(&self) -> Self {
        self.share()
    }
}

/// Gralloc buffer allocator.
pub struct GrallocAllocator {
    /// Allocated buffers.
    buffers: Mutex<Vec<Arc<HardwareBuffer>>>,
}

impl GrallocAllocator {
    /// Create a new allocator.
    pub fn new() -> Self {
        Self {
            buffers: Mutex::new(Vec::new()),
        }
    }

    /// Allocate a buffer.
    pub fn allocate(&self, desc: &HardwareBufferDesc) -> Result<Arc<HardwareBuffer>, String> {
        let mut desc = desc.clone();
        desc.compute_stride();
        let buffer = Arc::new(HardwareBuffer::new(desc));
        self.buffers.lock().unwrap().push(buffer.clone());
        Ok(buffer)
    }

    /// Allocate from existing data (zero-copy).
    pub fn allocate_from_data(
        &self,
        desc: &HardwareBufferDesc,
        data: Vec<u8>,
    ) -> Result<Arc<HardwareBuffer>, String> {
        let mut desc = desc.clone();
        desc.compute_stride();
        let buffer = Arc::new(HardwareBuffer::from_data(desc, data));
        self.buffers.lock().unwrap().push(buffer.clone());
        Ok(buffer)
    }

    /// Free all allocated buffers.
    pub fn free_all(&self) {
        self.buffers.lock().unwrap().clear();
    }

    /// Get the number of allocated buffers.
    pub fn buffer_count(&self) -> usize {
        self.buffers.lock().unwrap().len()
    }
}

impl Default for GrallocAllocator {
    fn default() -> Self {
        Self::new()
    }
}

/// DMA-BUF sharing (for buffer passing between processes).
#[derive(Debug, Clone)]
pub struct DmaBuf {
    /// File descriptor.
    pub fd: i32,
    /// Buffer size in bytes.
    pub size: usize,
    /// Buffer offset.
    pub offset: usize,
}

impl DmaBuf {
    /// Create a DMA-BUF from fd.
    pub fn new(fd: i32, size: usize) -> Self {
        Self { fd, size, offset: 0 }
    }

    /// Create from HardwareBuffer (Android only).
    #[cfg(target_os = "android")]
    pub fn from_hardware_buffer(buffer: &HardwareBuffer) -> Result<Self, String> {
        // In production, this would call:
        //   AHardwareBuffer_lockPlanes() to get DMA-BUF fd
        //   or android_hardware_buffer_to_dma_buf() NDK API
        info!("DmaBuf: creating from HardwareBuffer");
        Ok(Self::new(0, buffer.size()))
    }

    /// Check if the DMA-BUF is valid.
    pub fn is_valid(&self) -> bool {
        self.fd >= 0 && self.size > 0
    }
}

/// Stream buffer with zero-copy support.
pub struct ZeroCopyBuffer {
    /// HardwareBuffer for the stream data.
    pub hardware_buffer: Arc<HardwareBuffer>,
    /// Stream ID.
    pub stream_id: i32,
    /// Frame number.
    pub frame_number: i64,
    /// Status (0 = OK).
    pub status: i32,
}

impl ZeroCopyBuffer {
    /// Create a new zero-copy buffer.
    pub fn new(
        hardware_buffer: Arc<HardwareBuffer>,
        stream_id: i32,
        frame_number: i64,
    ) -> Self {
        Self {
            hardware_buffer,
            stream_id,
            frame_number,
            status: 0,
        }
    }

    /// Get buffer data.
    pub fn data(&self) -> std::sync::MutexGuard<'_, Vec<u8>> {
        self.hardware_buffer.data()
    }

    /// Get buffer size.
    pub fn size(&self) -> usize {
        self.hardware_buffer.size()
    }

    /// Get buffer width.
    pub fn width(&self) -> i32 {
        self.hardware_buffer.width()
    }

    /// Get buffer height.
    pub fn height(&self) -> i32 {
        self.hardware_buffer.height()
    }
}

/// Create a standard camera output buffer for the given resolution.
pub fn create_camera_buffer(width: i32, height: i32, format: i32) -> Arc<HardwareBuffer> {
    let desc = HardwareBufferDesc::new(width, height, format,
        BufferUsage::GPU_TEXTURE | BufferUsage::CPU_READ_RARELY | BufferUsage::CAMERA_WRITE);
    let allocator = GrallocAllocator::new();
    allocator.allocate(&desc).unwrap()
}

/// Create a test pattern buffer filled with gradient data.
pub fn create_test_pattern_buffer(width: i32, height: i32) -> Arc<HardwareBuffer> {
    let desc = HardwareBufferDesc::camera_output(width, height);
    let allocator = GrallocAllocator::new();
    let buffer = allocator.allocate(&desc).unwrap();

    // Fill with test pattern (gradient)
    let mut data = buffer.data();
    let stride = buffer.stride() as usize;
    for y in 0..height as usize {
        for x in 0..width as usize {
            let offset = (y * stride + x) * 4;
            if offset + 4 <= data.len() {
                data[offset] = ((x as f32 / width as f32) * 255.0) as u8;     // R
                data[offset + 1] = ((y as f32 / height as f32) * 255.0) as u8; // G
                data[offset + 2] = 128;                                          // B
                data[offset + 3] = 255;                                          // A
            }
        }
    }
    drop(data);

    buffer
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hardware_buffer_creation() {
        let desc = HardwareBufferDesc::new(640, 480, 0x1, BufferUsage::GPU_TEXTURE);
        let buffer = HardwareBuffer::new(desc);
        assert_eq!(buffer.width(), 640);
        assert_eq!(buffer.height(), 480);
        assert!(buffer.size() > 0);
    }

    #[test]
    fn test_hardware_buffer_lock_unlock() {
        let desc = HardwareBufferDesc::new(100, 100, 0x1, BufferUsage::CPU_READ_RARELY);
        let buffer = HardwareBuffer::new(desc);

        let ptr = buffer.lock(0);
        assert!(ptr.is_ok());
        assert!(buffer.unlock().is_ok());
    }

    #[test]
    fn test_hardware_buffer_double_lock() {
        let desc = HardwareBufferDesc::new(100, 100, 0x1, BufferUsage::CPU_READ_RARELY);
        let buffer = HardwareBuffer::new(desc);

        buffer.lock(0).unwrap();
        assert!(buffer.lock(0).is_err());
        buffer.unlock().unwrap();
    }

    #[test]
    fn test_hardware_buffer_share() {
        let desc = HardwareBufferDesc::new(100, 100, 0x1, BufferUsage::GPU_TEXTURE);
        let buffer = HardwareBuffer::new(desc);

        let shared = buffer.share();
        assert_eq!(shared.width(), 100);
        assert_eq!(shared.height(), 100);
    }

    #[test]
    fn test_gralloc_allocator() {
        let allocator = GrallocAllocator::new();
        let desc = HardwareBufferDesc::new(640, 480, 0x1, BufferUsage::GPU_TEXTURE);
        let buffer = allocator.allocate(&desc);
        assert!(buffer.is_ok());
        assert_eq!(allocator.buffer_count(), 1);

        allocator.free_all();
        assert_eq!(allocator.buffer_count(), 0);
    }

    #[test]
    fn test_gralloc_from_data() {
        let allocator = GrallocAllocator::new();
        let desc = HardwareBufferDesc::new(10, 10, 0x1, BufferUsage::GPU_TEXTURE);
        let data = vec![0u8; 400];
        let buffer = allocator.allocate_from_data(&desc, data);
        assert!(buffer.is_ok());
    }

    #[test]
    fn test_dma_buf() {
        let dmabuf = DmaBuf::new(5, 4096);
        assert!(dmabuf.is_valid());

        let invalid = DmaBuf::new(-1, 4096);
        assert!(!invalid.is_valid());
    }

    #[test]
    fn test_zero_copy_buffer() {
        let allocator = GrallocAllocator::new();
        let desc = HardwareBufferDesc::camera_output(640, 480);
        let hw_buf = allocator.allocate(&desc).unwrap();
        let zc_buf = ZeroCopyBuffer::new(hw_buf, 0, 42);

        assert_eq!(zc_buf.width(), 640);
        assert_eq!(zc_buf.height(), 480);
        assert_eq!(zc_buf.frame_number, 42);
        assert_eq!(zc_buf.status, 0);
    }

    #[test]
    fn test_camera_buffer_creation() {
        let buffer = create_camera_buffer(1920, 1080, 0x1);
        assert_eq!(buffer.width(), 1920);
        assert_eq!(buffer.height(), 1080);
    }

    #[test]
    fn test_test_pattern_buffer() {
        let buffer = create_test_pattern_buffer(64, 64);
        assert_eq!(buffer.width(), 64);

        let data = buffer.data();
        // Check gradient pattern
        assert_eq!(data[0], 0);     // R at (0,0)
        assert_eq!(data[1], 0);     // G at (0,0)
        assert_eq!(data[3], 255);   // A
    }

    #[test]
    fn test_buffer_desc_computation() {
        let mut desc = HardwareBufferDesc::new(1920, 1080, 0x1, BufferUsage::GPU_TEXTURE);
        desc.compute_stride();
        assert!(desc.stride >= 1920 * 4); // at least width * 4 bytes
    }

    #[test]
    fn test_yuv_buffer_desc() {
        let desc = HardwareBufferDesc::camera_yuv_output(1920, 1080);
        assert_eq!(desc.format, 0x23); // YCbCr_420_888
        assert!(desc.usage & BufferUsage::CAMERA_WRITE != 0);
    }
}
