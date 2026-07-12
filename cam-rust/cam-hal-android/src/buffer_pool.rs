//! Hardware Buffer Pool for zero-copy Android camera HAL
//!
//! Manages a pool of AHardwareBuffer instances for efficient frame buffer
//! allocation across camera preview, video, and still capture streams.
//!
//! Key features:
//! - Zero-copy: avoids extra memcpy from camera HAL to ISP
//! - Stream-aware: allocates correct format per use case
//! - Refcounted: tracks which buffers are in-use vs available
//! - Multi-format: supports RAW, RGBA, YUV, JPEG concurrently

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

/// Pixel format for Android hardware buffers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HalPixelFormat {
    /// RGBA8888, packed
    Rgba8888,
    /// RAW Bayer 8/10/12/16
    RawBayer,
    /// YUV 4:2:0 semi-planar (NV12)
    Yuv420SemiPlanar,
    /// YUV 4:2:0 planar (YV12/I420)
    Yuv420Planar,
    /// RGB565 packed
    Rgb565,
    /// JPEG compressed
    Jpeg,
}

impl HalPixelFormat {
    pub fn bytes_per_pixel(&self) -> usize {
        match self {
            Self::Rgba8888 => 4,
            Self::RawBayer => 2,         // 16-bit per pixel pack
            Self::Yuv420SemiPlanar => 1, // approximate
            Self::Yuv420Planar => 1,
            Self::Rgb565 => 2,
            Self::Jpeg => 1,
        }
    }

    pub fn is_bayer(&self) -> bool {
        matches!(self, Self::RawBayer)
    }
}

/// Hardware buffer description matching AHardwareBuffer_Desc
#[derive(Debug, Clone, Copy)]
pub struct HalBufferDesc {
    pub width: u32,
    pub height: u32,
    pub format: HalPixelFormat,
    pub usage: u64,
    pub stride: u32,
}

impl HalBufferDesc {
    pub fn new(width: u32, height: u32, format: HalPixelFormat) -> Self {
        Self {
            width,
            height,
            format,
            usage: 0, // default: GPU read/write + camera capture
            stride: width,
        }
    }

    pub fn size_bytes(&self) -> usize {
        self.stride as usize * self.height as usize * self.format.bytes_per_pixel()
    }

    pub fn with_usage(mut self, usage: u64) -> Self {
        self.usage = usage;
        self
    }

    pub fn with_stride(mut self, stride: u32) -> Self {
        self.stride = stride;
        self
    }
}

/// Hardware buffer handle (opaque for this stub; would be AHardwareBuffer*)
#[derive(Debug, Clone, Copy)]
pub struct HalBufferHandle {
    pub id: u64,
    pub desc: HalBufferDesc,
    /// Is this buffer currently locked by GPU/CPU?
    pub locked: bool,
}

impl HalBufferHandle {
    pub fn new(id: u64, desc: HalBufferDesc) -> Self {
        Self {
            id,
            desc,
            locked: false,
        }
    }
}

/// Pool of hardware buffers tracked by format and stream
pub struct HardwareBufferPool {
    /// Buffer format → available buffers (free list)
    free_buffers: Mutex<HashMap<HalPixelFormat, VecDeque<HalBufferHandle>>>,
    /// Live buffer count per format (for monitoring)
    live_count: Mutex<HashMap<HalPixelFormat, usize>>,
    /// Total allocated capacity per format
    capacity: Mutex<HashMap<HalPixelFormat, usize>>,
    /// Pool max size per format
    pool_max: usize,
    /// Next buffer ID to assign
    next_id: Mutex<u64>,
}

impl HardwareBufferPool {
    pub fn new(pool_max: usize) -> Self {
        Self {
            free_buffers: Mutex::new(HashMap::new()),
            live_count: Mutex::new(HashMap::new()),
            capacity: Mutex::new(HashMap::new()),
            pool_max,
            next_id: Mutex::new(1),
        }
    }

    /// Acquire a buffer for the given format
    pub fn acquire(&self, desc: HalBufferDesc) -> Result<HalBufferHandle, String> {
        let format = desc.format;
        let mut free_buffers = self.free_buffers.lock().unwrap();
        let mut live_count = self.live_count.lock().unwrap();
        let mut capacity = self.capacity.lock().unwrap();

        // Try to reuse a free buffer if dimensions match
        if let Some(queue) = free_buffers.get_mut(&format) {
            while let Some(buf) = queue.pop_front() {
                if buf.desc.width == desc.width && buf.desc.height == desc.height {
                    *live_count.entry(format).or_insert(0) += 1;
                    return Ok(buf);
                }
            }
        }

        // Reuse exhausted — allocate new buffer
        let cap = *capacity.entry(format).or_insert(0);
        if cap >= self.pool_max {
            return Err(format!(
                "HardwareBufferPool: pool exhausted for format {:?}",
                format
            ));
        }

        let id = {
            let mut next = self.next_id.lock().unwrap();
            let id = *next;
            *next += 1;
            id
        };

        *capacity.entry(format).or_insert(0) += 1;
        *live_count.entry(format).or_insert(0) += 1;

        // In real impl, AHardwareBuffer_allocate would be called here
        Ok(HalBufferHandle::new(id, desc))
    }

    /// Release a buffer back to the pool
    pub fn release(&self, buffer: HalBufferHandle) {
        let format = buffer.desc.format;
        let mut free_buffers = self.free_buffers.lock().unwrap();
        let mut live_count = self.live_count.lock().unwrap();

        free_buffers.entry(format).or_default().push_back(buffer);
        if let Some(count) = live_count.get_mut(&format) {
            *count = count.saturating_sub(1);
        }
    }

    /// Get live buffer count for diagnostic purposes
    pub fn live_count(&self, format: HalPixelFormat) -> usize {
        *self.live_count.lock().unwrap().get(&format).unwrap_or(&0)
    }

    /// Get total capacity for diagnostic purposes
    pub fn capacity(&self, format: HalPixelFormat) -> usize {
        *self.capacity.lock().unwrap().get(&format).unwrap_or(&0)
    }

    /// Drain all free buffers (used on shutdown)
    pub fn drain(&self) {
        let mut free_buffers = self.free_buffers.lock().unwrap();
        free_buffers.clear();
    }
}

/// Thread-safe reference-counted buffer pool
pub type SharedBufferPool = Arc<HardwareBufferPool>;

#[cfg(test)]
mod tests {
    use super::*;

    fn test_desc(format: HalPixelFormat) -> HalBufferDesc {
        HalBufferDesc::new(1920, 1080, format)
    }

    #[test]
    fn test_pixel_format_bpp() {
        assert_eq!(HalPixelFormat::Rgba8888.bytes_per_pixel(), 4);
        assert_eq!(HalPixelFormat::RawBayer.bytes_per_pixel(), 2);
        assert!(HalPixelFormat::RawBayer.is_bayer());
        assert!(!HalPixelFormat::Rgba8888.is_bayer());
    }

    #[test]
    fn test_buffer_description() {
        let desc = HalBufferDesc::new(1920, 1080, HalPixelFormat::Rgba8888)
            .with_stride(1920)
            .with_usage(0x100);
        assert_eq!(desc.width, 1920);
        assert_eq!(desc.height, 1080);
        assert_eq!(desc.stride, 1920);
        assert_eq!(desc.size_bytes(), 1920 * 1080 * 4);
    }

    #[test]
    fn test_pool_acquire_release() {
        let pool = HardwareBufferPool::new(4);
        let buf = pool.acquire(test_desc(HalPixelFormat::Rgba8888)).unwrap();
        assert_eq!(pool.live_count(HalPixelFormat::Rgba8888), 1);
        pool.release(buf);
        assert_eq!(pool.live_count(HalPixelFormat::Rgba8888), 0);
    }

    #[test]
    fn test_pool_reuses_released_buffers() {
        let pool = HardwareBufferPool::new(4);
        let buf1 = pool.acquire(test_desc(HalPixelFormat::Rgba8888)).unwrap();
        let id1 = buf1.id;
        pool.release(buf1);

        let buf2 = pool.acquire(test_desc(HalPixelFormat::Rgba8888)).unwrap();
        assert_eq!(buf2.id, id1, "Pool should reuse released buffer");
    }

    #[test]
    fn test_pool_capacity_limit() {
        let pool = HardwareBufferPool::new(2);
        let b1 = pool.acquire(test_desc(HalPixelFormat::Rgba8888)).unwrap();
        let b2 = pool.acquire(test_desc(HalPixelFormat::Rgba8888)).unwrap();
        let result = pool.acquire(test_desc(HalPixelFormat::Rgba8888));
        assert!(result.is_err(), "Pool should reject excess allocations");
        drop(b1);
        drop(b2);
    }

    #[test]
    fn test_pool_separates_formats() {
        let pool = HardwareBufferPool::new(8);
        let rgba = pool.acquire(test_desc(HalPixelFormat::Rgba8888)).unwrap();
        let yuv = pool
            .acquire(test_desc(HalPixelFormat::Yuv420SemiPlanar))
            .unwrap();
        pool.release(rgba);
        pool.release(yuv);
        assert_eq!(pool.capacity(HalPixelFormat::Rgba8888), 1);
        assert_eq!(pool.capacity(HalPixelFormat::Yuv420SemiPlanar), 1);
    }

    #[test]
    fn test_pool_drain() {
        let pool = HardwareBufferPool::new(8);
        let b1 = pool.acquire(test_desc(HalPixelFormat::Rgba8888)).unwrap();
        let b2 = pool.acquire(test_desc(HalPixelFormat::Rgba8888)).unwrap();
        pool.release(b1);
        pool.release(b2);
        pool.drain();
        assert_eq!(pool.capacity(HalPixelFormat::Rgba8888), 2);
        // Capacity remains but free list is empty
        let next = pool.acquire(test_desc(HalPixelFormat::Rgba8888));
        assert!(
            next.is_ok(),
            "Pool should still allocate new buffers after drain"
        );
    }
}
