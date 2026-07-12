//! Android Camera Adapter — bridges HAL3 device operations to ICameraAdapter.
//!
//! Provides:
//! - `AHardwareBuffer` FFI definitions (lock/unlock camera buffers)
//! - `AndroidCameraAdapter` implementing `ICameraAdapter`
//! - Buffer extraction helpers

#![allow(non_snake_case)]
#![allow(dead_code)]

use std::os::raw::c_void;
use std::ptr;
use std::sync::Arc;

use crate::util;
use cam_hal::buffer::{self as hal_buffer};
use cam_hal::camera::{ByteFrame, CameraState, FrameCallback, ICameraAdapter, StreamConfig};

// ── AHardwareBuffer FFI ─────────────────────────────────────────────────────
//
// FFI to the NDK AHardwareBuffer API (Android API 26+).
// Resolved at runtime via `dlsym` from libnativewindow.so.

pub mod ahardware_buffer {
    use std::os::raw::c_void;

    pub enum AHardwareBuffer {}

    #[repr(C)]
    pub struct AHardwareBuffer_Desc {
        pub width: u32,
        pub height: u32,
        pub layers: u32,
        pub format: u32,
        pub usage: u64,
        pub stride: u32,
        pub reserved: [u32; 4],
    }

    #[repr(C)]
    pub struct ARect {
        pub left: i32,
        pub top: i32,
        pub right: i32,
        pub bottom: i32,
    }

    pub const AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN: u64 = 0x00000003;
    pub const AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN: u64 = 0x00000030;

    extern "C" {
        pub fn AHardwareBuffer_lock(
            buffer: *mut AHardwareBuffer,
            usage: u64,
            fence: i32,
            rect: *const ARect,
            out_bytes: *mut *mut c_void,
        ) -> i32;
        pub fn AHardwareBuffer_unlock(buffer: *mut AHardwareBuffer, fence: *mut i32) -> i32;
        pub fn AHardwareBuffer_describe(
            buffer: *const AHardwareBuffer,
            out_desc: *mut AHardwareBuffer_Desc,
        );
    }
}

pub use ahardware_buffer::*;

// ── CameraBuffer implementation for AHardwareBuffer ─────────────────────────

/// Wraps a raw `mmap`-ed dma-buf fd as a `CameraBuffer`.
/// This is the same approach libcamera uses (no NDK dependency).
#[derive(Debug)]
pub struct MmapFdCameraBuffer {
    fd: std::os::unix::io::RawFd,
    ptr: *mut u8,
    size: usize,
    width: u32,
    height: u32,
    format: i32,
    frame_number: u64,
}

// SAFETY: Only accessed from camera HAL callbacks (serialized).
unsafe impl Send for MmapFdCameraBuffer {}
unsafe impl Sync for MmapFdCameraBuffer {}

impl MmapFdCameraBuffer {
    /// Open a dma-buf fd, determine its size, and mmap it.
    ///
    /// # Safety
    ///
    /// - `fd` must be a valid file descriptor for a dma-buf
    /// - The fd must remain valid for the lifetime of this buffer
    /// - The buffer must not be unmapped externally while this struct exists
    pub unsafe fn new(
        fd: std::os::unix::io::RawFd,
        format: i32,
        width: u32,
        height: u32,
        frame_number: u64,
    ) -> Result<Self, String> {
        // Determine buffer size via lseek
        let size = libc::lseek(fd, 0, libc::SEEK_END);
        if size < 0 {
            return Err("lseek failed on dma-buf fd".to_string());
        }
        let size = size as usize;

        let ptr = libc::mmap(
            std::ptr::null_mut(),
            size,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            fd,
            0,
        );
        if ptr == libc::MAP_FAILED {
            return Err("mmap failed on dma-buf fd".to_string());
        }

        Ok(Self {
            fd,
            ptr: ptr as *mut u8,
            size,
            width,
            height,
            format,
            frame_number,
        })
    }
}

impl Drop for MmapFdCameraBuffer {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe { libc::munmap(self.ptr as *mut libc::c_void, self.size) };
        }
        // Don't close the fd — framework owns it
    }
}

impl cam_hal::buffer::MappedBuffer for MmapFdCameraBuffer {
    fn map(&self) -> Result<(*mut u8, usize), String> {
        Ok((self.ptr, self.size))
    }
    fn unmap(&self) -> Result<(), String> {
        Ok(())
    }
    fn is_mapped(&self) -> bool {
        !self.ptr.is_null()
    }
    fn size(&self) -> usize {
        self.size
    }
    fn fd(&self) -> Option<i32> {
        Some(self.fd)
    }
}

impl cam_hal::buffer::CameraBuffer for MmapFdCameraBuffer {
    fn width(&self) -> u32 {
        self.width
    }
    fn height(&self) -> u32 {
        self.height
    }
    fn stride(&self) -> u32 {
        self.width
    } // stride = width for simple case
    fn format(&self) -> i32 {
        self.format
    }
    fn timestamp(&self) -> u64 {
        0
    }
    fn frame_number(&self) -> u64 {
        self.frame_number
    }
}

/// Wraps an `AHardwareBuffer` pointer as a `cam_hal::buffer::CameraBuffer`.
#[derive(Debug)]
pub struct AHardwareBufferBacked {
    buffer: *mut ahardware_buffer::AHardwareBuffer,
    width: u32,
    height: u32,
    stride: u32,
    format: i32,
    mapped_ptr: std::cell::Cell<*mut u8>,
    size: usize,
    frame_number: u64,
}

// SAFETY: The AHardwareBuffer is externally synchronized by the camera framework.
unsafe impl Send for AHardwareBufferBacked {}
unsafe impl Sync for AHardwareBufferBacked {}

impl AHardwareBufferBacked {
    /// Wrap a raw AHardwareBuffer pointer.
    ///
    /// # Safety
    /// `buffer` must be a valid AHardwareBuffer pointer for the lifetime of this struct.
    pub unsafe fn new(
        buffer: *mut ahardware_buffer::AHardwareBuffer,
        width: u32,
        height: u32,
        format: i32,
        frame_number: u64,
    ) -> Self {
        let mut desc: AHardwareBuffer_Desc = std::mem::zeroed();
        AHardwareBuffer_describe(buffer, &mut desc);
        let stride = desc.stride;
        let bpp = util::hal_format_bpp(format);
        let size = ((stride.max(width) as u64) * (height as u64) * (bpp as u64)) as usize;

        Self {
            buffer,
            width,
            height,
            stride: stride.max(width),
            format,
            mapped_ptr: std::cell::Cell::new(std::ptr::null_mut()),
            size,
            frame_number,
        }
    }
}

impl cam_hal::buffer::MappedBuffer for AHardwareBufferBacked {
    fn map(&self) -> Result<(*mut u8, usize), String> {
        let mut cpu_ptr: *mut c_void = std::ptr::null_mut();
        let ret = unsafe {
            AHardwareBuffer_lock(
                self.buffer,
                AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN,
                -1,
                std::ptr::null(),
                &mut cpu_ptr,
            )
        };
        if ret != 0 {
            return Err(format!("AHardwareBuffer_lock failed: {}", ret));
        }
        self.mapped_ptr.set(cpu_ptr as *mut u8);
        Ok((cpu_ptr as *mut u8, self.size))
    }

    fn unmap(&self) -> Result<(), String> {
        let ret = unsafe { AHardwareBuffer_unlock(self.buffer, std::ptr::null_mut()) };
        self.mapped_ptr.set(std::ptr::null_mut());
        if ret != 0 {
            return Err(format!("AHardwareBuffer_unlock failed: {}", ret));
        }
        Ok(())
    }

    fn is_mapped(&self) -> bool {
        !self.mapped_ptr.get().is_null()
    }

    fn size(&self) -> usize {
        self.size
    }
}

impl cam_hal::buffer::CameraBuffer for AHardwareBufferBacked {
    fn width(&self) -> u32 {
        self.width
    }
    fn height(&self) -> u32 {
        self.height
    }
    fn stride(&self) -> u32 {
        self.stride
    }
    fn format(&self) -> i32 {
        self.format
    }
    fn timestamp(&self) -> u64 {
        0
    }
    fn frame_number(&self) -> u64 {
        self.frame_number
    }
}

/// Allocator backend for AHardwareBuffer.
#[derive(Debug)]
pub struct AHardwareBufferAllocator;

impl cam_hal::buffer::BufferAllocator for AHardwareBufferAllocator {
    fn name(&self) -> &'static str {
        "ahardwarebuffer"
    }

    fn allocate(&self, size: usize) -> Result<Box<dyn cam_hal::buffer::MappedBuffer>, String> {
        // AHardwareBuffer doesn't support arbitrary allocations.
        // Fall back to heap for non-camera buffers.
        Ok(Box::new(hal_buffer::HeapBuffer::new(size)))
    }

    fn allocate_frame(
        &self,
        width: u32,
        height: u32,
        stride: u32,
        format: i32,
    ) -> Result<Box<dyn cam_hal::buffer::CameraBuffer>, String> {
        // For now, delegate to heap. Real impl would create AHardwareBuffer via NDK.
        let bpp = util::hal_format_bpp(format);
        let size = (stride as usize) * (height as usize) * bpp;
        Ok(Box::new(hal_buffer::HeapCameraBuffer::new(
            width,
            height,
            stride,
            format,
            cam_hal::buffer::HeapBuffer::new(size),
        )))
    }
}

// ── Processing function type ────────────────────────────────────────────────
//
// A frame processing function provided by the pipeline manager.
// Takes (raw_bytes, width, height, hal_format) and returns processed bytes.
pub type FrameProcessor =
    Arc<dyn Fn(&[u8], u32, u32, i32) -> Result<Vec<u8>, String> + Send + Sync>;

// ── Buffer locking helper ───────────────────────────────────────────────────

/// Lock an AHardwareBuffer and copy its contents out.
///
/// # Safety
/// `buffer` must be a valid pointer to an AHardwareBuffer.
unsafe fn lock_and_copy_buffer(
    buffer: *mut ahardware_buffer::AHardwareBuffer,
    _stream_format: i32,
    _stream_width: u32,
    _stream_height: u32,
) -> Result<Vec<u8>, i32> {
    if buffer.is_null() {
        return Err(-1);
    }

    let mut desc: AHardwareBuffer_Desc = std::mem::zeroed();
    AHardwareBuffer_describe(buffer, &mut desc);

    let stride = desc.stride;
    let width = if stride > 0 { stride } else { _stream_width };
    let height = _stream_height;

    // Estimate bytes-per-pixel from format
    let bpp = util::hal_format_bpp(_stream_format);
    let buf_size = (width as u64) * (height as u64) * (bpp as u64);

    let mut cpu_ptr: *mut c_void = ptr::null_mut();
    let lock_ret = AHardwareBuffer_lock(
        buffer,
        AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN,
        -1,
        ptr::null(),
        &mut cpu_ptr,
    );
    if lock_ret != 0 {
        log::warn!("AHardwareBuffer_lock failed: {}", lock_ret);
        return Err(lock_ret);
    }

    let data = if buf_size <= usize::MAX as u64 {
        let sz = buf_size as usize;
        core::slice::from_raw_parts(cpu_ptr as *const u8, sz).to_vec()
    } else {
        log::warn!("Buffer size {} exceeds usize", buf_size);
        Vec::new()
    };

    AHardwareBuffer_unlock(buffer, ptr::null_mut());

    Ok(data)
}

// ── AndroidCameraAdapter ────────────────────────────────────────────────────

/// Adapter wrapping a SoftISP pipeline behind the ICameraAdapter trait.
///
/// Stored inside `camera3_device_t.priv_` as a leaked `Box<AndroidCameraAdapter>`.
pub struct AndroidCameraAdapter {
    /// Optional frame processing function (provided by pipeline manager).
    processor: Option<FrameProcessor>,
    /// Current stream configuration.
    config: Option<StreamConfig>,
    /// Frame callback (set by pipeline manager via set_frame_callback).
    callback: Option<FrameCallback>,
    /// Current device state.
    state: CameraState,
    /// Device name / camera ID.
    name: String,
}

// SAFETY: All fields are Send+Sync (processor is Arc<dyn Fn + Send + Sync>,
// callback is Box<dyn Fn + Send + Sync>, others are plain values).
unsafe impl Send for AndroidCameraAdapter {}
unsafe impl Sync for AndroidCameraAdapter {}

impl AndroidCameraAdapter {
    pub fn new(name: &str) -> Self {
        Self {
            processor: None,
            config: None,
            callback: None,
            state: CameraState::Closed,
            name: name.to_string(),
        }
    }

    /// Attach a frame processor (from pipeline manager).
    pub fn set_processor(&mut self, processor: FrameProcessor) {
        self.processor = Some(processor);
    }

    /// Process a raw buffer through the attached processor (or pass-through).
    pub fn process_buffer(
        &self,
        data: &[u8],
        width: u32,
        height: u32,
        format: i32,
    ) -> Result<Vec<u8>, String> {
        if let Some(ref proc) = self.processor {
            (proc)(data, width, height, format)
        } else {
            Ok(data.to_vec()) // pass-through
        }
    }

    /// Lock an AHardwareBuffer and process its contents (read-only).
    ///
    /// # Safety
    /// `buffer` must be valid.
    pub unsafe fn lock_and_process(
        &self,
        buffer: *mut ahardware_buffer::AHardwareBuffer,
        format: i32,
        width: u32,
        height: u32,
    ) -> Result<ByteFrame, String> {
        let data = lock_and_copy_buffer(buffer, format, width, height)
            .map_err(|e| format!("AHB lock failed: {}", e))?;

        let processed = if let Some(ref proc) = self.processor {
            (proc)(&data, width, height, format)?
        } else {
            data
        };

        Ok(ByteFrame {
            data: processed,
            width,
            height,
            format: cam_types::FrameFormat::Rgba8888, // pipeline produces RGBA
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
        })
    }

    /// Lock an AHardwareBuffer for CPU write access.
    /// Returns (pointer, size_in_bytes).
    ///
    /// # Safety
    /// `buffer` must be valid. Caller must unlock with `unlock_buffer`.
    pub unsafe fn lock_for_write(
        buffer: *mut ahardware_buffer::AHardwareBuffer,
    ) -> Result<(*mut u8, usize), String> {
        if buffer.is_null() {
            return Err("Null buffer".to_string());
        }
        let mut cpu_ptr: *mut c_void = std::ptr::null_mut();
        let ret = AHardwareBuffer_lock(
            buffer,
            AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN,
            -1,
            std::ptr::null(),
            &mut cpu_ptr,
        );
        if ret != 0 {
            return Err(format!("AHB lock write failed: {}", ret));
        }
        // We don't know exact size here; caller must know or query via describe
        let mut desc: AHardwareBuffer_Desc = std::mem::zeroed();
        AHardwareBuffer_describe(buffer, &mut desc);
        let size = (desc.stride.max(desc.width) as usize) * (desc.height as usize) * 4;
        Ok((cpu_ptr as *mut u8, size))
    }

    /// Unlock a previously write-locked buffer.
    ///
    /// # Safety
    ///
    /// - `buffer` must be a valid pointer to an AHardwareBuffer
    /// - The buffer must have been previously locked via `lock_buffer`
    pub unsafe fn unlock_buffer(
        buffer: *mut ahardware_buffer::AHardwareBuffer,
    ) -> Result<(), String> {
        let ret = AHardwareBuffer_unlock(buffer, std::ptr::null_mut());
        if ret != 0 {
            return Err(format!("AHB unlock failed: {}", ret));
        }
        Ok(())
    }
}

impl ICameraAdapter for AndroidCameraAdapter {
    fn source_type(&self) -> cam_types::CameraSourceType {
        cam_types::CameraSourceType::AndroidHal
    }

    fn open(&mut self, config: &StreamConfig) -> Result<(), String> {
        self.config = Some(config.clone());
        self.state = CameraState::Open;
        log::info!(
            "Camera '{}' opened: {}x{}",
            self.name,
            config.width,
            config.height
        );
        Ok(())
    }

    fn close(&mut self) {
        self.state = CameraState::Closed;
        self.config = None;
        log::info!("Camera '{}' closed", self.name);
    }

    fn start_streaming(&mut self) -> Result<(), String> {
        self.state = CameraState::Streaming;
        log::info!("Camera '{}' streaming started", self.name);
        Ok(())
    }

    fn stop_streaming(&mut self) {
        self.state = CameraState::Open;
        log::info!("Camera '{}' streaming stopped", self.name);
    }

    fn set_frame_callback(&mut self, callback: FrameCallback) {
        self.callback = Some(callback);
    }

    fn state(&self) -> CameraState {
        self.state
    }

    fn device_name(&self) -> &str {
        &self.name
    }

    fn send_frame(&self, frame: ByteFrame) -> Result<(), String> {
        if let Some(ref cb) = self.callback {
            (cb)(frame);
            Ok(())
        } else {
            Err("No frame callback registered".to_string())
        }
    }
}
