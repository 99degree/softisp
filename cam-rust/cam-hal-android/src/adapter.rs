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

    extern "C" {
        pub fn AHardwareBuffer_lock(
            buffer: *mut AHardwareBuffer,
            usage: u64,
            fence: i32,
            rect: *const ARect,
            out_bytes: *mut *mut c_void,
        ) -> i32;
        pub fn AHardwareBuffer_unlock(
            buffer: *mut AHardwareBuffer,
            fence: *mut i32,
        ) -> i32;
        pub fn AHardwareBuffer_describe(
            buffer: *const AHardwareBuffer,
            out_desc: *mut AHardwareBuffer_Desc,
        );
    }
}

pub use ahardware_buffer::*;

// ── Processing function type ────────────────────────────────────────────────
//
// A frame processing function provided by the pipeline manager.
// Takes (raw_bytes, width, height, hal_format) and returns processed bytes.
pub type FrameProcessor = Arc<dyn Fn(&[u8], u32, u32, i32) -> Result<Vec<u8>, String> + Send + Sync>;

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
    let bpp = match _stream_format {
        0x20 => 2,    // RAW16
        0x25 | 0x26 => 2, // RAW10/12
        0x11 => 1,    // NV21 Y-plane
        0x22 => 4,    // IMPLEMENTATION_DEFINED (assume RGBA)
        _ => 2,
    };
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
    pub fn process_buffer(&self, data: &[u8], width: u32, height: u32, format: i32) -> Result<Vec<u8>, String> {
        if let Some(ref proc) = self.processor {
            (proc)(data, width, height, format)
        } else {
            Ok(data.to_vec()) // pass-through
        }
    }

    /// Lock an AHardwareBuffer and process its contents.
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
            timestamp: 0, // TODO: get from capture request
        })
    }
}

impl ICameraAdapter for AndroidCameraAdapter {
    fn source_type(&self) -> cam_types::CameraSourceType {
        cam_types::CameraSourceType::AndroidHal
    }

    fn open(&mut self, config: &StreamConfig) -> Result<(), String> {
        self.config = Some(config.clone());
        self.state = CameraState::Open;
        log::info!("Camera '{}' opened: {}x{}", self.name, config.width, config.height);
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
