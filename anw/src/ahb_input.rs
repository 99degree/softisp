//! Stable worker-owned AHB slots for MNN input tensors.
//!
//! Binding the camera's `AImage` AHB directly is not safe: the image is deleted
//! as soon as `next_frame()` returns, and MNN's Vulkan backend caches the
//! imported AHB for a tensor/geometry instead of re-importing a swapped pointer.
//! This module keeps one `AHardwareBuffer` per worker and stages camera pixels
//! straight into its CPU mapping. MNN imports that same buffer once, before the
//! first resize, so every subsequent frame is already in device memory.

use std::ffi::c_void;
use std::ptr::null_mut;

use cam_types::{AhbBinding, AhbFenceTokens};
use ndk_sys as sys;

use crate::camera::RawFrame;
use crate::pipeline::stage_into;

/// One CPU-mapped AHB bound to a worker's frame input tensor.
///
/// The slot is never swapped while the geometry is unchanged. A geometry change
/// creates a new slot and invalidates the prepared session so the binding is
/// installed before the next `resizeSession()`.
pub(crate) struct AhbInputSlot {
    ahb: *mut sys::AHardwareBuffer,
    map: *mut u8,
    capacity: usize,
    width: u32,
    height: u32,
    generation: u64,
}

// The NDK handles are owned by this struct and touched only on the worker that
// owns the session. The CPU mapping is written before that worker runs MNN.
unsafe impl Send for AhbInputSlot {}
unsafe impl Sync for AhbInputSlot {}

impl std::fmt::Debug for AhbInputSlot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AhbInputSlot")
            .field("width", &self.width)
            .field("height", &self.height)
            .field("capacity", &self.capacity)
            .field("generation", &self.generation)
            .finish_non_exhaustive()
    }
}

fn slot_capacity_bytes(width: u32, height: u32) -> usize {
    // MNN's Vulkan backend sizes int tensors as four bytes per element even
    // when the host tensor is u16, so the AHB needs 4x the staged byte count.
    // A further 2x margin covers padded/fused graph variants without a swap.
    (width as usize)
        .saturating_mul(height as usize)
        .saturating_mul(16)
        .next_power_of_two()
        .max(4096)
}

impl AhbInputSlot {
    pub(crate) fn new(width: u32, height: u32, generation: u64) -> Result<Self, String> {
        let capacity = slot_capacity_bytes(width, height);
        if capacity == 0 || capacity > u32::MAX as usize {
            return Err(format!("invalid AHB input capacity {capacity}"));
        }

        let mut desc: sys::AHardwareBuffer_Desc = unsafe { std::mem::zeroed() };
        desc.width = capacity as u32;
        desc.height = 1;
        desc.layers = 1;
        desc.format = sys::AHardwareBuffer_Format::AHARDWAREBUFFER_FORMAT_BLOB.0;
        desc.usage = sys::AHardwareBuffer_UsageFlags::AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN.0
            | sys::AHardwareBuffer_UsageFlags::AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN.0
            | sys::AHardwareBuffer_UsageFlags::AHARDWAREBUFFER_USAGE_GPU_DATA_BUFFER.0;

        let mut ahb: *mut sys::AHardwareBuffer = null_mut();
        let rc = unsafe { sys::AHardwareBuffer_allocate(&desc, &mut ahb) };
        if rc != 0 || ahb.is_null() {
            return Err(format!(
                "AHardwareBuffer_allocate({capacity}B) failed: rc={rc}"
            ));
        }

        let mut map: *mut c_void = null_mut();
        let rc = unsafe {
            sys::AHardwareBuffer_lock(
                ahb,
                sys::AHardwareBuffer_UsageFlags::AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN.0
                    | sys::AHardwareBuffer_UsageFlags::AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN.0,
                -1,
                null_mut(),
                &mut map,
            )
        };
        if rc != 0 || map.is_null() {
            unsafe { sys::AHardwareBuffer_release(ahb) };
            return Err(format!("AHardwareBuffer_lock(input) failed: rc={rc}"));
        }

        log::info!("input AHB {width}x{height} generation={generation} capacity={capacity}B");
        Ok(Self {
            ahb,
            map: map as *mut u8,
            capacity,
            width,
            height,
            generation,
        })
    }

    pub(crate) fn matches(&self, width: u32, height: u32) -> bool {
        self.width == width && self.height == height
    }

    /// De-stride one camera frame directly into the AHB mapping.
    ///
    /// The slot is owned by one worker, so interior mutation through the stable
    /// mapping is safe; the immutable `bytes()` view is only read by that same
    /// worker until the next call.
    pub(crate) fn stage(&self, frame: &RawFrame) -> Result<(), String> {
        if !self.matches(frame.width, frame.height) {
            return Err(format!(
                "input AHB geometry {}x{} does not match frame {}x{}",
                self.width, self.height, frame.width, frame.height
            ));
        }
        let needed = frame.data.len().saturating_mul(2);
        let bytes = unsafe { std::slice::from_raw_parts_mut(self.map, self.capacity) };
        stage_into(frame, &mut bytes[..self.capacity]).map_err(|e| {
            format!(
                "input AHB capacity {}B too small for frame {needed}B: {e}",
                self.capacity
            )
        })
    }

    pub(crate) fn bytes(&self) -> &[u8] {
        let len = (self.width as usize)
            .saturating_mul(self.height as usize)
            .saturating_mul(2)
            .min(self.capacity);
        unsafe { std::slice::from_raw_parts(self.map, len) }
    }

    pub(crate) fn buffers(&self) -> (AhbBinding, (*mut u8, usize)) {
        let binding = AhbBinding::new(
            self.ahb as *mut c_void,
            self.capacity,
            AhbFenceTokens::new(-1, -1, self.generation),
        );
        (binding, (self.map, self.capacity))
    }
}

impl Drop for AhbInputSlot {
    fn drop(&mut self) {
        if !self.map.is_null() {
            unsafe { sys::AHardwareBuffer_unlock(self.ahb, null_mut()) };
            self.map = null_mut();
        }
        if !self.ahb.is_null() {
            unsafe { sys::AHardwareBuffer_release(self.ahb) };
            self.ahb = null_mut();
        }
    }
}
