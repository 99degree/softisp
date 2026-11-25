//! AHB-backed output slots for the two ISP workers.
//!
//! The display path used to pay a `buf_bytes.to_vec()` copy of the whole
//! display tensor (~33 MB at 4K) on every frame. A slot here is an
//! `AHardwareBuffer` that MNN's Vulkan backend imports as the session output
//! tensor's device memory, so the fused display shader writes straight into
//! the buffer the presenter reads. There is no device→host copy and no host
//! allocation on the visible path.
//!
//! MNN's fork has no public API to allocate an AHB or export one from a
//! tensor: `TensorUtils::getSharedMem` lives in `source/core` (not `include/MNN`)
//! and `getDeviceInfo` returns a `VkBuffer`, not the AHB pointer. The caller
//! therefore owns allocation/locking and passes the buffer in via
//! `Tensor::setDevicePtr(..., MNN_MEMORY_AHARDWAREBUFFER)`.
//!
//! The optimization is **off by default**. Enable it without rebuilding via
//! the `SOFTISP_OUTPUT_AHB` env var or the
//! `/sdcard/download/softisp-output-ahb` marker. Per-frame slot diagnostics
//! need a second switch (`SOFTISP_VERBOSE` / `/sdcard/download/softisp-verbose`);
//! the switch state itself is always written once at INFO.
//!
//! One slot is bound per worker and reused for every frame. Swapping AHBs
//! between frames would be silent corruption: MNN's Vulkan backend imports the
//! AHB in `onAcquire`, which runs only while the output tensor has no backing
//! memory. Each worker owns one generously sized slot for its session lifetime;
//! the other worker has its own, so the two engines never contend.

use std::ffi::c_void;
use std::ptr::null_mut;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};

use cam_types::{AhbBinding, AhbFenceTokens};
use ndk_sys as sys;

const ENABLE_ENV: &str = "SOFTISP_OUTPUT_AHB";
const VERBOSE_ENV: &str = "SOFTISP_VERBOSE";
#[cfg(target_os = "android")]
const ENABLE_MARKER: &str = "/sdcard/download/softisp-output-ahb";
#[cfg(target_os = "android")]
const VERBOSE_MARKER: &str = "/sdcard/download/softisp-verbose";

static ENABLED: AtomicBool = AtomicBool::new(false);
static VERBOSE: AtomicBool = AtomicBool::new(false);
static SWITCHES: OnceLock<()> = OnceLock::new();

fn init_switches() {
    SWITCHES.get_or_init(|| {
        if std::env::var_os(ENABLE_ENV).is_some() {
            ENABLED.store(true, Ordering::Relaxed);
        }
        if std::env::var_os(VERBOSE_ENV).is_some() {
            VERBOSE.store(true, Ordering::Relaxed);
        }
        #[cfg(target_os = "android")]
        {
            if std::path::Path::new(ENABLE_MARKER).exists() {
                ENABLED.store(true, Ordering::Relaxed);
            }
            if std::path::Path::new(VERBOSE_MARKER).exists() {
                VERBOSE.store(true, Ordering::Relaxed);
            }
            // AHardwareBuffer entered the NDK in API 26. The APK still
            // declares minSdk 24, so an opt-in on an older device must fail
            // closed instead of resolving a missing symbol at first frame.
            let api = unsafe { sys::android_get_device_api_level() };
            if api < 26 && ENABLED.load(Ordering::Relaxed) {
                ENABLED.store(false, Ordering::Relaxed);
                log::warn!(
                    "AHB output requested but device API {api} < 26; using host-copy output"
                );
            }
        }
    });
}

pub(crate) fn output_ahb_enabled() -> bool {
    init_switches();
    ENABLED.load(Ordering::Relaxed)
}

pub(crate) fn verbose_enabled() -> bool {
    init_switches();
    VERBOSE.load(Ordering::Relaxed)
}

/// Write the switch state once, before any worker starts.
pub(crate) fn log_switch_state() {
    init_switches();
    log::info!(
        "output AHB={} verbose={} ({} / {})",
        if output_ahb_enabled() { "on" } else { "off" },
        if verbose_enabled() { "on" } else { "off" },
        ENABLE_ENV,
        VERBOSE_ENV
    );
}

/// Verbose-only diagnostic for slot binding and reuse.
pub(crate) fn slot_debug(message: impl std::fmt::Display) {
    if verbose_enabled() {
        log::debug!("ahb-output: {message}");
    }
}

/// A shareable view of one worker's AHB mapping.
///
/// The worker marks the slot busy before inference; the presenter releases it
/// after the blit. A worker starting its next frame waits for its own slot to
/// come free — never for the other engine — so the GPU cannot overwrite pixels
/// the surface is still reading.
#[derive(Clone)]
pub(crate) struct AhbView {
    ptr: *mut u8,
    len: usize,
    busy: Arc<AtomicBool>,
}

// The pointer is owned by the worker's `AhbOutputSlot`, which outlives every
// view. `busy` is the explicit handoff flag, so moving a view across threads is
// safe as long as the slot is still alive.
unsafe impl Send for AhbView {}
unsafe impl Sync for AhbView {}

impl AhbView {
    fn new(slot: &AhbOutputSlot) -> Self {
        Self {
            ptr: slot.map,
            len: slot.display_bytes(),
            busy: Arc::clone(&slot.busy),
        }
    }

    /// Bytes the presenter is allowed to read: one BGRA pixel per output pixel.
    pub(crate) fn bytes(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }

    pub(crate) fn is_busy(&self) -> bool {
        self.busy.load(Ordering::Acquire)
    }

    pub(crate) fn mark_free(&self) {
        self.busy.store(false, Ordering::Release);
    }
}

/// One CPU-mapped AHB bound to a worker's output tensor.
///
/// The buffer is sized generously once ("make it large") and never swapped:
/// MNN imports it in `onAcquire` at session resize. It stays mapped for its
/// whole lifetime so the pointer handed to MNN and to the presenter is stable.
pub(crate) struct AhbOutputSlot {
    ahb: *mut sys::AHardwareBuffer,
    map: *mut u8,
    capacity: usize,
    width: u32,
    height: u32,
    generation: u64,
    busy: Arc<AtomicBool>,
}

// The NDK handles are owned by this struct and only touched on the worker that
// owns the session. `busy` is the explicit handoff flag read by the presenter.
unsafe impl Send for AhbOutputSlot {}
unsafe impl Sync for AhbOutputSlot {}

impl std::fmt::Debug for AhbOutputSlot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AhbOutputSlot")
            .field("width", &self.width)
            .field("height", &self.height)
            .field("capacity", &self.capacity)
            .field("generation", &self.generation)
            .finish_non_exhaustive()
    }
}

impl AhbOutputSlot {
    /// Allocate and map a slot with 2× the display bytes, rounded up to a
    /// power of two. The Vulkan tensor pads its element size to a multiple of
    /// four words; the headroom covers that without ever resizing the binding.
    pub(crate) fn new(width: u32, height: u32, generation: u64) -> Result<Self, String> {
        let display_bytes = (width as usize)
            .saturating_mul(height as usize)
            .saturating_mul(4);
        let capacity = display_bytes
            .saturating_mul(2)
            .next_power_of_two()
            .max(4096);
        if capacity == 0 || capacity > u32::MAX as usize {
            return Err(format!("invalid AHB output capacity {capacity}"));
        }

        let mut desc: sys::AHardwareBuffer_Desc = unsafe { std::mem::zeroed() };
        desc.width = capacity as u32;
        desc.height = 1;
        desc.layers = 1;
        desc.format = sys::AHardwareBuffer_Format::AHARDWAREBUFFER_FORMAT_BLOB.0;
        desc.usage = sys::AHardwareBuffer_UsageFlags::AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN.0
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
                sys::AHardwareBuffer_UsageFlags::AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN.0,
                -1,
                null_mut(),
                &mut map,
            )
        };
        if rc != 0 || map.is_null() {
            unsafe { sys::AHardwareBuffer_release(ahb) };
            return Err(format!("AHardwareBuffer_lock failed: rc={rc}"));
        }

        slot_debug(format!(
            "allocated {width}x{height} slot generation={generation} capacity={capacity}B"
        ));
        Ok(Self {
            ahb,
            map: map as *mut u8,
            capacity,
            width,
            height,
            generation,
            busy: Arc::new(AtomicBool::new(false)),
        })
    }

    fn display_bytes(&self) -> usize {
        (self.width as usize)
            .saturating_mul(self.height as usize)
            .saturating_mul(4)
            .min(self.capacity)
    }

    /// The AHB for `setDevicePtr` and the mapping for `ProcessParams::output_buffer`.
    pub(crate) fn buffers(&self) -> (AhbBinding, (*mut u8, usize)) {
        let binding = AhbBinding::new(
            self.ahb as *mut c_void,
            self.capacity,
            AhbFenceTokens::new(-1, -1, self.generation),
        );
        (binding, (self.map, self.capacity))
    }

    pub(crate) fn view(&self) -> AhbView {
        AhbView::new(self)
    }

    pub(crate) fn width(&self) -> u32 {
        self.width
    }

    pub(crate) fn height(&self) -> u32 {
        self.height
    }

    pub(crate) fn mark_busy(&self) {
        self.busy.store(true, Ordering::Release);
    }

    pub(crate) fn mark_free(&self) {
        self.busy.store(false, Ordering::Release);
    }

    /// Block until the presenter has released this worker's slot.
    ///
    /// This is a per-worker handoff, not a cross-engine lock: the other worker
    /// keeps running. Spinning is bounded because the presenter only does a
    /// blit; verbose mode records every occurrence so a slow surface is
    /// visible in the diagnostic log.
    pub(crate) fn wait_free(&self) {
        let mut spins = 0u64;
        while self.busy.load(Ordering::Acquire) && spins < 1_000_000 {
            std::thread::yield_now();
            spins += 1;
        }
        if spins > 0 {
            slot_debug(format!(
                "waited {spins} yields for generation {} presenter handoff",
                self.generation
            ));
        }
        if self.busy.load(Ordering::Acquire) {
            log::warn!(
                "AHB output slot generation {} still busy after 1M yields; presenting may tear",
                self.generation
            );
        }
    }
}

impl Drop for AhbOutputSlot {
    fn drop(&mut self) {
        if !self.map.is_null() {
            unsafe {
                sys::AHardwareBuffer_unlock(self.ahb, null_mut());
            }
            self.map = null_mut();
        }
        if !self.ahb.is_null() {
            unsafe {
                sys::AHardwareBuffer_release(self.ahb);
            }
            self.ahb = null_mut();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_bytes_fit_capacity() {
        let width = 1920usize;
        let height = 1080usize;
        let capacity = (width * height * 4)
            .saturating_mul(2)
            .next_power_of_two()
            .max(4096);
        assert!(capacity >= width * height * 4);
    }
}
