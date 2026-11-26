//! Path B presentation: `AHardwareBuffer` + `ASurfaceControl` / `ASurfaceTransaction`.
//!
//! This is the **no-Vulkan-context, no-swapchain** presentation path. Instead of
//! `VkSwapchainKHR` + `vkQueuePresentKHR`, we take the worker's already-GPU-written
//! `AHardwareBuffer` mapping (`AhbOutputSlot` / `AhbView`), create an
//! `ASurfaceControl` from the app's `ANativeWindow`, build an
//! `ASurfaceTransaction` attaching that AHB to the control, and call
//! `ASurfaceTransaction_apply()` — SurfaceFlinger consumes the buffer on the next
//! VSYNC, no CPU blit, no intermediate `ANativeWindow_lock` copy.
//!
//! This mirrors ncnn's output recommendation: a separate exportable AHB buffer
//! + `vkGetMemoryAndroidHardwareBufferANDROID` / `ASurfaceTransaction` handoff.
//!
//! The MNN Vulkan backend already imported the AHB as `VkDeviceMemory` and wrote
//! the display shader output straight into it; `mnn_run_ahb_output()` already
//! waited for GPU completion (`output->wait(MAP_TENSOR_READ, true)`), so the
//! mapping contains valid pixels by the time we reach here.
//!
//! `ASurfaceControl` / `ASurfaceTransaction` are **not** in `ndk-sys` 0.6, so we
//! dlsym them from `libnativewindow.so` at runtime (same pattern as
//! `vulkan_display.rs` dlsyms `libvulkan.so`).

use std::ffi::CString;
use std::os::raw::c_void;

// ── Function pointer types ──────────────────────────────────────────────────

type PfnAsurfaceControlCreateFromWindow =
    extern "C" fn(*mut c_void /*ANativeWindow*/) -> *mut ASurfaceControl;
type PfnAsurfaceControlRelease = extern "C" fn(*mut ASurfaceControl);
type PfnAsurfaceTransactionCreate = extern "C" fn() -> *mut ASurfaceTransaction;
type PfnAsurfaceTransactionRelease = extern "C" fn(*mut ASurfaceTransaction);
type PfnAsurfaceTransactionSetBuffer = extern "C" fn(
    *mut ASurfaceTransaction,
    *mut ASurfaceControl,
    *mut c_void, /*AHardwareBuffer*/
);
type PfnAsurfaceTransactionApply = extern "C" fn(*mut ASurfaceTransaction);

/// Opaque handle types for `libnativewindow.so`.
pub type ASurfaceControl = c_void;
pub type ASurfaceTransaction = c_void;

/// Loader for `libnativewindow.so` surface-control entry points.
///
/// All symbols are resolved via `dlsym` at runtime so the crate links cleanly
/// on any NDK version that ships `libnativewindow.so`.
pub struct SurfaceControlLoader {
    pub create_from_window: PfnAsurfaceControlCreateFromWindow,
    pub release: PfnAsurfaceControlRelease,
    pub txn_create: PfnAsurfaceTransactionCreate,
    pub txn_release: PfnAsurfaceTransactionRelease,
    pub txn_set_buffer: PfnAsurfaceTransactionSetBuffer,
    pub txn_apply: PfnAsurfaceTransactionApply,
    pub handle: *mut c_void,
}

unsafe impl Send for SurfaceControlLoader {}
unsafe impl Sync for SurfaceControlLoader {}

impl SurfaceControlLoader {
    /// Open `libnativewindow.so` and resolve all entry points.
    ///
    /// # Safety
    ///
    /// Must be called on Android where `libnativewindow.so` is present.
    pub unsafe fn open() -> Result<Self, String> {
        let name = CString::new("libnativewindow.so").unwrap();
        let handle = libc::dlopen(name.as_ptr(), libc::RTLD_NOW);
        if handle.is_null() {
            // Fall back to libandroid.so on older NDKs.
            let name = CString::new("libandroid.so").unwrap();
            let handle = libc::dlopen(name.as_ptr(), libc::RTLD_NOW);
            if handle.is_null() {
                return Err("dlopen libnativewindow.so/libandroid.so failed".to_string());
            }
        }
        let sym = |s: &str| -> *const c_void {
            let c = CString::new(s).unwrap();
            libc::dlsym(handle, c.as_ptr())
        };
        macro_rules! sym {
            ($name:expr) => {{
                let raw = sym($name);
                if raw.is_null() {
                    libc::dlclose(handle);
                    return Err(format!("missing symbol {}", $name).to_string());
                }
                raw
            }};
        }
        Ok(Self {
            create_from_window: unsafe {
                std::mem::transmute::<*const c_void, PfnAsurfaceControlCreateFromWindow>(sym!(
                    "ASurfaceControl_createFromWindow"
                ))
            },
            release: unsafe {
                std::mem::transmute::<*const c_void, PfnAsurfaceControlRelease>(sym!(
                    "ASurfaceControl_release"
                ))
            },
            txn_create: unsafe {
                std::mem::transmute::<*const c_void, PfnAsurfaceTransactionCreate>(sym!(
                    "ASurfaceTransaction_create"
                ))
            },
            txn_release: unsafe {
                std::mem::transmute::<*const c_void, PfnAsurfaceTransactionRelease>(sym!(
                    "ASurfaceTransaction_release"
                ))
            },
            txn_set_buffer: unsafe {
                std::mem::transmute::<*const c_void, PfnAsurfaceTransactionSetBuffer>(sym!(
                    "ASurfaceTransaction_setBuffer"
                ))
            },
            txn_apply: unsafe {
                std::mem::transmute::<*const c_void, PfnAsurfaceTransactionApply>(sym!(
                    "ASurfaceTransaction_apply"
                ))
            },
            handle,
        })
    }
}

impl Drop for SurfaceControlLoader {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { libc::dlclose(self.handle) };
        }
    }
}

/// Zero-copy display sink: presents an `AHardwareBuffer` directly to
/// SurfaceFlinger via `ASurfaceTransaction`, with no CPU blit.
pub struct AhbDisplay {
    loader: SurfaceControlLoader,
    control: *mut ASurfaceControl,
    window_width: u32,
    window_height: u32,
}

unsafe impl Send for AhbDisplay {}
unsafe impl Sync for AhbDisplay {}

impl AhbDisplay {
    /// Create an `ASurfaceControl` bound to `window`.
    ///
    /// `window` is the `ANativeWindow*` from `android_app.window` (via
    /// `NativeWindow::as_ptr()` or equivalent).
    ///
    /// # Safety
    ///
    /// `window` must be a valid `ANativeWindow*` pointer.
    pub unsafe fn new(window: *mut c_void) -> Result<Self, String> {
        let loader = SurfaceControlLoader::open()?;
        let control = (loader.create_from_window)(window);
        if control.is_null() {
            return Err("ASurfaceControl_createFromWindow returned null".into());
        }
        Ok(Self {
            loader,
            control,
            window_width: 0,
            window_height: 0,
        })
    }

    /// Present a raw `AHardwareBuffer*` directly to SurfaceFlinger.
    ///
    /// `ahb` is the producer-owned `AHardwareBuffer*` (e.g. from
    /// `AhbOutputSlot` via `AhbBinding.ptr`). `bytes` is the CPU-mapped slice
    /// of the same buffer — passed for the caller's bookkeeping but SurfaceFlinger
    /// reads from the AHB handle itself, not from `bytes`.
    ///
    /// # Safety
    ///
    /// `ahb` must be a valid, live `AHardwareBuffer*` that remains valid until
    /// SurfaceFlinger releases it. `bytes` must describe the same buffer's
    /// mapped range.
    pub unsafe fn present_ahb_raw(
        &mut self,
        ahb: *mut c_void,
        _bytes: &[u8],
        width: u32,
        height: u32,
    ) -> Result<usize, String> {
        if ahb.is_null() {
            return Err("null AHardwareBuffer".into());
        }

        let txn = (self.loader.txn_create)();
        if txn.is_null() {
            return Err("ASurfaceTransaction_create returned null".into());
        }

        (self.loader.txn_set_buffer)(txn, self.control, ahb);

        // Optionally set a desired present time (VSYNC-aligned). We skip it
        // here for simplicity; SurfaceFlinger presents on the next VSYNC.

        (self.loader.txn_apply)(txn);
        (self.loader.txn_release)(txn);

        self.window_width = width;
        self.window_height = height;
        Ok((width as usize) * (height as usize) * 4)
    }
}

impl Drop for AhbDisplay {
    fn drop(&mut self) {
        if !self.control.is_null() {
            (self.loader.release)(self.control);
        }
    }
}
