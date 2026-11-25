//! softisp_anw: ANativeWindow consumer for softisp IPA on Android

use ndk_sys::{ANativeWindow, ANativeWindow_Buffer};
use cam_ipa::{IpaConfig, IpaContext};
use std::sync::Mutex;

// Thread-safe storage for IPA contexts.
// `*mut IpaContext` is not Send/Sync by default; we wrap it since the Mutex
// serializes all access and the context itself is Send+Sync (IspEngine: Send+Sync).
struct AnwIpACtx(*mut IpaContext);
unsafe impl Send for AnwIpACtx {}
unsafe impl Sync for AnwIpACtx {}

static ANW_CTX: Mutex<Option<AnwIpACtx>> = Mutex::new(None);

#[no_mangle]
pub extern "C" fn softisp_anw_init(window: *mut ANativeWindow) -> i32 {
    if window.is_null() {
        return -1;
    }
    
    let config = IpaConfig::default();
    match IpaContext::new(config) {
        Ok(ctx) => {
            let mut guard = ANW_CTX.lock().unwrap();
            *guard = Some(AnwIpACtx(Box::into_raw(Box::new(ctx))));
            0
        }
        Err(_) => -1,
    }
}

#[no_mangle]
pub extern "C" fn softisp_anw_start() -> i32 {
    let mut guard = ANW_CTX.lock().unwrap();
    match guard.as_mut() {
        Some(AnwIpACtx(ptr)) => {
            let ctx = unsafe { &mut **ptr };
            match ctx.start() {
                Ok(()) => 0,
                Err(_) => -1,
            }
        }
        None => -1,
    }
}

#[no_mangle]
pub extern "C" fn softisp_anw_stop() {
    let mut guard = ANW_CTX.lock().unwrap();
    if let Some(AnwIpACtx(ptr)) = guard.take() {
        unsafe {
            let ctx = Box::from_raw(ptr);
            drop(ctx);
        }
    }
}