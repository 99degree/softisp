//! softisp_jni: JNI bridge for softisp IPA on Android
//!
//! Builds on Android target only (aarch64-linux-android).
//! On host it compiles as a stub crate; JNI functions are no-ops.

use jni::JNIEnv;
use jni::objects::{JClass, JString};
use cam_ipa::{IpaConfig, IpaContext};
use std::sync::Mutex;

// Thread-safe storage for IPA contexts.
// `*mut IpaContext` is not Send/Sync by default; we wrap it since the Mutex
// serializes all access and the context itself is Send+Sync (IspEngine: Send+Sync).
struct JniIpACtx(*mut IpaContext);
unsafe impl Send for JniIpACtx {}
unsafe impl Sync for JniIpACtx {}

static IPA_CTX: Mutex<Option<JniIpACtx>> = Mutex::new(None);

#[no_mangle]
pub extern "C" fn Java_com_softisp_ipa_IpaSession_nativeInit(
    mut env: JNIEnv,
    _class: JClass,
) -> jni::sys::jboolean {
    let config = IpaConfig::default();
    match IpaContext::new(config) {
        Ok(ctx) => {
            let mut guard = IPA_CTX.lock().unwrap();
            *guard = Some(JniIpACtx(Box::into_raw(Box::new(ctx))));
            1
        }
        Err(_) => 0,
    }
}

#[no_mangle]
pub extern "C" fn Java_com_softisp_ipa_IpaSession_nativeStart(
    mut env: JNIEnv,
    _class: JClass,
) -> jni::sys::jint {
    let mut guard = IPA_CTX.lock().unwrap();
    match guard.as_mut() {
        Some(JniIpACtx(ptr)) => {
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
pub extern "C" fn Java_com_softisp_ipa_IpaSession_nativeStop(
    mut env: JNIEnv,
    _class: JClass,
) {
    let mut guard = IPA_CTX.lock().unwrap();
    if let Some(JniIpACtx(ptr)) = guard.take() {
        unsafe {
            let ctx = Box::from_raw(ptr);
            drop(ctx);
        }
    }
}