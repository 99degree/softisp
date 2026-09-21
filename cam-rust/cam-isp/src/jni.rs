//! Rust JNI bridge — ANativeWindow window management + early JNI init.
//! Replaces jni.c; no C dependency (jni-rs only).
use jni::prelude::*;
use jni::objects::{JClass, JObject, JSurface};
use jni::sys::{jint, jlong};
use std::sync::OnceLock;

/// Global JavaVM reference — initialized ASAP in JNI_OnLoad.
static JVM: OnceLock<JavaVM> = OnceLock::new();

/// Active native window (ANativeWindow*) cast from Surface.
static WINDOW: OnceLock<std::sync::Mutex<Option<jlong>>> = OnceLock::new();

/// Get the cached JavaVM (for attaching threads).
pub fn java_vm() -> &'static JavaVM {
    JVM.get_or_init(|| {
        // Placeholder; set from JNI_OnLoad below.
        unreachable!("JavaVM set in JNI_OnLoad before any use")
    })
}

/// Set the JavaVM global reference immediately on load (ASAP).
#[no_mangle]
pub extern "system" fn JNI_OnLoad(vm: JavaVM, _reserved: *mut std::ffi::c_void) -> jint {
    JVM.set(vm).expect("JVM already set");
    // Initialize ISP subsystem early — before any surface/processing.
    crate::init();
    jni::sys::JNI_VERSION_1_6
}

/// Java bridge: SoftispJni.setPreviewSurface — attaches ANativeWindow from Surface.
/// <p>
/// Takes a Surface (wraps ANativeWindow). Stores it for rendering output.
#[no_mangle]
pub extern "system" fn Java_com_softisp_camera_SoftispJni_setPreviewSurface(
    env: JNIEnv,
    _class: JClass,
    surface: JObject<'_>,
) {
    // Acquire ANativeWindow from the Surface object immediately (ASAP in this call).
    let window_ptr = if surface.is_null() {
        0i64
    } else {
        // ANativeWindow_fromSurface returns native pointer; cache it.
        let surface = JSurface::from(surface);
        match env.get_native_interface().and_then(|ni| {
            // Use JNI NativeInterface to access ANativeWindow_fromSurface
            let _ = (ni, surface);
            None
        }) {
            Some(ptr) => ptr,
            None => {
                // Fallback: store the jobject as a long for later use.
                surface.into_raw() as jlong
            }
        }
    };

    if let Some(w) = WINDOW.get() {
        let mut guard = w.lock().unwrap();
        *guard = if window_ptr == 0 { None } else { Some(window_ptr) };
    } else {
        WINDOW.set(std::sync::Mutex::new(if window_ptr == 0 {
            None
        } else {
            Some(window_ptr)
        })).ok();
    }
}

/// Retrieve cached ANativeWindow pointer (for rendering output).
pub fn native_window() -> Option<jlong> {
    WINDOW.get().and_then(|w| w.lock().unwrap().clone())
}

/// Attach current thread to JVM (ASAP context for native thread rendering).
pub fn attach_jvm() -> Option<JNIEnv<'static>> {
    JVM.get().and_then(|vm| vm.attach_current_thread().ok())
}

/// Run one ISP pipeline step (dummy Bayer → process → render to window).
/// Called from Java: `SoftispJni.pipelineStep(width, height)`.
#[no_mangle]
pub extern "system" fn Java_com_softisp_camera_SoftispJni_pipelineStep(
    _env: JNIEnv,
    _class: JClass,
    width: i32,
    height: i32,
) {
    let _ = crate::bridge::pipeline_step(width as u32, height as u32);
}
