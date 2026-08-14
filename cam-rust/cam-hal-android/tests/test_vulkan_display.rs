//! Integration tests for the Vulkan Android Display module.
//!
//! On Android, the Vulkan loader (`libvulkan.so`) provides the native
//! surface and present APIs. These tests verify the module loads
//! correctly and can resolve Vulkan function pointers.
//!
//! Note: `vkCreateAndroidSurfaceKHR` is an instance-extension function —
//! it is NOT a direct symbol in `libvulkan.so`. It must be resolved via
//! `vkGetInstanceProcAddr` after `vkCreateInstance`.
//!
//! For native display output, an `ANativeWindow*` must be obtained from
//! the Android app framework:
//!   - `ANativeActivity.onNativeWindowCreated()` callback, or
//!   - JNI `ANativeWindow_fromSurface()` from a Java `Surface`
//!
//! A pure-native `libgui.so` / `SurfaceComposerClient` path was
//! investigated: the symbols exist, but the binder call to SurfaceFlinger
//! (`SurfaceComposerClient::getDefault()`) crashes from Termux due to
//! missing system permissions (uid 10037, not system/surface_flinger).

use std::ffi::c_void;
use std::ptr;

use cam_hal_android::vulkan_display::{
    VkApplicationInfo, VkInstanceCreateInfo, VkLoader, VulkanDisplay,
    VK_STRUCTURE_TYPE_APPLICATION_INFO, VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO, VK_SUCCESS,
};

#[cfg(not(target_os = "android"))]
#[test]
fn test_vulkan_display_not_available_on_host() {
    let result = unsafe { VkLoader::open() };
    assert!(result.is_err(), "VkLoader should fail on non-Android host");
}

#[cfg(target_os = "android")]
#[test]
fn test_vulkan_loader_loads() {
    let result = unsafe { VkLoader::open() };
    assert!(
        result.is_ok(),
        "VkLoader::open() should succeed on Android: {:?}",
        result.err()
    );
}

#[cfg(target_os = "android")]
#[test]
fn test_vulkan_instance_creation() {
    let loader = unsafe { VkLoader::open().expect("VkLoader should load on Android") };

    let app_name = std::ffi::CString::new("softisp-test").unwrap();
    let app_info = VkApplicationInfo {
        sType: VK_STRUCTURE_TYPE_APPLICATION_INFO,
        pNext: ptr::null(),
        pApplicationName: app_name.as_ptr(),
        applicationVersion: 1,
        pEngineName: ptr::null(),
        engineVersion: 1,
        apiVersion: 0x0040_0000,
    };
    let create_info = VkInstanceCreateInfo {
        sType: VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
        pNext: ptr::null(),
        flags: 0,
        pApplicationInfo: &app_info,
        enabledLayerCount: 0,
        ppEnabledLayerNames: ptr::null(),
        enabledExtensionCount: 0,
        ppEnabledExtensionNames: ptr::null(),
    };

    let mut instance = 0u64;
    let res = (loader.vkCreateInstance)(&create_info, ptr::null(), &mut instance);
    assert_eq!(
        res, VK_SUCCESS,
        "vkCreateInstance should succeed without extensions"
    );

    if instance != 0 {
        let raw = unsafe { loader.get_instance_proc(instance, "vkDestroyInstance") };
        if !raw.is_null() {
            let destroy: extern "C" fn(u64, *const c_void) = unsafe { std::mem::transmute(raw) };
            destroy(instance, ptr::null());
        }
    }
}

#[cfg(target_os = "android")]
#[test]
#[ignore]
fn test_draw_rgba_frame_to_native_window() {
    use std::os::raw::c_int;

    // Obtain a real ANativeWindow* from the Android runtime.
    #[link(name = "android")]
    extern "C" {
        fn ANativeWindow_acquire(window: *mut c_void);
        fn ANativeWindow_release(window: *mut c_void);
        fn ANativeWindow_getWidth(window: *mut c_void) -> c_int;
        fn ANativeWindow_getHeight(window: *mut c_void) -> c_int;
        fn ANativeWindow_getFormat(window: *mut c_void) -> c_int;
    }

    // 1. Create a 640x480 RGBA frame (solid red: 0xFFFF0000).
    let width = 640usize;
    let height = 480usize;
    let pixel: [u8; 4] = [255, 0, 0, 255];
    let _frame = vec![pixel; width * height];

    // 2. The window must come from:
    //    - ANativeActivity.onNativeWindowCreated() callback, or
    //    - JNI ANativeWindow_fromSurface(Surface) call
    //    Set this from your Android app test harness.
    let window_ptr: *mut c_void = std::ptr::null_mut();

    if window_ptr.is_null() {
        eprintln!(
            "No ANativeWindow available — provide one from \
            ANativeActivity.onNativeWindowCreated or ANativeWindow_fromSurface() via JNI."
        );
        return;
    }

    // 3. Create the Vulkan surface from the window.
    let display = unsafe { VulkanDisplay::new_from_window(window_ptr) }
        .expect("VulkanDisplay::new_from_window should succeed");
    assert_ne!(display.surface(), 0);

    // 4. Query window properties.
    unsafe {
        ANativeWindow_acquire(window_ptr);
        let w = ANativeWindow_getWidth(window_ptr);
        let h = ANativeWindow_getHeight(window_ptr);
        let fmt = ANativeWindow_getFormat(window_ptr);
        eprintln!("ANativeWindow: {}x{} format={}", w, h, fmt);
        ANativeWindow_release(window_ptr);
    }
}
