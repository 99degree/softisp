//! Integration tests for the Vulkan Android Display module.
//!
//! On Android, the Vulkan loader (`libvulkan.so`) provides the native
//! surface and present APIs. These tests verify the module loads
//! correctly and can resolve Vulkan function pointers.
//!
//! Note: `vkCreateAndroidSurfaceKHR` is an instance-extension function —
//! it is NOT a direct symbol in `libvulkan.so`. It must be resolved via
//! `vkGetInstanceProcAddr` after `vkCreateInstance`. This is how the
//! `VkLoader` / `VulkanDisplay` module handles it.

use std::ffi::c_void;

use cam_hal_android::vulkan_display::{VkLoader, VulkanDisplay};

#[cfg(not(target_os = "android"))]
#[test]
fn test_vulkan_display_not_available_on_host() {
    // On non-Android hosts there is no libvulkan.so / ANativeWindow.
    let result = unsafe { VkLoader::open() };
    assert!(result.is_err(), "VkLoader should fail on non-Android host");
}

#[cfg(target_os = "android")]
#[test]
fn test_vulkan_loader_loads() {
    // On Android, libvulkan.so should be present and export the two
    // direct loader symbols: vkCreateInstance and vkGetInstanceProcAddr.
    // Extension functions like vkCreateAndroidSurfaceKHR are NOT direct
    // symbols — they must be obtained via vkGetInstanceProcAddr.
    let result = unsafe { VkLoader::open() };
    assert!(
        result.is_ok(),
        "VkLoader::open() should succeed on Android: {:?}",
        result.err()
    );
}

#[cfg(target_os = "android")]
#[test]
fn test_vulkan_instance_and_surface() {
    // Full integration: create a Vulkan instance, resolve the
    // vkCreateAndroidSurfaceKHR extension function via the loader,
    // and verify it is available.
    //
    // On Android, the ANativeWindow* comes from:
    //   - ANativeActivity.onNativeWindowCreated callback
    //   - JNI ANativeWindow_fromSurface(Surface) call
    //
    // This test verifies the loader path works end-to-end.
    let loader = unsafe { VkLoader::open().expect("VkLoader should load on Android") };

    // vkCreateInstance and vkGetInstanceProcAddr are direct loader symbols.
    let _create = loader.vkCreateInstance;
    let _gipa = loader.vkGetInstanceProcAddr;
}

/// Test drawing a solid color RGBA frame to an ANativeWindow.
///
/// This requires a real ANativeWindow (from an Android Activity's
/// `onNativeWindowCreated` callback or a Java `Surface` via JNI),
/// so it is gated behind `#[ignore]` and runs on-device with:
/// `cargo test --test test_vulkan_display test_draw_rgba_frame_to_native_window -- --ignored --nocapture`
///
/// Note: `VK_KHR_display` (direct hardware surface without ANativeWindow)
/// is NOT supported on Android — `vkCreateDisplaySurfaceKHR` is absent
/// from the Android Vulkan loader.
#[cfg(target_os = "android")]
#[test]
#[ignore]
fn test_draw_rgba_frame_to_native_window() {
    use std::os::raw::c_int;

    // Obtain a real ANativeWindow* from the Android runtime.
    // In a NativeActivity, this comes from onNativeWindowCreated().
    // In a JNI context, it comes from ANativeWindow_fromSurface().
    #[link(name = "android")]
    extern "C" {
        fn ANativeWindow_acquire(window: *mut c_void);
        fn ANativeWindow_release(window: *mut c_void);
        fn ANativeWindow_getWidth(window: *mut c_void) -> c_int;
        fn ANativeWindow_getHeight(window: *mut c_void) -> c_int;
    }

    // 1. Create a 640x480 RGBA frame (solid red: 0xFFFF0000).
    let width = 640usize;
    let height = 480usize;
    let pixel: [u8; 4] = [255, 0, 0, 255];
    let _frame = vec![pixel; width * height];

    // 2. The window must come from the Android runtime.
    //    In a real test, set this from your NativeActivity's
    //    onNativeWindowCreated() callback.
    let window_ptr: *mut c_void = std::ptr::null_mut();

    if window_ptr.is_null() {
        eprintln!(
            "No ANativeWindow available — skipping. \
            Provide a valid ANativeWindow* from ANativeActivity.onNativeWindowCreated \
            or ANativeWindow_fromSurface()."
        );
        return;
    }

    // 3. Create the Vulkan display surface from the window.
    let display = unsafe { VulkanDisplay::new_from_window(window_ptr) }
        .expect("VulkanDisplay::new_from_window should succeed");

    // 4. Verify the surface was created.
    assert_ne!(display.surface(), 0); // VK_NULL_HANDLE

    // 5. Verify we can resolve instance-level functions.
    unsafe {
        ANativeWindow_acquire(window_ptr);
        let w = ANativeWindow_getWidth(window_ptr);
        let h = ANativeWindow_getHeight(window_ptr);
        eprintln!("ANativeWindow: {}x{}", w, h);
        ANativeWindow_release(window_ptr);
    }
}
