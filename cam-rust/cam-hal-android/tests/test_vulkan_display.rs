//! Tests for the Vulkan Android Display module.
//!
//! These tests are gated behind `#[cfg(target_os = "android")]` since
//! the Vulkan loader + ANativeWindow only exist on Android.

#![cfg(target_os = "android")]

use cam_hal_android::vulkan_display::VulkanDisplay;

#[test]
fn test_vulkan_display_struct_accessible() {
    // The struct is public; verify we can reference it (can't construct
    // without a real ANativeWindow on device).
    fn _assert_constructible(
    ) -> Result<VulkanDisplay, String> {
        unreachable!()
    }
}
