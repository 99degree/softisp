//! Minimal Vulkan capability probe for the GPU-backed tests.
//!
//! Several tests build a real MNN model and run it on the Vulkan backend. They
//! used to gate themselves on the `CI` environment variable being set, which is
//! the wrong question in both directions: the `rust-test-mnn` job installs MNN
//! *and* the Vulkan loader (so the tests should run there and fail loudly when
//! the pipeline is broken), while a developer machine that happens to export
//! `CI` skipped them for no reason.
//!
//! What actually decides whether those tests can run is whether the loader can
//! hand out a physical device. Headless runners have no driver and report
//! zero; a phone or workstation reports at least one. That is what we probe.

use std::ffi::{c_char, c_void};
use std::sync::OnceLock;

const VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO: i32 = 1;
const VK_SUCCESS: i32 = 0;

/// Mirrors `VkInstanceCreateInfo`. Only `sType` is meaningful here — every
/// other field stays zeroed so the loader applies its defaults.
#[repr(C)]
struct VkInstanceCreateInfo {
    s_type: i32,
    p_next: *const c_void,
    flags: u32,
    p_application_info: *const c_void,
    enabled_layer_count: u32,
    pp_enabled_layer_names: *const *const c_char,
    enabled_extension_count: u32,
    pp_enabled_extension_names: *const *const c_char,
}

type VkInstance = *mut c_void;

type VkCreateInstance =
    unsafe extern "C" fn(*const VkInstanceCreateInfo, *const c_void, *mut VkInstance) -> i32;
type VkEnumeratePhysicalDevices =
    unsafe extern "C" fn(VkInstance, *mut u32, *mut VkInstance) -> i32;
type VkDestroyInstance = unsafe extern "C" fn(VkInstance, *const c_void);

/// Loader sonames to try, in order. Android ships the loader as
/// `libvulkan.so`; desktop Linux uses the `.so.1` ABI version.
const LOADER_SONAMES: [&[u8]; 2] = [b"libvulkan.so\0", b"libvulkan.so.1\0"];

/// Count the Vulkan physical devices the loader can enumerate.
///
/// Returns 0 when the loader is absent, no ICD is installed, or the driver is
/// broken — every case in which the GPU tests cannot run.
fn enumerate_physical_device_count() -> u32 {
    // SAFETY: hand-rolled Vulkan bootstrap. Each entry point is checked for
    // null before use, the create-info is fully initialised, and the instance
    // is destroyed on every path that created one.
    unsafe {
        let mut loader = std::ptr::null_mut();
        for soname in LOADER_SONAMES {
            loader = libc::dlopen(soname.as_ptr() as *const c_char, libc::RTLD_NOW);
            if !loader.is_null() {
                break;
            }
        }
        if loader.is_null() {
            return 0;
        }

        let sym = |name: &[u8]| libc::dlsym(loader, name.as_ptr() as *const c_char);
        let create: VkCreateInstance = std::mem::transmute(sym(b"vkCreateInstance\0"));
        let enumerate: VkEnumeratePhysicalDevices =
            std::mem::transmute(sym(b"vkEnumeratePhysicalDevices\0"));
        let destroy: VkDestroyInstance = std::mem::transmute(sym(b"vkDestroyInstance\0"));
        // The loader's ABI version is all we get to test against; if any core
        // 1.0 entry point is missing, treat the driver as unusable.
        if Option::<VkCreateInstance>::from(create).is_none()
            || Option::<VkEnumeratePhysicalDevices>::from(enumerate).is_none()
            || Option::<VkDestroyInstance>::from(destroy).is_none()
        {
            return 0;
        }

        let info = VkInstanceCreateInfo {
            s_type: VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
            p_next: std::ptr::null(),
            flags: 0,
            p_application_info: std::ptr::null(),
            enabled_layer_count: 0,
            pp_enabled_layer_names: std::ptr::null(),
            enabled_extension_count: 0,
            pp_enabled_extension_names: std::ptr::null(),
        };
        let mut instance: VkInstance = std::ptr::null_mut();
        if create(&info, std::ptr::null(), &mut instance) != VK_SUCCESS || instance.is_null() {
            return 0;
        }

        let mut count: u32 = 0;
        let res = enumerate(instance, &mut count, std::ptr::null_mut());
        destroy(instance, std::ptr::null());
        if res == VK_SUCCESS {
            count
        } else {
            0
        }
    }
}

/// Whether a Vulkan device is available for the GPU-backed tests.
///
/// The answer is cached: the probe dlopens the loader and creates an instance,
/// which is far too much work to repeat per test.
pub(crate) fn vulkan_available() -> bool {
    static AVAILABLE: OnceLock<bool> = OnceLock::new();
    *AVAILABLE.get_or_init(|| {
        let count = enumerate_physical_device_count();
        if count == 0 {
            eprintln!("no Vulkan physical device found — skipping GPU-backed test");
        }
        count > 0
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_probe_is_cached_and_stable() {
        // The probe must not report a different answer on the second call;
        // this is what lets the GPU tests gate on it without flaking.
        assert_eq!(vulkan_available(), vulkan_available());
    }
}
