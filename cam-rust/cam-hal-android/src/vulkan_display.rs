//! Minimal Vulkan-on-Android presentor.
//!
//! MNN's Vulkan backend is **compute-only** — it never calls
//! `vkQueuePresentKHR`, so it cannot present to a screen. This module fills
//! that gap: it dlsyms the Android Vulkan loader (`libvulkan.so`) and sets up
//! an `ANativeWindow*` → `VkSurfaceKHR` → swapchain pipeline so the
//! ISP-processed RGBA output can be drawn directly to the display.
//!
//! Design notes:
//! - No `ash`/bindgen dependency: we declare the handful of Vulkan structs we
//!   need and resolve entry points from `libvulkan.so` at runtime (same dlsym
//!   style as `adapter.rs` / `gralloc.rs`).
//! - `vkCreateAndroidSurfaceKHR` is a loader symbol (not provided by MNN),
//!   so we load the loader ourselves.
//! - Images are uploaded via a `HOST_VISIBLE` staging buffer →
//!   `DEVICE_LOCAL` swapchain blit. This is the simplest correct path and
//!   avoids needing a shared Vulkan device with MNN.
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(dead_code)]

use std::ffi::{c_char, c_void};
use std::ptr;

// ── Minimal Vulkan FFI types ───────────────────────────────────────────────

pub type VkInstance = u64;
pub type VkDevice = u64;
pub type VkPhysicalDevice = u64;
pub type VkSurfaceKHR = u64;
pub type VkSwapchainKHR = u64;
pub type VkImage = u64;
pub type VkImageView = u64;
pub type VkDeviceMemory = u64;
pub type VkBuffer = u64;
pub type VkCommandPool = u64;
pub type VkCommandBuffer = u64;
pub type VkQueue = u64;
pub type VkRenderPass = u64;
pub type VkFramebuffer = u64;
pub type VkSemaphore = u64;
pub type VkFence = u64;
pub type VkShaderModule = u64;
pub type VkPipeline = u64;
pub type VkPipelineLayout = u64;
pub type VkDescriptorSetLayout = u64;
pub type VkDescriptorPool = u64;
pub type VkDescriptorSet = u64;
pub type VkResult = i32;
pub type VkFlags = u32;
pub type VkBool32 = u32;
pub type VkDeviceSize = u64;

pub const VK_SUCCESS: VkResult = 0;
pub const VK_NULL_HANDLE: u64 = 0;
pub type VkStructureType = i32;
pub const VK_STRUCTURE_TYPE_ANDROID_SURFACE_CREATE_INFO_KHR: VkStructureType = 1000008000;

pub const VK_IMAGE_LAYOUT_UNDEFINED: i32 = 0;
pub const VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL: i32 = 3;
pub const VK_IMAGE_LAYOUT_PRESENT_SRC_KHR: i32 = 1000001002;
pub const VK_FORMAT_R8G8B8A8_UNORM: i32 = 37;

pub const VK_IMAGE_USAGE_TRANSFER_DST_BIT: VkFlags = 0x0000_0001;
pub const VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT: VkFlags = 0x0000_0010;
pub const VK_BUFFER_USAGE_TRANSFER_SRC_BIT: VkFlags = 0x0000_0001;

pub const VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT: VkFlags = 0x0000_0002;
pub const VK_MEMORY_PROPERTY_HOST_COHERENT_BIT: VkFlags = 0x0000_0004;
pub const VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT: VkFlags = 0x0000_0001;

pub const VK_SHARING_MODE_EXCLUSIVE: i32 = 0;
pub const VK_COMMAND_BUFFER_LEVEL_PRIMARY: i32 = 0;
pub const VK_PIPELINE_STAGE_TRANSFER_BIT: VkFlags = 0x0000_0400;
pub const VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT: VkFlags = 0x0000_0200;
pub const VK_ACCESS_TRANSFER_WRITE_BIT: VkFlags = 0x0000_0400;
pub const VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT: VkFlags = 0x0000_1000;

pub const VK_IMAGE_ASPECT_COLOR_BIT: VkFlags = 0x0000_0001;
pub const VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT: VkFlags = 0x0000_0001;
pub const VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT: VkFlags = 0x0000_0080;
pub const VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT: VkFlags = 0x0000_0001;

#[repr(C)]
pub struct VkAndroidSurfaceCreateInfoKHR {
    pub sType: VkStructureType,
    pub pNext: *const c_void,
    pub flags: VkFlags,
    pub window: *mut c_void, // ANativeWindow*
}

#[repr(C)]
pub struct VkApplicationInfo {
    pub sType: VkStructureType,
    pub pNext: *const c_void,
    pub pApplicationName: *const c_char,
    pub applicationVersion: u32,
    pub pEngineName: *const c_char,
    pub engineVersion: u32,
    pub apiVersion: u32,
}

#[repr(C)]
pub struct VkInstanceCreateInfo {
    pub sType: VkStructureType,
    pub pNext: *const c_void,
    pub flags: VkFlags,
    pub pApplicationInfo: *const VkApplicationInfo,
    pub enabledLayerCount: u32,
    pub ppEnabledLayerNames: *const *const c_char,
    pub enabledExtensionCount: u32,
    pub ppEnabledExtensionNames: *const *const c_char,
}

// ── Loader entry points (dlsym from libvulkan.so) ──────────────────────────

type PfnVkCreateInstance =
    extern "C" fn(*const VkInstanceCreateInfo, *const c_void, *mut VkInstance) -> VkResult;
type PfnVkGetInstanceProcAddr = extern "C" fn(VkInstance, *const c_char) -> *const c_void;
type PfnVkCreateAndroidSurfaceKHR = extern "C" fn(
    VkInstance,
    *const VkAndroidSurfaceCreateInfoKHR,
    *const c_void,
    *mut VkSurfaceKHR,
) -> VkResult;

#[derive(Clone, Copy)]
pub struct VkLoader {
    pub vkCreateInstance: PfnVkCreateInstance,
    pub vkGetInstanceProcAddr: PfnVkGetInstanceProcAddr,
    pub vkCreateAndroidSurfaceKHR: PfnVkCreateAndroidSurfaceKHR,
    pub handle: *mut c_void,
}

unsafe impl Send for VkLoader {}
unsafe impl Sync for VkLoader {}

impl VkLoader {
    /// Load the Vulkan loader from `libvulkan.so`.
    ///
    /// # Safety
    /// Must be called on Android where `libvulkan.so` is present.
    pub unsafe fn open() -> Result<Self, String> {
        let name = std::ffi::CString::new("libvulkan.so").unwrap();
        let handle = libc::dlopen(name.as_ptr(), libc::RTLD_NOW);
        if handle.is_null() {
            return Err("dlopen libvulkan.so failed".to_string());
        }
        let sym = |s: &str| -> *const c_void {
            let c = std::ffi::CString::new(s).unwrap();
            libc::dlsym(handle, c.as_ptr())
        };
        let raw_create = sym("vkCreateInstance");
        let raw_gipa = sym("vkGetInstanceProcAddr");
        let raw_surface = sym("vkCreateAndroidSurfaceKHR");
        if raw_create.is_null() || raw_gipa.is_null() || raw_surface.is_null() {
            libc::dlclose(handle);
            return Err("missing Vulkan loader symbols".to_string());
        }
        let vkCreateInstance: PfnVkCreateInstance = std::mem::transmute(raw_create);
        let vkGetInstanceProcAddr: PfnVkGetInstanceProcAddr = std::mem::transmute(raw_gipa);
        let vkCreateAndroidSurfaceKHR: PfnVkCreateAndroidSurfaceKHR =
            std::mem::transmute(raw_surface);
        Ok(Self {
            vkCreateInstance,
            vkGetInstanceProcAddr,
            vkCreateAndroidSurfaceKHR,
            handle,
        })
    }
}

/// A Vulkan instance + surface bound to an Android native window.
pub struct VulkanDisplay {
    loader: VkLoader,
    instance: VkInstance,
    surface: VkSurfaceKHR,
    _window: *mut c_void,
}

unsafe impl Send for VulkanDisplay {}

impl VulkanDisplay {
    /// Create a Vulkan instance and an Android surface from `window`.
    ///
    /// # Safety
    /// `window` must be a valid `ANativeWindow*` (from `android_app.window`).
    pub unsafe fn new_from_window(window: *mut c_void) -> Result<Self, String> {
        let loader = VkLoader::open()?;

        let ext0 = std::ffi::CString::new("VK_KHR_surface").unwrap();
        let ext1 = std::ffi::CString::new("VK_KHR_android_surface").unwrap();
        let exts = [ext0.as_ptr(), ext1.as_ptr()];

        let app_name = std::ffi::CString::new("softisp-display").unwrap();
        let app_info = VkApplicationInfo {
            sType: 0,
            pNext: ptr::null(),
            pApplicationName: app_name.as_ptr(),
            applicationVersion: 1,
            pEngineName: ptr::null(),
            engineVersion: 1,
            apiVersion: 0x0040_0000,
        };
        let create_info = VkInstanceCreateInfo {
            sType: 1,
            pNext: ptr::null(),
            flags: 0,
            pApplicationInfo: &app_info,
            enabledLayerCount: 0,
            ppEnabledLayerNames: ptr::null(),
            enabledExtensionCount: 2,
            ppEnabledExtensionNames: exts.as_ptr(),
        };
        let mut instance = VK_NULL_HANDLE;
        let res = (loader.vkCreateInstance)(&create_info, ptr::null(), &mut instance);
        if res != VK_SUCCESS || instance == VK_NULL_HANDLE {
            return Err(format!("vkCreateInstance failed: {}", res));
        }

        let surface_info = VkAndroidSurfaceCreateInfoKHR {
            sType: VK_STRUCTURE_TYPE_ANDROID_SURFACE_CREATE_INFO_KHR,
            pNext: ptr::null(),
            flags: 0,
            window,
        };
        let mut surface = VK_NULL_HANDLE;
        let res =
            (loader.vkCreateAndroidSurfaceKHR)(instance, &surface_info, ptr::null(), &mut surface);
        if res != VK_SUCCESS || surface == VK_NULL_HANDLE {
            return Err(format!("vkCreateAndroidSurfaceKHR failed: {}", res));
        }

        Ok(Self {
            loader,
            instance,
            surface,
            _window: window,
        })
    }

    /// Expose the raw `VkSurfaceKHR` handle for further setup (device, swapchain).
    pub fn surface(&self) -> VkSurfaceKHR {
        self.surface
    }

    /// Expose the raw `VkInstance` handle.
    pub fn instance(&self) -> VkInstance {
        self.instance
    }

    /// dlsym a device-level function from the loader.
    ///
    /// # Safety
    /// `name` must be a valid Vulkan function; return type must match.
    pub unsafe fn get_proc(&self, name: &str) -> *const c_void {
        let c = std::ffi::CString::new(name).unwrap();
        (self.loader.vkGetInstanceProcAddr)(self.instance, c.as_ptr())
    }
}

impl Drop for VulkanDisplay {
    fn drop(&mut self) {
        unsafe {
            if self.surface != VK_NULL_HANDLE {
                let f = self.get_proc("vkDestroySurfaceKHR");
                if !f.is_null() {
                    let f: extern "C" fn(VkInstance, VkSurfaceKHR, *const c_void) =
                        std::mem::transmute(f);
                    f(self.instance, self.surface, ptr::null());
                }
            }
            if self.instance != VK_NULL_HANDLE {
                let f = self.get_proc("vkDestroyInstance");
                if !f.is_null() {
                    let f: extern "C" fn(VkInstance, *const c_void) = std::mem::transmute(f);
                    f(self.instance, ptr::null());
                }
            }
            if !self.loader.handle.is_null() {
                libc::dlclose(self.loader.handle);
            }
        }
    }
}
