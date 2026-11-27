#![cfg(all(feature = "direct_vulkan", target_os = "android"))]
//! On-device proof of the standalone Vulkan AHB/VkBuffer path.
//!
//! Both buffers are BLOB AHBs, which are opaque byte containers holding
//! little-endian RAW16 words and packed BGRA words. The direct engine imports
//! both once, reuses the imported buffers on a second dispatch, waits on a
//! Vulkan fence, and leaves non-black pixels in the output AHB.
//!
//! ```text
//! LD_LIBRARY_PATH=$PWD/cam-rust/lib/aarch64-v8a \
//!   cargo test -p cam-isp --test test_direct_vulkan_ahb --features direct_vulkan \
//!   -- --ignored --nocapture --test-threads=1
//! ```

use cam_isp::direct_vulkan_sys::DirectVulkanContext;
use std::ffi::c_void;
use std::os::raw::{c_int, c_uint};
use std::ptr::null_mut;
use std::slice;

const W: u32 = 16;
const H: u32 = 16;
const FORMAT_BLOB: c_uint = 0x21;
const USAGE_CPU_RW: u64 = 3 | (3 << 4);
const USAGE_CPU_READ: u64 = 1;
const USAGE_GPU_DATA_BUFFER: u64 = 1 << 24;

#[repr(C)]
#[derive(Default)]
struct AhbDesc {
    width: u32,
    height: u32,
    layers: u32,
    format: u32,
    usage: u64,
    stride: u32,
    rfu0: u32,
    rfu1: u64,
}

#[link(name = "android")]
extern "C" {
    fn AHardwareBuffer_allocate(desc: *const AhbDesc, out: *mut *mut c_void) -> c_int;
    fn AHardwareBuffer_release(buffer: *mut c_void);
    fn AHardwareBuffer_lock(
        buffer: *mut c_void,
        usage: u64,
        fence: i32,
        rect: *const c_void,
        out: *mut *mut c_void,
    ) -> c_int;
    fn AHardwareBuffer_unlock(buffer: *mut c_void, fence: *mut i32) -> c_int;
}

struct TestAhb(*mut c_void);

impl TestAhb {
    fn allocate(bytes: usize) -> Self {
        let mut ptr = null_mut();
        let rc = unsafe {
            AHardwareBuffer_allocate(
                &AhbDesc {
                    width: bytes as u32,
                    height: 1,
                    layers: 1,
                    format: FORMAT_BLOB,
                    usage: USAGE_CPU_RW | USAGE_GPU_DATA_BUFFER,
                    ..Default::default()
                },
                &mut ptr,
            )
        };
        assert_eq!(rc, 0, "AHardwareBuffer_allocate failed");
        assert!(!ptr.is_null());
        Self(ptr)
    }

    fn map(&self, usage: u64) -> *mut u8 {
        let mut ptr = null_mut();
        let rc = unsafe { AHardwareBuffer_lock(self.0, usage, -1, null_mut(), &mut ptr) };
        assert_eq!(rc, 0, "AHardwareBuffer_lock failed");
        assert!(!ptr.is_null());
        ptr as *mut u8
    }

    fn unmap(&self) {
        let rc = unsafe { AHardwareBuffer_unlock(self.0, null_mut()) };
        assert_eq!(rc, 0, "AHardwareBuffer_unlock failed");
    }
}

impl Drop for TestAhb {
    fn drop(&mut self) {
        unsafe { AHardwareBuffer_release(self.0) };
    }
}

#[test]
#[ignore = "requires an Android device with Vulkan and AHardwareBuffer support"]
fn raw16_words_to_bgra_ahb_via_vulkan_buffer() {
    let pixels = (W as usize) * (H as usize);
    let input = TestAhb::allocate(pixels * 2);
    let output = TestAhb::allocate(pixels * 4);

    let input_ptr = input.map(USAGE_CPU_RW);
    let samples = unsafe { slice::from_raw_parts_mut(input_ptr as *mut u16, pixels) };
    for (index, value) in samples.iter_mut().enumerate() {
        *value = (index % 1024) as u16;
    }
    input.unmap();

    let ctx =
        DirectVulkanContext::new().expect("direct Vulkan context should be created on device");
    for _ in 0..2 {
        ctx.run_ahb(input.0, pixels * 2, output.0, pixels * 4, W, H, 0)
            .expect("AHB->AHB Vulkan run should succeed");
    }

    let output_ptr = output.map(USAGE_CPU_READ);
    let words = unsafe { slice::from_raw_parts(output_ptr as *const u32, pixels) };
    assert!(
        words.iter().any(|word| *word != 0 && *word != 0xff000000),
        "output must contain non-black BGRA pixels"
    );
    assert_eq!(words[0] & 0xff, 0);
    output.unmap();
}
