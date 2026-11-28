#![cfg(all(feature = "direct_vulkan", target_os = "android"))]
//! On-device perf/correctness proof for the AHB-backed storage `VkImage` path.
//!
//! The output AHB is RGBA_8888 with `GPU_FRAMEBUFFER`, exactly like the
//! production display slots. It can be imported as a `VkImage` and written by
//! the compute shader, then handed to SurfaceFlinger with no CPU read window.
//!
//! ```text
//! LD_LIBRARY_PATH=$PWD/cam-rust/lib/aarch64-v8a \
//!   cargo test -p cam-isp --test test_direct_vulkan_image --features direct_vulkan \
//!   -- --ignored --nocapture --test-threads=1
//! ```

use cam_isp::direct_vulkan_sys::{probe_dmabuf_ahb, probe_opaque_fd_ahb, DirectVulkanContext};
use std::ffi::c_void;
use std::os::raw::{c_int, c_uint};
use std::ptr::null_mut;
use std::slice;
use std::time::Instant;

const W: u32 = 64;
const H: u32 = 48;
const FORMAT_BLOB: c_uint = 0x21;
const FORMAT_RGBA8888: c_uint = 1;
const FORMAT_RAW16: c_uint = 0x20;
const USAGE_CPU_RW: u64 = 1 | (1 << 2);
const USAGE_CPU_READ: u64 = 1;
const USAGE_GPU_DATA_BUFFER: u64 = 1 << 24;
const USAGE_GPU_FRAMEBUFFER: u64 = 1 << 9;
const USAGE_GPU_SAMPLED_IMAGE: u64 = 1 << 8;

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
    fn allocate(width: u32, height: u32, format: c_uint, usage: u64) -> Self {
        let mut ptr = null_mut();
        let rc = unsafe {
            AHardwareBuffer_allocate(
                &AhbDesc {
                    width,
                    height,
                    layers: 1,
                    format,
                    usage,
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
fn probe_and_run_ahb_storage_image() {
    let pixels = (W as usize) * (H as usize);
    let words = (pixels + 1) / 2;
    let input = TestAhb::allocate(
        (words * 4) as u32,
        1,
        FORMAT_BLOB,
        USAGE_CPU_RW | USAGE_GPU_DATA_BUFFER,
    );
    let output = TestAhb::allocate(W, H, FORMAT_RGBA8888, USAGE_CPU_RW | USAGE_GPU_FRAMEBUFFER);

    let input_ptr = input.map(USAGE_CPU_RW);
    let samples = unsafe { slice::from_raw_parts_mut(input_ptr as *mut u32, words) };
    for (index, value) in samples.iter_mut().enumerate() {
        let lo = (index % 1024) as u32;
        let hi = ((index * 3 + 7) % 1024) as u32;
        *value = lo | (hi << 16);
    }
    input.unmap();

    let ctx = match DirectVulkanContext::new() {
        Ok(ctx) => ctx,
        Err(err) => {
            println!("skip AHB image run (ICD lacks AHB external memory): {err}");
            return;
        }
    };

    let (ok, detail) = ctx.probe_image_ahb(output.0, W, H);
    assert!(
        ok,
        "RGBA_8888 graphics AHB must import as storage VkImage: {detail}"
    );

    let iterations = 32;
    let t0 = Instant::now();
    for _ in 0..iterations {
        ctx.run_ahb_image(input.0, words * 4, output.0, W, H, 0)
            .expect("AHB -> storage VkImage run should succeed");
    }
    let elapsed = t0.elapsed();
    println!(
        "image output: {} dispatches in {:?} ({:.3} ms/run)",
        iterations,
        elapsed,
        elapsed.as_secs_f64() * 1000.0 / iterations as f64,
    );

    let output_ptr = output.map(USAGE_CPU_RW);
    let words_out = unsafe { slice::from_raw_parts(output_ptr as *const u32, pixels) };
    assert!(
        words_out.iter().any(|word| *word & 0x00ff_ffff != 0),
        "output image must contain non-black pixels"
    );
    output.unmap();
}

#[test]
#[ignore = "requires an Android device with Vulkan and AHardwareBuffer support"]
fn raw16_ahb_explicit_vulkan_format_import() {
    let raw = TestAhb::allocate(W, H, FORMAT_RAW16, USAGE_CPU_READ | USAGE_GPU_SAMPLED_IMAGE);
    let ctx = match DirectVulkanContext::new() {
        Ok(ctx) => ctx,
        Err(err) => {
            println!("skip AHB format probe (ICD lacks AHB external memory): {err}");
            return;
        }
    };
    let (ok, detail) = ctx.probe_raw16_image_ahb(raw.0, W, H);
    println!("RAW16 explicit format probe: ok={ok} {detail}");
    println!(
        "external memory extensions: {}",
        ctx.probe_external_memory_extensions(raw.0)
    );
    // No assertion: this is a capability probe. The result is recorded so we
    // can decide whether the camera AHB can ever be consumed without a copy.
}

#[test]
#[ignore = "requires an Android device with Vulkan and AHardwareBuffer support"]
fn raw16_ahb_dmabuf_import() {
    let raw = TestAhb::allocate(W, H, FORMAT_RAW16, USAGE_CPU_READ | USAGE_GPU_SAMPLED_IMAGE);
    let (ok, detail) = probe_dmabuf_ahb(raw.0);
    println!("RAW16 dma_buf AHB import: ok={ok} {detail}");
    // Capability probe: vendor ICD and Mesa Turnip differ here.
}

#[test]
#[ignore = "requires an Android device with Vulkan and AHardwareBuffer support"]
fn raw16_ahb_opaque_fd_import() {
    let raw = TestAhb::allocate(W, H, FORMAT_RAW16, USAGE_CPU_READ | USAGE_GPU_SAMPLED_IMAGE);
    let (ok, detail) = probe_opaque_fd_ahb(raw.0);
    println!("RAW16 opaque_fd AHB import: ok={ok} {detail}");
    // Cheap experiment: the camera fd is dma_buf, not a Vulkan-opaque fd.
}

#[test]
#[ignore = "requires an Android device with Vulkan and AHardwareBuffer support"]
fn raw16_ahb_fd_image_run() {
    const FD_W: u32 = 80;
    const FD_H: u32 = 60;
    const PIXELS: usize = (FD_W as usize) * (FD_H as usize);
    let input = TestAhb::allocate(
        FD_W,
        FD_H,
        FORMAT_RAW16,
        USAGE_CPU_RW | USAGE_GPU_SAMPLED_IMAGE,
    );
    let output = TestAhb::allocate(
        FD_W,
        FD_H,
        FORMAT_RGBA8888,
        USAGE_CPU_RW | USAGE_GPU_FRAMEBUFFER,
    );

    let input_ptr = input.map(USAGE_CPU_RW);
    let words = unsafe { slice::from_raw_parts_mut(input_ptr as *mut u32, PIXELS / 2) };
    for (index, word) in words.iter_mut().enumerate() {
        let lo = (index % 1024) as u32;
        let hi = ((index * 3 + 7) % 1024) as u32;
        *word = lo | (hi << 16);
    }
    input.unmap();

    let ctx = match DirectVulkanContext::new() {
        Ok(ctx) => ctx,
        Err(err) => {
            println!("skip fd image run (no Vulkan compute context): {err}");
            return;
        }
    };
    if !ctx.supports_dma_buf() {
        println!("skip fd image run (ICD lacks VK_EXT_external_memory_dma_buf)");
        return;
    }

    let iterations = 32;
    let t0 = Instant::now();
    for _ in 0..iterations {
        ctx.run_ahb_fd_image(input.0, PIXELS * 4, output.0, FD_W, FD_H, 0)
            .expect("RAW16 dma_buf fd -> storage VkImage run should succeed");
    }
    let elapsed = t0.elapsed();
    println!(
        "fd image output: {} dispatches in {:?} ({:.3} ms/run)",
        iterations,
        elapsed,
        elapsed.as_secs_f64() * 1000.0 / iterations as f64,
    );

    let output_ptr = output.map(USAGE_CPU_RW);
    let out_words = unsafe { slice::from_raw_parts(output_ptr as *const u32, PIXELS) };
    assert!(
        out_words.iter().any(|word| *word & 0x00ff_ffff != 0),
        "fd image output must contain non-black pixels"
    );
    output.unmap();
}

#[test]
#[ignore = "requires an Android device with Vulkan and AHardwareBuffer support"]
fn raw16_ahb_native_handle_layout() {
    let raw = TestAhb::allocate(
        80,
        60,
        FORMAT_RAW16,
        USAGE_CPU_READ | USAGE_GPU_SAMPLED_IMAGE,
    );
    let (ok, detail) = probe_dmabuf_ahb(raw.0);
    println!("RAW16 80x60 native-handle layout: ok={ok} {detail}");
    // Confirms where width/height/stride/format live in the private handle.
}
