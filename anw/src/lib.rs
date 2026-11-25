//! softisp_anw: ANativeWindow consumer for softisp IPA on Android
//!
//! Consumes ANativeWindow buffers and runs them through the cam-ipa IPA
//! pipeline (MNN/Vulkan GPU engine). The presentation logic (Viewport,
//! WindowTarget, present(), paint_bars()) lives in presentation.rs and is
//! reused here for displaying processed frames.
//!
//! Build: cargo build --release -p softisp_anw (for Android target)
//! APK:   cargo apk2 build -p softisp_anw

use cam_ipa::{Buffer, IpaConfig, IpaContext, IpaFormat, IpaSettings, IpaSensorInfo, MemBuffer, Size};
use cam_isp::profile::PipelineProfile;
use ndk_sys::{ANativeWindow, ANativeWindow_Buffer};
use std::sync::Mutex;

mod presentation;
use presentation::WindowTarget;

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

    let config = IpaConfig {
        image_width: 1920,
        image_height: 1080,
        format: IpaFormat::BGRA,
        num_output_buffers: 2,
        profile: PipelineProfile::FUSED,
        ..IpaConfig::default()
    };

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

/// Present an RGBA buffer to the ANativeWindow.
///
/// Called from the Android NativeActivity loop when a new frame is ready to
/// display. Wraps the ANativeWindow in a WindowTarget and blits the BGRA
/// RGBA data into it with aspect-preserving letterboxing.
#[no_mangle]
pub extern "C" fn softisp_anw_present(
    window: *mut ANativeWindow,
    src: *const u8,
    src_w: u32,
    src_h: u32,
) -> i32 {
    if window.is_null() || src.is_null() {
        return -1;
    }

    let mut guard = match ANW_CTX.lock() {
        Ok(g) => g,
        Err(_) => return -1,
    };

    let Some(AnwIpACtx(ctx_ptr)) = guard.as_mut() else {
        return -1;
    };

    let ctx = unsafe { &mut **ctx_ptr };

    // Create a window target for the ANativeWindow
    // Safety: window pointer is valid for the duration of this call
    let window_ref = unsafe { &*window };
    let window_target = match WindowTarget::new(window_ref, src_w as i32, src_h as i32) {
        Ok(wt) => wt,
        Err(_) => return -1,
    };

    // Present the BGRA frame into the window
    match window_target.present(src, src_w, src_h) {
        Ok(_) => 0,
        Err(_) => -1,
    }
}

/// Process a raw Bayer frame through the IPA pipeline and present to ANativeWindow.
///
/// This is the main entry point for the camera capture -> ISP -> display flow.
/// Takes a raw Bayer (u16) input buffer, processes it through the IPA pipeline
/// (3A + neural + ISP blocks), and displays the BGRA output on the ANativeWindow.
#[no_mangle]
pub extern "C" fn softisp_anw_process_and_present(
    window: *mut ANativeWindow,
    input: *const u8,
    input_w: u32,
    input_h: u32,
) -> i32 {
    if window.is_null() || input.is_null() {
        return -1;
    }

    let mut guard = match ANW_CTX.lock() {
        Ok(g) => g,
        Err(_) => return -1,
    };

    let Some(AnwIpACtx(ctx_ptr)) = guard.as_mut() else {
        return -1;
    };

    let ctx = unsafe { &mut **ctx_ptr };

    let w = input_w;
    let h = input_h;
    let input_size = (w * h * 2) as usize; // 16-bit Bayer

    // Safety: input pointer is valid for the duration of this call
    let input_data = unsafe { std::slice::from_raw_parts(input, input_size) };

    let mut input_buf = MemBuffer::new(input_data.len());
    input_buf.data.copy_from_slice(input_data);

    let output_size = (w * h * 4) as usize; // BGRA
    let mut output_buf = MemBuffer::new(output_size);

    let mut controls = cam_ipa::ControlList::new();
    controls.set_f32("AnalogueGain", 2.0);
    controls.set_i32("ExposureTime", 2000);
    controls.set_f32_array("ColourGains", &[1.5, 1.2]);

    // Queue, compute params, process stats, then process frame through IPA
    if ctx.queue_request(1, &controls).is_err() {
        return -1;
    }
    if ctx.compute_params(1).is_err() {
        return -1;
    }

    let stats_buf = MemBuffer::new(((w / 4) * (h / 4) * 4) as usize);
    if ctx.process_stats(1, 1, &stats_buf).is_err() {
        return -1;
    }

    if ctx.process_frame(
        1, 1,
        &mut input_buf, &mut output_buf,
        0, 0, w, h, &controls,
    ).is_err() {
        return -1;
    }

    // Present the BGRA output to the ANativeWindow
    let window_ref = unsafe { &*window };
    let window_target = match WindowTarget::new(window_ref, w as i32, h as i32) {
        Ok(wt) => wt,
        Err(_) => return -1,
    };

    match window_target.present(&output_buf.data, w, h) {
        Ok(_) => 0,
        Err(_) => -1,
    }
}
