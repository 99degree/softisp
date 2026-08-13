//! C ABI for the softisp ISP pipeline.
//!
//! Exposes a minimal set of functions callable from C/C++ and Android NDK
//! code (via JNI or direct FFI) for composing ISP pipelines without
//! requiring the full Rust toolchain at the call site.
//!
//! The `.so` is built with `crate-type = ["cdylib", "rlib"]` and exposes:
//!
//! ```c
//! // Compose an ONNX model for a given profile + resolution.
//! // Returns 0 on success, non-zero error code on failure.
//! // On success, `*out_len` is set and `*out_bytes` points to malloc'd memory
//! // (caller must free with softisp_free).
//! int softisp_compose_onnx(int width, int height, const char* profile,
//!                          uint8_t** out_bytes, size_t* out_len);
//!
//! // Compose ONNX→MNN (requires `mnn` feature).
//! int softisp_compose_mnn(int width, int height, const char* profile,
//!                         uint8_t** out_bytes, size_t* out_len);
//!
//! // Free memory returned by softisp_compose_*.
//! void softisp_free(uint8_t* bytes, size_t len);
//! ```

use crate::pipeline::GraphComposer;
use crate::pipeline::IspBlock;
use crate::profile::PipelineProfile;
use std::ffi::CStr;
use std::os::raw::c_char;

/// Error codes returned by the C ABI functions.
pub const SOFTISP_OK: i32 = 0;
pub const SOFTISP_ERR_INVALID_INPUT: i32 = 1;
pub const SOFTISP_ERR_COMPOSE_FAILED: i32 = 2;
pub const SOFTISP_ERR_CONVERT_FAILED: i32 = 3;
pub const SOFTISP_ERR_UNKNOWN_PROFILE: i32 = 4;

/// Resolve a profile string to a PipelineProfile.
fn parse_profile(s: &str) -> Option<PipelineProfile> {
    match s {
        "LITE" => Some(PipelineProfile::LITE),
        "MED" => Some(PipelineProfile::MED),
        "HEAVY" => Some(PipelineProfile::HEAVY),
        "PRO" => Some(PipelineProfile::PRO),
        "UNIFIED" => Some(PipelineProfile::UNIFIED),
        "HDR" => Some(PipelineProfile::HDR),
        "TEST" => Some(PipelineProfile::TEST),
        _ => None,
    }
}

/// Compose a pipeline for the given profile and resolution.
fn compose_pipeline(w: u32, h: u32, profile: PipelineProfile) -> Result<Vec<u8>, String> {
    let blocks = profile.build_blocks(w, 0);
    let mut pipeline = blocks;
    pipeline.push(Box::new(crate::blocks::DisplayBlock::new(h)));
    GraphComposer::wire_blocks(&mut pipeline);
    let block_refs: Vec<&dyn IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    GraphComposer::compose_from_vec(&block_refs, &[], 16)
}

/// Compose an ONNX model for the given profile + resolution.
///
/// # Safety
///
/// - `profile` must be a valid null-terminated C string.
/// - `out_bytes` must be non-null and will be set to point to a newly
///   allocated buffer (caller frees with `softisp_free`).
/// - `out_len` must be non-null and will be set to the buffer length.
#[no_mangle]
pub unsafe extern "C" fn softisp_compose_onnx(
    width: i32,
    height: i32,
    profile: *const c_char,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if profile.is_null() || out_bytes.is_null() || out_len.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let profile_str = match CStr::from_ptr(profile).to_str() {
        Ok(s) => s,
        Err(_) => return SOFTISP_ERR_INVALID_INPUT,
    };

    if width <= 0 || height <= 0 {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let w = width as u32;
    let h = height as u32;
    let profile_enum = match parse_profile(profile_str) {
        Some(p) => p,
        None => return SOFTISP_ERR_UNKNOWN_PROFILE,
    };

    let onnx = match compose_pipeline(w, h, profile_enum) {
        Ok(bytes) => bytes,
        Err(e) => {
            eprintln!("[softisp] compose_from_vec failed: {}", e);
            return SOFTISP_ERR_COMPOSE_FAILED;
        }
    };

    let len = onnx.len();
    let ptr = onnx.as_ptr();
    std::mem::forget(onnx);

    *out_bytes = ptr as *mut u8;
    *out_len = len;
    SOFTISP_OK
}

/// Compose an ONNX→MNN model for the given profile + resolution.
///
/// Requires the `mnn` feature at build time.
///
/// # Safety
///
/// Same as `softisp_compose_onnx`.
#[cfg(feature = "mnn")]
#[no_mangle]
pub unsafe extern "C" fn softisp_compose_mnn(
    width: i32,
    height: i32,
    profile: *const c_char,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if profile.is_null() || out_bytes.is_null() || out_len.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let profile_str = match CStr::from_ptr(profile).to_str() {
        Ok(s) => s,
        Err(_) => return SOFTISP_ERR_INVALID_INPUT,
    };

    if width <= 0 || height <= 0 {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let w = width as u32;
    let h = height as u32;
    let profile_enum = match parse_profile(profile_str) {
        Some(p) => p,
        None => return SOFTISP_ERR_UNKNOWN_PROFILE,
    };

    let onnx = match compose_pipeline(w, h, profile_enum) {
        Ok(bytes) => bytes,
        Err(e) => {
            eprintln!("[softisp] compose_from_vec failed: {}", e);
            return SOFTISP_ERR_COMPOSE_FAILED;
        }
    };

    let mnn = match crate::mnn_converter::convert_onnx_buffer(&onnx) {
        Ok(bytes) => bytes,
        Err(e) => {
            eprintln!("[softisp] onnx→mnn convert failed: {}", e);
            return SOFTISP_ERR_CONVERT_FAILED;
        }
    };

    let len = mnn.len();
    let ptr = mnn.as_ptr();
    std::mem::forget(mnn);

    *out_bytes = ptr as *mut u8;
    *out_len = len;
    SOFTISP_OK
}

/// Free a buffer returned by `softisp_compose_onnx` or `softisp_compose_mnn`.
///
/// # Safety
///
/// `bytes` must have been returned by one of the compose functions, and
/// `len` must match the original allocation size.
#[no_mangle]
pub unsafe extern "C" fn softisp_free(bytes: *mut u8, _len: usize) {
    if bytes.is_null() {
        return;
    }
    let _ = Box::from_raw(bytes);
}

/// Get the version string of the cam-isp library.
#[no_mangle]
pub extern "C" fn softisp_version() -> *const c_char {
    c"cam-isp 0.1.0 GPL-3.0-or-later".as_ptr()
}
