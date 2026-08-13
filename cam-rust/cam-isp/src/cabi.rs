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
//! // Set custom opset mode for ONNX emission.
//! int softisp_set_opset_mode(int mode);  // 0=primitive, 1=custom
//!
//! // Free memory returned by softisp_compose_*.
//! void softisp_free(uint8_t* bytes, size_t len);
//! ```

use crate::pipeline::GraphComposer;
use crate::pipeline::{BlockOpsetMode, IspBlock};
use crate::profile::PipelineProfile;
use std::ffi::CStr;
use std::os::raw::c_char;

/// Error codes returned by the C ABI functions.
pub const SOFTISP_OK: i32 = 0;
pub const SOFTISP_ERR_INVALID_INPUT: i32 = 1;
pub const SOFTISP_ERR_COMPOSE_FAILED: i32 = 2;
pub const SOFTISP_ERR_CONVERT_FAILED: i32 = 3;
pub const SOFTISP_ERR_UNKNOWN_PROFILE: i32 = 4;

/// Opset mode for ONNX emission.
pub const SOFTISP_OPSET_PRIMITIVE: i32 = 0;
pub const SOFTISP_OPSET_CUSTOM: i32 = 1;

// Global opset mode override (matches PipelineBuilder::force_mode).
static GLOBAL_OPSET_MODE: std::sync::Mutex<Option<BlockOpsetMode>> = std::sync::Mutex::new(None);

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

/// Resolve an opset mode integer to BlockOpsetMode.
fn parse_opset_mode(mode: i32) -> Option<BlockOpsetMode> {
    match mode {
        SOFTISP_OPSET_PRIMITIVE => Some(BlockOpsetMode::Primitive),
        SOFTISP_OPSET_CUSTOM => Some(BlockOpsetMode::Custom),
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

/// Compose a pipeline with an optional global opset mode override.
fn compose_pipeline_with_mode(
    w: u32,
    h: u32,
    profile: PipelineProfile,
    force_mode: Option<BlockOpsetMode>,
) -> Result<Vec<u8>, String> {
    let blocks = profile.build_blocks(w, 0);
    let mut pipeline = blocks;
    pipeline.push(Box::new(crate::blocks::DisplayBlock::new(h)));
    GraphComposer::wire_blocks(&mut pipeline);
    let block_refs: Vec<&dyn IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    GraphComposer::compose_from_vec_with_mode(&block_refs, &[], 16, force_mode)
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

    // Apply global opset mode override if set
    let force_mode = GLOBAL_OPSET_MODE.lock().unwrap().clone();

    let onnx = match compose_pipeline_with_mode(w, h, profile_enum, force_mode) {
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

    // Apply global opset mode override if set
    let force_mode = GLOBAL_OPSET_MODE.lock().unwrap().clone();

    let onnx = match compose_pipeline_with_mode(w, h, profile_enum, force_mode) {
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

/// Set the global opset mode for ONNX emission.
///
/// This affects all subsequent softisp_compose_* calls until changed or cleared.
///
/// @param mode  SOFTISP_OPSET_PRIMITIVE (0) or SOFTISP_OPSET_CUSTOM (1).
/// @return SOFTISP_OK on success, SOFTISP_ERR_INVALID_INPUT if mode is invalid.
#[no_mangle]
pub unsafe extern "C" fn softisp_set_opset_mode(mode: i32) -> i32 {
    let new_mode = match parse_opset_mode(mode) {
        Some(m) => m,
        None => return SOFTISP_ERR_INVALID_INPUT,
    };
    let mut guard = GLOBAL_OPSET_MODE.lock().unwrap();
    *guard = Some(new_mode);
    SOFTISP_OK
}

/// Clear the global opset mode override.
///
/// After calling this, blocks use their own opset_mode() again.
#[no_mangle]
pub unsafe extern "C" fn softisp_clear_opset_mode() -> i32 {
    let mut guard = GLOBAL_OPSET_MODE.lock().unwrap();
    *guard = None;
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

/// Get a human-readable error string for an error code.
#[no_mangle]
pub extern "C" fn softisp_error_string(code: i32) -> *const c_char {
    match code {
        SOFTISP_OK => c"OK".as_ptr(),
        SOFTISP_ERR_INVALID_INPUT => c"Invalid input".as_ptr(),
        SOFTISP_ERR_COMPOSE_FAILED => c"Pipeline composition failed".as_ptr(),
        SOFTISP_ERR_CONVERT_FAILED => c"ONNX→MNN conversion failed".as_ptr(),
        SOFTISP_ERR_UNKNOWN_PROFILE => c"Unknown profile".as_ptr(),
        _ => c"Unknown error".as_ptr(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cabi_compose_onnx() {
        let mut out_bytes: *mut u8 = std::ptr::null_mut();
        let mut out_len: usize = 0;
        let profile = std::ffi::CString::new("LITE").unwrap();
        let rc = unsafe {
            softisp_compose_onnx(640, 480, profile.as_ptr(), &mut out_bytes, &mut out_len)
        };
        assert_eq!(rc, SOFTISP_OK);
        assert!(!out_bytes.is_null());
        assert!(out_len > 0);
        unsafe { softisp_free(out_bytes, out_len) };
    }

    #[test]
    fn test_cabi_compose_unknown_profile() {
        let mut out_bytes: *mut u8 = std::ptr::null_mut();
        let mut out_len: usize = 0;
        let profile = std::ffi::CString::new("UNKNOWN").unwrap();
        let rc = unsafe {
            softisp_compose_onnx(640, 480, profile.as_ptr(), &mut out_bytes, &mut out_len)
        };
        assert_eq!(rc, SOFTISP_ERR_UNKNOWN_PROFILE);
    }

    #[test]
    fn test_cabi_opset_mode_switch() {
        // Default is primitive
        assert_eq!(unsafe { softisp_set_opset_mode(SOFTISP_OPSET_CUSTOM) }, SOFTISP_OK);
        // Switching to custom
        assert_eq!(unsafe { softisp_set_opset_mode(SOFTISP_OPSET_PRIMITIVE) }, SOFTISP_OK);
        // Invalid mode
        assert_eq!(unsafe { softisp_set_opset_mode(99) }, SOFTISP_ERR_INVALID_INPUT);
        // Clear
        assert_eq!(unsafe { softisp_clear_opset_mode() }, SOFTISP_OK);
    }

    #[test]
    fn test_cabi_error_strings() {
        assert!(!unsafe { softisp_error_string(SOFTISP_OK) }.is_null());
        assert!(!unsafe { softisp_error_string(SOFTISP_ERR_COMPOSE_FAILED) }.is_null());
        assert!(!unsafe { softisp_error_string(-1) }.is_null());
    }

    #[test]
    fn test_cabi_version() {
        assert!(!unsafe { softisp_version() }.is_null());
    }
}
