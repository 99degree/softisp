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
//! // Create a custom pipeline profile.
//! SoftispProfile* softisp_profile_create(void);
//! void softisp_profile_free(SoftispProfile* profile);
//! int softisp_profile_set_flag(SoftispProfile* profile, const char* flag, int value);
//! int softisp_profile_set_demosaic_quality(SoftispProfile* profile, int quality);
//! int softisp_profile_set_rotate_mode(SoftispProfile* profile, int mode);
//! int softisp_profile_set_output_format(SoftispProfile* profile, int format);
//! int softisp_compose_onnx_with_profile(int width, int height,
//!                                       const SoftispProfile* profile,
//!                                       uint8_t** out_bytes, size_t* out_len);
//!
//! // Free memory returned by softisp_compose_*.
//! void softisp_free(uint8_t* bytes, size_t len);
//! ```

use crate::engine::OutputFormat;
use crate::profile::{DemosaicQuality, PipelineLevel, PipelineProfile};
use crate::pipeline::GraphComposer;
use crate::pipeline::{BlockOpsetMode, IspBlock};
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

// Output format constants (mirrors softisp_api.h).
pub const SOFTISP_OUTPUT_FLOAT_RGB: i32 = 0;
pub const SOFTISP_OUTPUT_FLOAT_BGRA: i32 = 1;
pub const SOFTISP_OUTPUT_PACKED_RGB: i32 = 2;
pub const SOFTISP_OUTPUT_BGRA: i32 = 3;
pub const SOFTISP_OUTPUT_RGBA: i32 = 4;
pub const SOFTISP_OUTPUT_ARGB: i32 = 5;
pub const SOFTISP_OUTPUT_ABGR: i32 = 6;
pub const SOFTISP_OUTPUT_RGB: i32 = 7;
pub const SOFTISP_OUTPUT_BGR: i32 = 8;
pub const SOFTISP_OUTPUT_FLOAT16_RGB: i32 = 9;
pub const SOFTISP_OUTPUT_FLOAT16_BGRA: i32 = 10;

// Demosaic quality constants (mirrors softisp_api.h).
pub const SOFTISP_DEMOSAIC_STANDARD: i32 = 0;
pub const SOFTISP_DEMOSAIC_HQ_LINEAR: i32 = 1;
pub const SOFTISP_DEMOSAIC_EDGE: i32 = 2;

// Global opset mode override (matches PipelineBuilder::force_mode).
static GLOBAL_OPSET_MODE: std::sync::Mutex<Option<BlockOpsetMode>> = std::sync::Mutex::new(None);

// ── Custom profile config ─────────────────────────────────────────────────

/// Mutable configuration for a custom pipeline profile.
/// Boxed and exposed as an opaque `SoftispProfile*` to C.
#[derive(Debug, Clone, Copy)]
pub struct CustomProfile {
    label: &'static str,
    level: PipelineLevel,
    use_unpack: bool,
    force_input16: bool,
    use_fcs: bool,
    use_ldci: bool,
    use_ee: bool,
    use_bad_pixel: bool,
    demosaic_quality: DemosaicQuality,
    use_local_contrast: bool,
    use_unsharp: bool,
    use_lsc: bool,
    use_warp: bool,
    use_hdr: bool,
    rotate_mode: i32,
    use_zone_stats: bool,
    use_channel_means: bool,
    use_tone_stats: bool,
    use_histogram: bool,
    stats_downscale_max: u32,
    pipeline_downscale_target: u32,
    eis_margin: f64,
    use_bilateral: bool,
    use_saturation: bool,
    use_vignetting: bool,
    use_colorspace: bool,
    use_gamma: bool,
    use_sharpen: bool,
    use_wavelet_denoise: bool,
    use_auto_contrast: bool,
    use_normalize: bool,
    use_tiled_rendering: bool,
    tile_count_x: u32,
    tile_count_y: u32,
    tile_overlap: u32,
    output_format: OutputFormat,
}

impl Default for CustomProfile {
    fn default() -> Self {
        Self {
            label: "CUSTOM",
            level: PipelineLevel::Medium,
            use_unpack: true,
            force_input16: false,
            use_fcs: false,
            use_ldci: false,
            use_ee: true,
            use_bad_pixel: true,
            demosaic_quality: DemosaicQuality::Standard,
            use_local_contrast: false,
            use_unsharp: true,
            use_lsc: false,
            use_warp: false,
            use_hdr: false,
            rotate_mode: 0,
            use_zone_stats: true,
            use_channel_means: true,
            use_tone_stats: true,
            use_histogram: false,
            stats_downscale_max: 0,
            pipeline_downscale_target: 0,
            eis_margin: 0.0,
            use_bilateral: false,
            use_saturation: false,
            use_vignetting: false,
            use_colorspace: false,
            use_gamma: false,
            use_sharpen: false,
            use_wavelet_denoise: false,
            use_auto_contrast: false,
            use_normalize: false,
            use_tiled_rendering: false,
            tile_count_x: 1,
            tile_count_y: 1,
            tile_overlap: 0,
            output_format: OutputFormat::Bgra,
        }
    }
}

impl From<CustomProfile> for PipelineProfile {
    fn from(c: CustomProfile) -> Self {
        Self {
            label: c.label,
            level: c.level,
            use_unpack: c.use_unpack,
            force_input16: c.force_input16,
            use_fcs: c.use_fcs,
            use_ldci: c.use_ldci,
            use_ee: c.use_ee,
            use_bad_pixel: c.use_bad_pixel,
            demosaic_quality: c.demosaic_quality,
            use_local_contrast: c.use_local_contrast,
            use_unsharp: c.use_unsharp,
            use_lsc: c.use_lsc,
            use_warp: c.use_warp,
            use_hdr: c.use_hdr,
            rotate_mode: c.rotate_mode,
            use_zone_stats: c.use_zone_stats,
            use_channel_means: c.use_channel_means,
            use_tone_stats: c.use_tone_stats,
            use_histogram: c.use_histogram,
            stats_downscale_max: c.stats_downscale_max,
            pipeline_downscale_target: c.pipeline_downscale_target,
            eis_margin: c.eis_margin,
            use_bilateral: c.use_bilateral,
            use_saturation: c.use_saturation,
            use_vignetting: c.use_vignetting,
            use_colorspace: c.use_colorspace,
            use_gamma: c.use_gamma,
            use_sharpen: c.use_sharpen,
            use_wavelet_denoise: c.use_wavelet_denoise,
            use_auto_contrast: c.use_auto_contrast,
            use_normalize: c.use_normalize,
            use_tiled_rendering: c.use_tiled_rendering,
            tile_count_x: c.tile_count_x,
            tile_count_y: c.tile_count_y,
            tile_overlap: c.tile_overlap,
            output_format: c.output_format,
        }
    }
}

// ── Helper functions ──────────────────────────────────────────────────────

/// Resolve an opset mode integer to BlockOpsetMode.
fn parse_opset_mode(mode: i32) -> Option<BlockOpsetMode> {
    match mode {
        SOFTISP_OPSET_PRIMITIVE => Some(BlockOpsetMode::Primitive),
        SOFTISP_OPSET_CUSTOM => Some(BlockOpsetMode::Custom),
        _ => None,
    }
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

// ── C ABI: custom profile ─────────────────────────────────────────────────

/// Opaque handle type for C.
pub type SoftispProfile = CustomProfile;

/// Create a new custom pipeline profile with default values.
///
/// # Safety
///
/// Returns a valid pointer on success, NULL on allocation failure.
/// Caller must free with softisp_profile_free().
#[no_mangle]
pub extern "C" fn softisp_profile_create() -> *mut SoftispProfile {
    Box::into_raw(Box::new(CustomProfile::default()))
}

/// Free a custom pipeline profile.
///
/// # Safety
///
/// `profile` must have been returned by softisp_profile_create() or be NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_free(profile: *mut SoftispProfile) {
    if profile.is_null() {
        return;
    }
    let _ = Box::from_raw(profile);
}

/// Set a boolean feature flag on a custom profile.
///
/// # Safety
///
/// `profile` must be non-NULL. `flag` must be a valid null-terminated C string.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_set_flag(
    profile: *mut SoftispProfile,
    flag: *const c_char,
    value: i32,
) -> i32 {
    if profile.is_null() || flag.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let flag_str = match CStr::from_ptr(flag).to_str() {
        Ok(s) => s,
        Err(_) => return SOFTISP_ERR_INVALID_INPUT,
    };

    let val = value != 0;
    let p = &mut *profile;

    match flag_str {
        "use_unpack" => p.use_unpack = val,
        "force_input16" => p.force_input16 = val,
        "use_fcs" => p.use_fcs = val,
        "use_ldci" => p.use_ldci = val,
        "use_ee" => p.use_ee = val,
        "use_bad_pixel" => p.use_bad_pixel = val,
        "use_local_contrast" => p.use_local_contrast = val,
        "use_unsharp" => p.use_unsharp = val,
        "use_lsc" => p.use_lsc = val,
        "use_warp" => p.use_warp = val,
        "use_hdr" => p.use_hdr = val,
        "use_zone_stats" => p.use_zone_stats = val,
        "use_channel_means" => p.use_channel_means = val,
        "use_tone_stats" => p.use_tone_stats = val,
        "use_histogram" => p.use_histogram = val,
        "use_bilateral" => p.use_bilateral = val,
        "use_saturation" => p.use_saturation = val,
        "use_vignetting" => p.use_vignetting = val,
        "use_colorspace" => p.use_colorspace = val,
        "use_gamma" => p.use_gamma = val,
        "use_sharpen" => p.use_sharpen = val,
        "use_wavelet_denoise" => p.use_wavelet_denoise = val,
        "use_auto_contrast" => p.use_auto_contrast = val,
        "use_normalize" => p.use_normalize = val,
        "use_tiled_rendering" => p.use_tiled_rendering = val,
        _ => return SOFTISP_ERR_INVALID_INPUT,
    }

    SOFTISP_OK
}

/// Set the demosaic quality on a custom profile.
///
/// # Safety
///
/// `profile` must be non-NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_set_demosaic_quality(
    profile: *mut SoftispProfile,
    quality: i32,
) -> i32 {
    if profile.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let q = match quality {
        SOFTISP_DEMOSAIC_STANDARD => DemosaicQuality::Standard,
        SOFTISP_DEMOSAIC_HQ_LINEAR => DemosaicQuality::HqLinear,
        SOFTISP_DEMOSAIC_EDGE => DemosaicQuality::Edge,
        _ => return SOFTISP_ERR_INVALID_INPUT,
    };

    (*profile).demosaic_quality = q;
    SOFTISP_OK
}

/// Set the rotation mode on a custom profile.
///
/// # Safety
///
/// `profile` must be non-NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_set_rotate_mode(
    profile: *mut SoftispProfile,
    mode: i32,
) -> i32 {
    if profile.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    if mode < 0 || mode > 5 {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    (*profile).rotate_mode = mode;
    SOFTISP_OK
}

/// Set the output pixel format on a custom profile.
///
/// # Safety
///
/// `profile` must be non-NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_set_output_format(
    profile: *mut SoftispProfile,
    format: i32,
) -> i32 {
    if profile.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let f = if format == SOFTISP_OUTPUT_FLOAT_RGB {
        OutputFormat::FloatRgb
    } else if format == SOFTISP_OUTPUT_FLOAT_BGRA {
        OutputFormat::FloatBgra
    } else if format == SOFTISP_OUTPUT_PACKED_RGB {
        OutputFormat::PackedRgb
    } else if format == SOFTISP_OUTPUT_BGRA {
        OutputFormat::Bgra
    } else if format == SOFTISP_OUTPUT_RGBA {
        OutputFormat::Rgba
    } else if format == SOFTISP_OUTPUT_ARGB {
        OutputFormat::Argb
    } else if format == SOFTISP_OUTPUT_ABGR {
        OutputFormat::Abgr
    } else if format == SOFTISP_OUTPUT_RGB {
        OutputFormat::Rgb
    } else if format == SOFTISP_OUTPUT_BGR {
        OutputFormat::Bgr
    } else if format == SOFTISP_OUTPUT_FLOAT16_RGB {
        OutputFormat::Float16Rgb
    } else if format == SOFTISP_OUTPUT_FLOAT16_BGRA {
        OutputFormat::Float16Bgra
    } else {
        return SOFTISP_ERR_INVALID_INPUT;
    };

    (*profile).output_format = f;
    SOFTISP_OK
}

/// Set the EIS margin fraction on a custom profile.
///
/// # Safety
///
/// `profile` must be non-NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_set_eis_margin(
    profile: *mut SoftispProfile,
    margin: std::os::raw::c_double,
) -> i32 {
    if profile.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }
    (*profile).eis_margin = margin;
    SOFTISP_OK
}

/// Set the stats downscale max dimension on a custom profile.
///
/// # Safety
///
/// `profile` must be non-NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_set_stats_downscale_max(
    profile: *mut SoftispProfile,
    max_px: u32,
) -> i32 {
    if profile.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }
    (*profile).stats_downscale_max = max_px;
    SOFTISP_OK
}

/// Set the pipeline downscale target on a custom profile.
///
/// # Safety
///
/// `profile` must be non-NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_set_pipeline_downscale_target(
    profile: *mut SoftispProfile,
    max_px: u32,
) -> i32 {
    if profile.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }
    (*profile).pipeline_downscale_target = max_px;
    SOFTISP_OK
}

/// Set tiled rendering parameters on a custom profile.
///
/// # Safety
///
/// `profile` must be non-NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_profile_set_tiling(
    profile: *mut SoftispProfile,
    tile_count_x: u32,
    tile_count_y: u32,
    overlap: u32,
) -> i32 {
    if profile.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }
    let p = &mut *profile;
    p.use_tiled_rendering = tile_count_x > 0 && tile_count_y > 0;
    p.tile_count_x = tile_count_x;
    p.tile_count_y = tile_count_y;
    p.tile_overlap = overlap;
    SOFTISP_OK
}

// ── C ABI: composition with custom profile ────────────────────────────────

/// Compose an ONNX model using a custom pipeline profile.
///
/// # Safety
///
/// `profile` must be non-NULL. `out_bytes` and `out_len` must be non-NULL.
#[no_mangle]
pub unsafe extern "C" fn softisp_compose_onnx_with_profile(
    width: i32,
    height: i32,
    profile: *const SoftispProfile,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if profile.is_null() || out_bytes.is_null() || out_len.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    if width <= 0 || height <= 0 {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let w = width as u32;
    let h = height as u32;
    let custom = *profile;
    let pipeline_profile: PipelineProfile = custom.into();

    let force_mode = GLOBAL_OPSET_MODE.lock().unwrap().clone();

    let onnx = match compose_pipeline_with_mode(w, h, pipeline_profile, force_mode) {
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

/// Compose an ONNX→MNN model using a custom pipeline profile.
///
/// Requires the `mnn` feature at build time.
///
/// # Safety
///
/// Same as softisp_compose_onnx_with_profile.
#[cfg(feature = "mnn")]
#[no_mangle]
pub unsafe extern "C" fn softisp_compose_mnn_with_profile(
    width: i32,
    height: i32,
    profile: *const SoftispProfile,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if profile.is_null() || out_bytes.is_null() || out_len.is_null() {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    if width <= 0 || height <= 0 {
        return SOFTISP_ERR_INVALID_INPUT;
    }

    let w = width as u32;
    let h = height as u32;
    let custom = *profile;
    let pipeline_profile: PipelineProfile = custom.into();

    let force_mode = GLOBAL_OPSET_MODE.lock().unwrap().clone();

    let onnx = match compose_pipeline_with_mode(w, h, pipeline_profile, force_mode) {
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

// ── C ABI: existing composition ───────────────────────────────────────────

/// Resolve a profile string to a PipelineProfile.
fn parse_profile_builtin(s: &str) -> Option<PipelineProfile> {
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
    let profile_enum = match parse_profile_builtin(profile_str) {
        Some(p) => p,
        None => return SOFTISP_ERR_UNKNOWN_PROFILE,
    };

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
    let profile_enum = match parse_profile_builtin(profile_str) {
        Some(p) => p,
        None => return SOFTISP_ERR_UNKNOWN_PROFILE,
    };

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

// ── C ABI: opset mode ─────────────────────────────────────────────────────

/// Set the global opset mode for ONNX emission.
///
/// This affects all subsequent softisp_compose_* calls until changed or cleared.
///
/// # Safety
///
/// `mode` must be one of the `SOFTISP_OPSET_*` constants.
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
/// After calling this, blocks use their own `opset_mode()` again.
#[no_mangle]
pub unsafe extern "C" fn softisp_clear_opset_mode() -> i32 {
    let mut guard = GLOBAL_OPSET_MODE.lock().unwrap();
    *guard = None;
    SOFTISP_OK
}

// ── C ABI: memory & info ──────────────────────────────────────────────────

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

// ── Tests ─────────────────────────────────────────────────────────────────

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
        assert_eq!(unsafe { softisp_set_opset_mode(SOFTISP_OPSET_CUSTOM) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_set_opset_mode(SOFTISP_OPSET_PRIMITIVE) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_set_opset_mode(99) }, SOFTISP_ERR_INVALID_INPUT);
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

    #[test]
    fn test_cabi_custom_profile_create_free() {
        let p = softisp_profile_create();
        assert!(!p.is_null());
        unsafe { softisp_profile_free(p) };
    }

    #[test]
    fn test_cabi_custom_profile_set_flag() {
        let p = softisp_profile_create();
        assert!(!p.is_null());

        let flag = std::ffi::CString::new("use_fcs").unwrap();
        assert_eq!(unsafe { softisp_profile_set_flag(p, flag.as_ptr(), 1) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_profile_set_flag(p, flag.as_ptr(), 0) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_profile_set_flag(p, flag.as_ptr(), 0) }, SOFTISP_OK);

        let bad = std::ffi::CString::new("no_such_flag").unwrap();
        assert_eq!(unsafe { softisp_profile_set_flag(p, bad.as_ptr(), 1) }, SOFTISP_ERR_INVALID_INPUT);

        unsafe { softisp_profile_free(p) };
    }

    #[test]
    fn test_cabi_custom_profile_setters() {
        let p = softisp_profile_create();
        assert!(!p.is_null());

        assert_eq!(unsafe { softisp_profile_set_demosaic_quality(p, SOFTISP_DEMOSAIC_EDGE) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_profile_set_rotate_mode(p, 2) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_profile_set_output_format(p, SOFTISP_OUTPUT_RGB) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_profile_set_eis_margin(p, 0.05) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_profile_set_stats_downscale_max(p, 1080) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_profile_set_pipeline_downscale_target(p, 1920) }, SOFTISP_OK);
        assert_eq!(unsafe { softisp_profile_set_tiling(p, 2, 2, 2) }, SOFTISP_OK);

        // Invalid values
        assert_eq!(unsafe { softisp_profile_set_demosaic_quality(p, 99) }, SOFTISP_ERR_INVALID_INPUT);
        assert_eq!(unsafe { softisp_profile_set_rotate_mode(p, 99) }, SOFTISP_ERR_INVALID_INPUT);
        assert_eq!(unsafe { softisp_profile_set_output_format(p, 99) }, SOFTISP_ERR_INVALID_INPUT);

        unsafe { softisp_profile_free(p) };
    }

    #[test]
    fn test_cabi_compose_with_custom_profile() {
        let p = softisp_profile_create();
        assert!(!p.is_null());

        let flag = std::ffi::CString::new("use_fcs").unwrap();
        unsafe { softisp_profile_set_flag(p, flag.as_ptr(), 1) };

        let mut out_bytes: *mut u8 = std::ptr::null_mut();
        let mut out_len: usize = 0;
        let rc = unsafe {
            softisp_compose_onnx_with_profile(640, 480, p, &mut out_bytes, &mut out_len)
        };
        assert_eq!(rc, SOFTISP_OK);
        assert!(!out_bytes.is_null());
        assert!(out_len > 0);
        unsafe { softisp_free(out_bytes, out_len) };

        unsafe { softisp_profile_free(p) };
    }

    #[test]
    fn test_cabi_compose_with_profile_null_checks() {
        let rc = unsafe {
            softisp_compose_onnx_with_profile(
                640,
                480,
                std::ptr::null(),
                &mut std::ptr::null_mut(),
                &mut 0usize,
            )
        };
        assert_eq!(rc, SOFTISP_ERR_INVALID_INPUT);
    }
}
