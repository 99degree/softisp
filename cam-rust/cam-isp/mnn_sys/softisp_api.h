/**
 * softisp_api.h — C ABI for the softisp ISP pipeline.
 *
 * Exposes a minimal set of functions callable from C/C++ and Android NDK
 * code (via JNI or direct FFI) for composing ISP pipelines without
 * requiring the full Rust toolchain at the call site.
 *
 * Build:
 *   The functions are compiled into the cam-isp shared library (.so).
 *   Link against libsoftisp.so from your C/C++ code.
 *
 * Example:
 *   uint8_t* model = NULL;
 *   size_t model_len = 0;
 *   int rc = softisp_compose_onnx(1920, 1080, "HEAVY", &model, &model_len);
 *   if (rc == SOFTISP_OK) {
 *       // use model bytes...
 *       softisp_free(model, model_len);
 *   }
 */

#ifndef SOFTISP_API_H
#define SOFTISP_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── Error codes ──────────────────────────────────────────────────────────

#define SOFTISP_OK                   0
#define SOFTISP_ERR_INVALID_INPUT    1
#define SOFTISP_ERR_COMPOSE_FAILED   2
#define SOFTISP_ERR_CONVERT_FAILED   3
#define SOFTISP_ERR_UNKNOWN_PROFILE  4

// ── Opset mode ───────────────────────────────────────────────────────────

#define SOFTISP_OPSET_PRIMITIVE  0  // custom isp::* lowered to primitives (default)
#define SOFTISP_OPSET_CUSTOM     1  // custom isp::* emitted verbatim in ONNX

// ── Demosaic quality ─────────────────────────────────────────────────────

#define SOFTISP_DEMOSAIC_STANDARD  0  // Fast bilinear
#define SOFTISP_DEMOSAIC_HQ_LINEAR 1  // Gradient-based (Malvar 2004)
#define SOFTISP_DEMOSAIC_EDGE      2  // Edge-aware with false color suppression

// ── Output format ────────────────────────────────────────────────────────

#define SOFTISP_OUTPUT_FLOAT_RGB      0   // f32×3 planar
#define SOFTISP_OUTPUT_FLOAT_BGRA     1   // f32×4 planar (bg4a)
#define SOFTISP_OUTPUT_PACKED_RGB     2   // INT32 packed (R<<16|G<<8|B)
#define SOFTISP_OUTPUT_BGRA           3   // u8 BGRA (default)
#define SOFTISP_OUTPUT_RGBA           4   // u8 RGBA
#define SOFTISP_OUTPUT_ARGB           5   // u8 ARGB
#define SOFTISP_OUTPUT_ABGR           6   // u8 ABGR
#define SOFTISP_OUTPUT_RGB            7   // u8 RGB
#define SOFTISP_OUTPUT_BGR            8   // u8 BGR
#define SOFTISP_OUTPUT_FLOAT16_RGB    9   // f16×3 planar
#define SOFTISP_OUTPUT_FLOAT16_BGRA   10  // f16×4 planar

// ── Pipeline profile flags ───────────────────────────────────────────────

// Boolean feature flags for softisp_profile_set_flag().
#define SOFTISP_FLAG_USE_UNPACK          "use_unpack"
#define SOFTISP_FLAG_FORCE_INPUT16       "force_input16"
#define SOFTISP_FLAG_USE_FCS             "use_fcs"
#define SOFTISP_FLAG_USE_LDCI            "use_ldci"
#define SOFTISP_FLAG_USE_EE              "use_ee"
#define SOFTISP_FLAG_USE_BAD_PIXEL       "use_bad_pixel"
#define SOFTISP_FLAG_USE_LOCAL_CONTRAST  "use_local_contrast"
#define SOFTISP_FLAG_USE_UNSHARP         "use_unsharp"
#define SOFTISP_FLAG_USE_LSC             "use_lsc"
#define SOFTISP_FLAG_USE_WARP            "use_warp"
#define SOFTISP_FLAG_USE_HDR             "use_hdr"
#define SOFTISP_FLAG_USE_ZONE_STATS      "use_zone_stats"
#define SOFTISP_FLAG_USE_CHANNEL_MEANS   "use_channel_means"
#define SOFTISP_FLAG_USE_TONE_STATS      "use_tone_stats"
#define SOFTISP_FLAG_USE_HISTOGRAM       "use_histogram"
#define SOFTISP_FLAG_USE_BILATERAL       "use_bilateral"
#define SOFTISP_FLAG_USE_SATURATION      "use_saturation"
#define SOFTISP_FLAG_USE_VIGNETTING      "use_vignetting"
#define SOFTISP_FLAG_USE_COLORSPACE      "use_colorspace"
#define SOFTISP_FLAG_USE_GAMMA           "use_gamma"
#define SOFTISP_FLAG_USE_SHARPEN         "use_sharpen"
#define SOFTISP_FLAG_USE_WAVELET_DENOISE "use_wavelet_denoise"
#define SOFTISP_FLAG_USE_AUTO_CONTRAST   "use_auto_contrast"
#define SOFTISP_FLAG_USE_NORMALIZE       "use_normalize"
#define SOFTISP_FLAG_USE_TILED_RENDERING "use_tiled_rendering"

// ── Opaque custom profile handle ─────────────────────────────────────────

/**
 * Opaque handle to a custom pipeline profile.
 *
 * Created with softisp_profile_create(), destroyed with softisp_profile_free().
 */
typedef struct SoftispProfile SoftispProfile;

// ── Custom profile lifecycle ─────────────────────────────────────────────

/**
 * Create a new custom pipeline profile with default values.
 *
 * @return Pointer to new profile, or NULL on allocation failure.
 */
SoftispProfile* softisp_profile_create(void);

/**
 * Free a custom pipeline profile.
 *
 * @param profile  Profile to free (may be NULL).
 */
void softisp_profile_free(SoftispProfile* profile);

// ── Custom profile configuration ─────────────────────────────────────────

/**
 * Set a boolean feature flag on a custom profile.
 *
 * @param profile  Profile handle.
 * @param flag     Flag name (see SOFTISP_FLAG_* defines).
 * @param value    0 for false, non-zero for true.
 * @return SOFTISP_OK on success, SOFTISP_ERR_INVALID_INPUT on bad args.
 */
int softisp_profile_set_flag(SoftispProfile* profile, const char* flag, int value);

/**
 * Set the demosaic quality on a custom profile.
 *
 * @param profile      Profile handle.
 * @param quality      One of SOFTISP_DEMOSAIC_* values.
 * @return SOFTISP_OK on success, SOFTISP_ERR_INVALID_INPUT on bad args.
 */
int softisp_profile_set_demosaic_quality(SoftispProfile* profile, int quality);

/**
 * Set the rotation mode on a custom profile.
 *
 * @param profile  Profile handle.
 * @param mode     0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip.
 * @return SOFTISP_OK on success, SOFTISP_ERR_INVALID_INPUT on bad args.
 */
int softisp_profile_set_rotate_mode(SoftispProfile* profile, int mode);

/**
 * Set the output pixel format on a custom profile.
 *
 * @param profile  Profile handle.
 * @param format   One of SOFTISP_OUTPUT_* values.
 * @return SOFTISP_OK on success, SOFTISP_ERR_INVALID_INPUT on bad args.
 */
int softisp_profile_set_output_format(SoftispProfile* profile, int format);

/**
 * Set the EIS margin fraction on a custom profile.
 *
 * @param profile  Profile handle.
 * @param margin   Margin as a fraction (e.g., 0.05 = 5%).
 * @return SOFTISP_OK.
 */
int softisp_profile_set_eis_margin(SoftispProfile* profile, double margin);

/**
 * Set the stats downscale max dimension on a custom profile.
 *
 * @param profile  Profile handle.
 * @param max_px   Maximum pixel dimension (0 = full resolution).
 * @return SOFTISP_OK.
 */
int softisp_profile_set_stats_downscale_max(SoftispProfile* profile, uint32_t max_px);

/**
 * Set the pipeline downscale target on a custom profile.
 *
 * @param profile  Profile handle.
 * @param max_px   Maximum pixel dimension (0 = full resolution).
 * @return SOFTISP_OK.
 */
int softisp_profile_set_pipeline_downscale_target(SoftispProfile* profile, uint32_t max_px);

/**
 * Set tiled rendering parameters on a custom profile.
 *
 * @param profile      Profile handle.
 * @param tile_count_x Horizontal tile count (0 to disable).
 * @param tile_count_y Vertical tile count (0 to disable).
 * @param overlap      Overlap pixels between tiles.
 * @return SOFTISP_OK.
 */
int softisp_profile_set_tiling(SoftispProfile* profile,
                               uint32_t tile_count_x, uint32_t tile_count_y,
                               uint32_t overlap);

// ── Pipeline composition with custom profile ──────────────────────────────

/**
 * Compose an ONNX model using a custom pipeline profile.
 *
 * @param width     Pipeline width in pixels.
 * @param height    Pipeline height in pixels.
 * @param profile   Custom profile handle (must not be NULL).
 * @param out_bytes Receives pointer to malloc'd ONNX model bytes.
 * @param out_len   Receives size of out_bytes in bytes.
 * @return SOFTISP_OK on success, error code on failure.
 *
 * The caller must free the returned buffer with softisp_free().
 */
int softisp_compose_onnx_with_profile(int width, int height,
                                      const SoftispProfile* profile,
                                      uint8_t** out_bytes, size_t* out_len);

/**
 * Compose an ONNX→MNN model using a custom pipeline profile.
 *
 * Requires the cam-isp library to be built with MNN support.
 *
 * @param width     Pipeline width in pixels.
 * @param height    Pipeline height in pixels.
 * @param profile   Custom profile handle (must not be NULL).
 * @param out_bytes Receives pointer to malloc'd MNN model bytes.
 * @param out_len   Receives size of out_bytes in bytes.
 * @return SOFTISP_OK on success, error code on failure.
 *
 * The caller must free the returned buffer with softisp_free().
 */
int softisp_compose_mnn_with_profile(int width, int height,
                                     const SoftispProfile* profile,
                                     uint8_t** out_bytes, size_t* out_len);

// ── Pipeline composition (existing) ──────────────────────────────────────

/**
 * Compose an ONNX model for a built-in profile.
 *
 * @param width     Pipeline width in pixels.
 * @param height    Pipeline height in pixels.
 * @param profile   Built-in profile name ("LITE", "MED", "HEAVY", "PRO", etc.).
 * @param out_bytes Receives pointer to malloc'd ONNX model bytes.
 * @param out_len   Receives size of out_bytes in bytes.
 * @return SOFTISP_OK on success, error code on failure.
 */
int softisp_compose_onnx(int width, int height, const char* profile,
                         uint8_t** out_bytes, size_t* out_len);

/**
 * Compose an ONNX→MNN model for a built-in profile.
 *
 * @param width     Pipeline width in pixels.
 * @param height    Pipeline height in pixels.
 * @param profile   Built-in profile name.
 * @param out_bytes Receives pointer to malloc'd MNN model bytes.
 * @param out_len   Receives size of out_bytes in bytes.
 * @return SOFTISP_OK on success, error code on failure.
 */
int softisp_compose_mnn(int width, int height, const char* profile,
                        uint8_t** out_bytes, size_t* out_len);

// ── Custom opset mode ────────────────────────────────────────────────────

int softisp_set_opset_mode(int mode);
int softisp_clear_opset_mode(void);

// ── Memory management ───────────────────────────────────────────────────

void softisp_free(uint8_t* bytes, size_t len);

// ── Library info ─────────────────────────────────────────────────────────

const char* softisp_version(void);
const char* softisp_error_string(int code);

#ifdef __cplusplus
}
#endif

#endif /* SOFTISP_API_H */
