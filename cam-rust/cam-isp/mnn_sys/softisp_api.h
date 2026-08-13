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

// ── Pipeline composition ─────────────────────────────────────────────────

/**
 * Compose an ONNX model for the given profile + resolution.
 *
 * @param width     Pipeline width in pixels.
 * @param height    Pipeline height in pixels.
 * @param profile   Profile name ("LITE", "MED", "HEAVY", "PRO", "UNIFIED", "HDR", "TEST").
 * @param out_bytes Receives pointer to malloc'd ONNX model bytes.
 * @param out_len   Receives size of out_bytes in bytes.
 * @return SOFTISP_OK on success, error code on failure.
 *
 * The caller must free the returned buffer with softisp_free().
 */
int softisp_compose_onnx(int width, int height, const char* profile,
                         uint8_t** out_bytes, size_t* out_len);

/**
 * Compose an ONNX→MNN model for the given profile + resolution.
 *
 * Requires the cam-isp library to be built with MNN support.
 *
 * @param width     Pipeline width in pixels.
 * @param height    Pipeline height in pixels.
 * @param profile   Profile name.
 * @param out_bytes Receives pointer to malloc'd MNN model bytes.
 * @param out_len   Receives size of out_bytes in bytes.
 * @return SOFTISP_OK on success, error code on failure.
 *
 * The caller must free the returned buffer with softisp_free().
 */
int softisp_compose_mnn(int width, int height, const char* profile,
                        uint8_t** out_bytes, size_t* out_len);

// ── Custom opset mode ────────────────────────────────────────────────────

/**
 * Set the global opset mode for ONNX emission.
 *
 * This affects all subsequent softisp_compose_* calls until changed or cleared.
 *
 * @param mode  SOFTISP_OPSET_PRIMITIVE (0) or SOFTISP_OPSET_CUSTOM (1).
 * @return SOFTISP_OK on success, SOFTISP_ERR_INVALID_INPUT if mode is invalid.
 */
int softisp_set_opset_mode(int mode);

/**
 * Clear the global opset mode override.
 *
 * After calling this, blocks use their own opset_mode() again.
 *
 * @return SOFTISP_OK.
 */
int softisp_clear_opset_mode(void);

// ── Memory management ───────────────────────────────────────────────────

/**
 * Free a buffer returned by softisp_compose_onnx or softisp_compose_mnn.
 *
 * @param bytes  Pointer returned by compose function.
 * @param len    Length returned by compose function.
 */
void softisp_free(uint8_t* bytes, size_t len);

// ── Library info ─────────────────────────────────────────────────────────

/**
 * Get the version string of the cam-isp library.
 * @return Static version string (do not free).
 */
const char* softisp_version(void);

/**
 * Get a human-readable error string for an error code.
 * @param code  Error code returned by a softisp function.
 * @return Static error string (do not free).
 */
const char* softisp_error_string(int code);

#ifdef __cplusplus
}
#endif

#endif /* SOFTISP_API_H */
