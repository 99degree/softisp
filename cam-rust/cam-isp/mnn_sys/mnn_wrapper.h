/**
 * C wrapper around MNN C++ API for use with Rust FFI.
 *
 * Exposes a minimal set of functions needed for ISP pipeline inference:
 *   - Load model from buffer
 *   - Create session with configurable backend
 *   - Resize session for input dimensions
 *   - Run inference
 *   - Get/set tensor data
 *
 * Compile with Android NDK:
 *   ${CROSS_COMPILE}clang++ -std=c++17 -c mnn_wrapper.cpp \
 *     -I${MNN_INCLUDE} -o mnn_wrapper.o
 *   Then link with -lMNN at build time.
 */

#ifndef MNN_WRAPPER_H
#define MNN_WRAPPER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Forward types matching MNNForwardType.h
 */
typedef enum {
    MNN_BACKEND_CPU = 0,
    MNN_BACKEND_OPENCL = 3,
    MNN_BACKEND_OPENGL = 6,
    MNN_BACKEND_VULKAN = 7,
    MNN_BACKEND_METAL = 9,
    MNN_BACKEND_NN = 11,
} MnnBackendType;

/**
 * MNN data types (matching MNN::Tensor::getType().code)
 */
#define MNN_DATA_TYPE_FLOAT  1
#define MNN_DATA_TYPE_INT32 3
#define MNN_DATA_TYPE_UINT8 5
#define MNN_DATA_TYPE_INT16 6
#define MNN_DATA_TYPE_INT64 8

/**
 * Opaque handles for MNN objects.
 */
typedef void* MnnInterpreter;
typedef void* MnnSession;
typedef void* MnnTensor;

/**
 * Create an Interpreter from a model file (.mnn format).
 * @param path    Path to the .mnn model file.
 * @return Opaque handle, or NULL on failure.
 */
MnnInterpreter mnn_interpreter_create_from_file(const char* path);

/**
 * Create an Interpreter from a model buffer.
 * @param buffer  Pointer to the serialized MNN model data.
 * @param size    Size of the buffer in bytes.
 * @return Opaque handle, or NULL on failure.
 */
MnnInterpreter mnn_interpreter_create_from_buffer(const void* buffer, size_t size);

/**
 * Destroy an Interpreter.
 */
void mnn_interpreter_destroy(MnnInterpreter interpreter);

/**
 * Create a Session from an Interpreter.
 * @param interpreter   Opaque handle from mnn_interpreter_create_from_buffer.
 * @param backend       Desired backend (MNN_BACKEND_*).
 * @param num_threads   Number of CPU threads (used for CPU backend).
 * @return Opaque handle, or NULL on failure.
 */
MnnSession mnn_session_create(MnnInterpreter interpreter, MnnBackendType backend, int num_threads);

/**
 * Release a Session.
 * @param session  Opaque handle from mnn_session_create.
 */
void mnn_session_release(MnnInterpreter interpreter, MnnSession session);

/**
 * Resize the session after setting input tensor dimensions.
 * Must be called before running, and after any shape changes.
 */
int mnn_session_resize(MnnInterpreter interpreter, MnnSession session);

/**
 * Run the session (forward inference).
 * @return 0 on success, non-zero on failure.
 */
int mnn_session_run(MnnInterpreter interpreter, MnnSession session);

/**
 * Get an input tensor by name (deprecated - use V2).
 * @param session  Session handle.
 * @param name     Tensor name (can be NULL for first input).
 * @return Opaque tensor handle, or NULL.
 */
MnnTensor mnn_session_get_input(MnnSession session, const char* name);

/**
 * Get an output tensor by name (deprecated - use V2).
 * @param session  Session handle.
 * @param name     Tensor name (can be NULL for first output).
 * @return Opaque tensor handle, or NULL.
 */
MnnTensor mnn_session_get_output(MnnSession session, const char* name);

/**
 * Get an input tensor by name.
 * @param interpreter Interpreter handle.
 * @param session  Session handle.
 * @param name     Tensor name (can be NULL for first input).
 * @return Opaque tensor handle, or NULL.
 */
MnnTensor mnn_session_get_input_v2(MnnInterpreter interpreter, MnnSession session, const char* name);

/**
 * Get an output tensor by name.
 * @param interpreter Interpreter handle.
 * @param session  Session handle.
 * @param name     Tensor name (can be NULL for first output).
 * @return Opaque tensor handle, or NULL.
 */
MnnTensor mnn_session_get_output_v2(MnnInterpreter interpreter, MnnSession session, const char* name);

/**
 * Get the shape of a tensor.
 * @param tensor   Tensor handle.
 * @param dims     Output buffer for dimensions (at least 4 ints).
 * @return Number of dimensions written.
 */
int mnn_tensor_get_shape(MnnTensor tensor, int* dims, int max_dims);

/**
 * Get the element type of a tensor (4=float32, 5=int32, etc.).
 */
int mnn_tensor_get_type(MnnTensor tensor);

/**
 * Get a mutable pointer to the tensor's host data.
 * For CPU tensors, this returns a pointer to the data buffer.
 * @param tensor  Tensor handle.
 * @return Pointer to data, or NULL if not host-readable.
 */
float* mnn_tensor_get_host_data(MnnTensor tensor);

/**
 * Set the shape of a tensor (requires interpreter for resizeTensor).
 * After setting shape, call mnn_session_resize to apply.
 * @param interpreter Interpreter handle.
 * @param session Session handle.
 * @param tensor  Tensor handle.
 * @param dims    Dimensions array.
 * @param ndim    Number of dimensions.
 * @return 0 on success.
 */
int mnn_tensor_set_shape(MnnInterpreter interpreter, MnnSession session, MnnTensor tensor, const int* dims, int ndim);

/**
 * Get raw host data pointer (void*) for any tensor element type.
 * The pointer is valid until the tensor is destroyed or resized.
 * Use mnn_tensor_get_type() to interpret the data correctly.
 * @param tensor Tensor handle.
 * @return Pointer to data, or NULL if not host-accessible.
 */
void* mnn_tensor_get_host_data_raw(MnnTensor tensor);

/**
 * Get the size in bytes of the tensor's data buffer.
 * @param tensor Tensor handle.
 * @return Size in bytes, or 0 on error.
 */
size_t mnn_tensor_get_data_size(MnnTensor tensor);

/**
 * Get the model buffer for saving in MNN format.
 * After loading an ONNX model via mnn_interpreter_create_from_buffer,
 * this returns the internal MNN format buffer.
 * @param interpreter  Interpreter handle.
 * @param out_size     Output parameter for buffer size.
 * @return Pointer to model buffer, or NULL on failure.
 *         The pointer is valid until interpreter is destroyed.
 */
const void* mnn_interpreter_get_model_buffer(MnnInterpreter interpreter, size_t* out_size);

/**
 * Save model buffer to file.
 * Convenience function to write the model buffer to disk.
 * @param interpreter  Interpreter handle.
 * @param path         File path to save.
 * @return 0 on success, non-zero on failure.
 */
int mnn_interpreter_save_model(MnnInterpreter interpreter, const char* path);

/**
 * Run inference with proper host-tensor management.
 * Creates host tensors with copyFromHostTensor / copyToHostTensor.
 * @param interpreter Interpreter handle.
 * @param session Session handle.
 * @param in_data Float32 input data.
 * @param in_shape Input shape array.
 * @param in_ndim Number of dimensions.
 * @param out_data Output buffer (float32).
 * @param max_out Max number of output elements.
 * @return Number of output elements written, or -1 on error.
 */
int mnn_run_host_tensors(MnnInterpreter interpreter, MnnSession session,
                          const float* in_data, const int* in_shape, int in_ndim,
                          float* out_data, int max_out);

/**
 * Zero-copy inference: wrap existing buffer directly as host tensor.
 * If buffer type matches model input type, no alloc/copy for input.
 * @param interpreter Interpreter handle.
 * @param session Session handle.
 * @param buffer      Pointer to existing frame buffer.
 * @param buffer_type_code Halide type code: 0=INT, 1=UINT, 2=FLOAT.
 * @param buffer_type_bits Bit width: 8, 16, 32.
 * @param in_shape    Input shape array.
 * @param in_ndim     Number of dimensions.
 * @param out_data    Output buffer (float32).
 * @param max_out     Max number of output elements.
 * @return Number of output elements written, or -1 on error.
 */
int mnn_run_with_buffer(MnnInterpreter interpreter, MnnSession session,
                          const void* buffer, int buffer_type_code, int buffer_type_bits,
                          const int* in_shape, int in_ndim,
                          float* out_data, int max_out);

// ── Zero-copy inference ────────────────────────────────────────────────
extern "C" int mnn_run_zero_copy(
    MnnInterpreter interpreter,
    MnnSession session,
    const void* buffer,
    const int* in_shape,
    int in_ndim,
    float* out_data,
    int max_out
);

// ── Get model input type ──────────────────────────────────────────────────
extern "C" int mnn_get_model_input_type(MnnInterpreter interpreter, MnnSession session,
    int* out_code, int* out_bits);

// ── True zero-copy inference ──────────────────────────────────────────────────
extern "C" int mnn_run_true_zero_copy(
    MnnInterpreter interpreter,
    MnnSession session,
    const void* buffer,
    int buffer_type_code,
    int buffer_type_bits,
    const int* in_shape,
    int in_ndim,
    float* out_data,
    int max_out
);

// ── Express Module API ──────────────────────────────────────────────────

typedef void* MnnExpressModule;
typedef void* MnnVARP;

MnnVARP* mnn_express_load_vars(const char* path, int* out_count);
MnnExpressModule mnn_express_extract(MnnVARP* inputs, int n_inputs, MnnVARP* outputs, int n_outputs);
void mnn_express_destroy_module(MnnExpressModule module);
MnnVARP mnn_express_create_input(const int* dims, int ndim, int format, int dtype);
void* mnn_express_write_map(MnnVARP varp);
const void* mnn_express_read_map(MnnVARP varp);
int mnn_express_var_info(MnnVARP varp, int* dims_out, int max_dims, int* out_format);
int mnn_express_var_resize(MnnVARP varp, const int* dims, int ndim);
MnnVARP* mnn_express_forward(MnnExpressModule module, MnnVARP* inputs, int n_inputs, int* out_count);
void mnn_varps_destroy(MnnVARP* varps, int count);

// Model info enum
typedef enum {
    MNN_MODEL_INFO_MEMORY = 0,
    MNN_MODEL_INFO_FLOPS = 1,
    MNN_MODEL_INFO_BACKENDS = 2,
    MNN_MODEL_INFO_RESIZE_STATUS = 3,
    MNN_MODEL_INFO_THREAD_NUMBER = 4,
} MnnModelInfo;

/**
 * Get model info (MEMORY, FLOPS, etc.) for a session.
 * @param interpreter Opaque handle from mnn_interpreter_create_from_file.
 * @param session Opaque handle from mnn_session_create.
 * @param info_code One of MNN_MODEL_INFO_*.
 * @param out Pointer to store result (float for MEMORY/FLOPS, int for others).
 * @return 0 on success, non-zero on failure.
 */
int mnn_get_model_info(MnnInterpreter interpreter, MnnSession session, MnnModelInfo info_code, void* out);

// Profiler API (requires MNN_PIPELINE_PROFILE=ON)
typedef void* MnnExecutor;
MnnExecutor mnn_executor_get_global();
void mnn_executor_reset_profile(MnnExecutor executor);
void mnn_executor_dump_profile(MnnExecutor executor);
float mnn_executor_get_last_gpu_time_ms(MnnExecutor executor);
#ifdef __cplusplus
}
#endif

#endif /* MNN_WRAPPER_H */
