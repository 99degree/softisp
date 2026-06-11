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
 * Opaque handles for MNN objects.
 */
typedef void* MnnInterpreter;
typedef void* MnnSession;
typedef void* MnnTensor;

/**
 * Create an Interpreter from a model buffer.
 * @param buffer  Pointer to the serialized MNN model (or ONNX model) data.
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
 * Get an input tensor by name.
 * @param session  Session handle.
 * @param name     Tensor name (can be NULL for first input).
 * @return Opaque tensor handle, or NULL.
 */
MnnTensor mnn_session_get_input(MnnSession session, const char* name);

/**
 * Get an output tensor by name.
 * @param session  Session handle.
 * @param name     Tensor name (can be NULL for first output).
 * @return Opaque tensor handle, or NULL.
 */
MnnTensor mnn_session_get_output(MnnSession session, const char* name);

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
 * Set the shape of a host tensor (for creating input tensors with specific dims).
 * @param tensor Tensor handle.
 * @param dims   Dimensions array.
 * @param ndim   Number of dimensions.
 * @return 0 on success.
 */
int mnn_tensor_set_shape(MnnTensor tensor, const int* dims, int ndim);

#ifdef __cplusplus
}
#endif

#endif /* MNN_WRAPPER_H */
