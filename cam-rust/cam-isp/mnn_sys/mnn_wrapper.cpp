/**
 * C wrapper implementation around MNN C++ API.
 *
 * Compile with Android NDK:
 *   ${CROSS_COMPILE}clang++ -std=c++17 -c mnn_wrapper.cpp \
 *     -I${MNN_INCLUDE_DIR} -o mnn_wrapper.o
 *
 * Then link against libMNN.so when building the final shared library.
 */

#include "mnn_wrapper.h"
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <MNN/MNNForwardType.h>
#include <cstring>

// ── Interpreter ──────────────────────────────────────────────────────────

MnnInterpreter mnn_interpreter_create_from_file(const char* path) {
    if (!path) return nullptr;
    auto* interpreter = MNN::Interpreter::createFromFile(path);
    return reinterpret_cast<MnnInterpreter>(interpreter);
}

MnnInterpreter mnn_interpreter_create_from_buffer(const void* buffer, size_t size) {
    auto* interpreter = MNN::Interpreter::createFromBuffer(buffer, size);
    return reinterpret_cast<MnnInterpreter>(interpreter);
}

void mnn_interpreter_destroy(MnnInterpreter interpreter) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    if (net) {
        MNN::Interpreter::destroy(net);
    }
}

// ── Session ──────────────────────────────────────────────────────────────

MnnSession mnn_session_create(MnnInterpreter interpreter, MnnBackendType backend, int num_threads) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    if (!net) return nullptr;

    MNN::ScheduleConfig config;
    config.numThread = num_threads;

    // Map our backend enum to MNNForwardType
    switch (backend) {
        case MNN_BACKEND_CPU:    config.type = MNN_FORWARD_CPU; break;
        case MNN_BACKEND_OPENCL: config.type = MNN_FORWARD_OPENCL; break;
        case MNN_BACKEND_VULKAN: config.type = MNN_FORWARD_VULKAN; break;
        case MNN_BACKEND_METAL:  config.type = MNN_FORWARD_METAL; break;
        case MNN_BACKEND_NN:     config.type = MNN_FORWARD_NN; break;
        default:                 config.type = MNN_FORWARD_CPU; break;
    }
    
    // Use high precision for Vulkan backend to ensure correct ISP output
    MNN::BackendConfig backendConfig;
    backendConfig.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &backendConfig;

    auto* session = net->createSession(config);
    return reinterpret_cast<MnnSession>(session);
}

void mnn_session_release(MnnInterpreter interpreter, MnnSession session) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (net && sess) {
        net->releaseSession(sess);
    }
}

int mnn_session_resize(MnnInterpreter interpreter, MnnSession session) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return -1;

    net->resizeSession(sess);
    return 0;
}

int mnn_session_run(MnnInterpreter interpreter, MnnSession session) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return -1;

    auto error = net->runSession(sess);
    return static_cast<int>(error);
}

// ── Tensor ───────────────────────────────────────────────────────────────

MnnTensor mnn_session_get_input(MnnSession session, const char* /*name*/) {
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!sess) return nullptr;

    // We need the interpreter to call getSessionInput, but the Interpreter
    // owns the session. The Rust caller must manage this.
    // For this wrapper, we provide a helper that takes the interpreter explicitly.
    return nullptr; // Use the variant below
}

MnnTensor mnn_session_get_output(MnnSession session, const char* name) {
    (void)session;
    (void)name;
    return nullptr;
}

// Better variants that take the interpreter:
MnnTensor mnn_session_get_input_v2(MnnInterpreter interpreter, MnnSession session, const char* name) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return nullptr;
    auto* tensor = net->getSessionInput(sess, name);
    return reinterpret_cast<MnnTensor>(tensor);
}

MnnTensor mnn_session_get_output_v2(MnnInterpreter interpreter, MnnSession session, const char* name) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return nullptr;
    auto* tensor = net->getSessionOutput(sess, name);
    return reinterpret_cast<MnnTensor>(tensor);
}

// ── Tensor data access ───────────────────────────────────────────────────

int mnn_tensor_get_shape(MnnTensor tensor, int* dims, int max_dims) {
    auto* t = reinterpret_cast<MNN::Tensor*>(tensor);
    if (!t || !dims) return 0;

    auto shape = t->shape();
    int n = static_cast<int>(shape.size());
    if (n > max_dims) n = max_dims;
    for (int i = 0; i < n; i++) {
        dims[i] = shape[i];
    }
    return n;
}

int mnn_tensor_get_type(MnnTensor tensor) {
    auto* t = reinterpret_cast<MNN::Tensor*>(tensor);
    if (!t) return 0;
    // MNN::Tensor::getType() returns halide_type_t
    auto type = t->getType();
    if (type.code == halide_type_float) return 4;    // float32
    if (type.code == halide_type_int) return 5;       // int32
    if (type.code == halide_type_uint) return 6;      // uint32
    return 0;
}

float* mnn_tensor_get_host_data(MnnTensor tensor) {
    auto* t = reinterpret_cast<MNN::Tensor*>(tensor);
    if (!t) return nullptr;
    return t->host<float>();
}

int mnn_tensor_set_shape(MnnInterpreter interpreter, MnnSession /*session*/, MnnTensor tensor, const int* dims, int ndim) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* t = reinterpret_cast<MNN::Tensor*>(tensor);
    if (!net || !t || !dims) return -1;
    std::vector<int> shape(dims, dims + ndim);
    net->resizeTensor(t, shape);
    return 0;
}

// ── Raw data access ───────────────────────────────────────────────────────

void* mnn_tensor_get_host_data_raw(MnnTensor tensor) {
    auto* t = reinterpret_cast<MNN::Tensor*>(tensor);
    if (!t) return nullptr;
    // MNN::Tensor::host<T>() returns T*
    // We can just return the buffer pointer as void*
    // Note: This works because host<float>() etc. all return the same underlying pointer
    // But we need to actually call a template. Let's use reinterpret_cast hack.
    // The MNN Tensor stores data in a Buffer. We can access via t->host<void>()
    // However, MNN doesn't have host<void>(). We'll use the float pointer and cast.
    // This is safe because the buffer is contiguous.
    return static_cast<void*>(t->host<float>());
}

size_t mnn_tensor_get_data_size(MnnTensor tensor) {
    auto* t = reinterpret_cast<MNN::Tensor*>(tensor);
    if (!t) return 0;
    auto shape = t->shape();
    size_t element_count = 1;
    for (auto d : shape) {
        element_count *= static_cast<size_t>(d);
    }
    auto type = t->getType();
    size_t element_size = 4; // default to float32
    if (type.code == halide_type_float) element_size = 4;
    else if (type.code == halide_type_int) element_size = 4; // int32
    else if (type.code == halide_type_uint) element_size = 1; // uint8
    else if (type.code == halide_type_int && type.bits == 16) element_size = 2; // int16
    else if (type.code == halide_type_int && type.bits == 64) element_size = 8; // int64
    return element_count * element_size;
}

// ── Model buffer / conversion ───────────────────────────────────────────

const void* mnn_interpreter_get_model_buffer(MnnInterpreter interpreter, size_t* out_size) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    if (!net || !out_size) return nullptr;
    
    auto buffer = net->getModelBuffer();
    *out_size = buffer.second;
    return buffer.first;
}

int mnn_interpreter_save_model(MnnInterpreter interpreter, const char* path) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    if (!net || !path) return -1;
    
    auto buffer = net->getModelBuffer();
    if (!buffer.first || buffer.second == 0) return -1;
    
    FILE* f = fopen(path, "wb");
    if (!f) return -1;
    
    size_t written = fwrite(buffer.first, 1, buffer.second, f);
    fclose(f);
    
    return (written == buffer.second) ? 0 : -1;
}

// ── Host-tensor inference (copyFromHostTensor / copyToHostTensor) ──────

extern "C" int mnn_run_host_tensors(
    MnnInterpreter interpreter,
    MnnSession session,
    const float* in_data,
    const int* in_shape,
    int in_ndim,
    float* out_data,
    int max_out
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !in_data || !out_data) return -1;

    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    // Create host tensor wrapping input data as FLOAT32.
    // MNN's copyFromHostTensor handles type conversion to the backend type.
    std::vector<int> host_shape(in_shape, in_shape + in_ndim);
    halide_type_t float_type(halide_type_float, 32);
    auto* host_in = MNN::Tensor::create(
        host_shape,
        float_type,
        const_cast<float*>(in_data),
        MNN::Tensor::CAFFE);

    if (!host_in) return -3;

    // Copy host → backend (handles GPU transfer)
    in_tensor->copyFromHostTensor(host_in);

    // Run inference
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) return -2;

    // Get output tensor
    auto* out_tensor = net->getSessionOutput(sess, nullptr);
    if (!out_tensor) return -1;
    
    auto out_shape = out_tensor->shape();
    
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;

    int n = out_total < max_out ? out_total : max_out;
    if (n <= 0) return -5;

    // Create host tensor wrapping output buffer as FLOAT32.
    halide_type_t out_float_type(halide_type_float, 32);
    auto* host_out = MNN::Tensor::create(
        out_shape,
        out_float_type,
        out_data,
        MNN::Tensor::CAFFE);
    
    if (!host_out) return -3;

    out_tensor->copyToHostTensor(host_out);

    // Clean up host tensor wrappers (actual data buffers are caller-owned)
    delete host_in;
    delete host_out;

    return n;
}

// ── FP16 output inference ────────────────────────────────────────────────
// For models with Cast(FLOAT→FLOAT16) at the end, reads raw float16 output.
// Output buffer must hold at least max_out * sizeof(uint16_t) bytes.
// Returns number of float16 elements written, or negative on error.
extern "C" int mnn_run_host_tensors_fp16(
    MnnInterpreter interpreter,
    MnnSession session,
    const float* in_data,
    const int* in_shape,
    int in_ndim,
    void* out_data,
    int max_out
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !in_data || !out_data) return -1;

    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    std::vector<int> host_shape(in_shape, in_shape + in_ndim);
    halide_type_t float_type(halide_type_float, 32);
    auto* host_in = MNN::Tensor::create(
        host_shape, float_type, const_cast<float*>(in_data), MNN::Tensor::CAFFE);
    if (!host_in) return -3;

    in_tensor->copyFromHostTensor(host_in);
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) { delete host_in; return -2; }

    auto* out_tensor = net->getSessionOutput(sess, nullptr);
    if (!out_tensor) { delete host_in; return -1; }

    auto out_shape = out_tensor->shape();
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    int n = out_total < max_out ? out_total : max_out;
    if (n <= 0) { delete host_in; return -5; }

    // Determine output type — if model outputs FLOAT16, read as-is
    auto out_type = out_tensor->getType();
    if (out_type.code == halide_type_float && out_type.bits == 16) {
        // FP16 output — copy raw bytes
        halide_type_t fp16_type(halide_type_float, 16);
        auto* host_out = MNN::Tensor::create(out_shape, fp16_type, out_data, MNN::Tensor::CAFFE);
        if (!host_out) { delete host_in; return -3; }
        out_tensor->copyToHostTensor(host_out);
        delete host_out;
    } else {
        // FP32 output — copy as float, caller can convert
        halide_type_t fp32_type(halide_type_float, 32);
        auto* host_out = MNN::Tensor::create(out_shape, fp32_type, out_data, MNN::Tensor::CAFFE);
        if (!host_out) { delete host_in; return -3; }
        out_tensor->copyToHostTensor(host_out);
        delete host_out;
    }

    delete host_in;
    return n;
}


// ── Set named input tensor (for multi-input models) ──────────────────────

extern "C" int mnn_set_input_float(
    MnnInterpreter interpreter,
    MnnSession session,
    const char* name,
    const float* data,
    const int* shape,
    int ndim
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return -1;

    auto* tensor = net->getSessionInput(sess, name);
    if (!tensor) return -2;  // tensor not found

    std::vector<int> host_shape(shape, shape + ndim);
    halide_type_t float_type(halide_type_float, 32);
    auto* host = MNN::Tensor::create(host_shape, float_type, const_cast<float*>(data), MNN::Tensor::CAFFE);
    if (!host) return -3;

    tensor->copyFromHostTensor(host);
    delete host;
    return 0;
}


// ── Zero-copy inference: directly set host pointer ──────────────────────
/**
 * Zero-copy inference with direct host pointer assignment.
 * This bypasses copyFromHostTensor which fails when the input tensor has no backend.
 * 
 * For models where copyFromHostTensor fails (e.g., LITE+ profiles with memory optimization),
 * this function directly sets the input tensor's host pointer to the provided buffer.
 * 
 * WARNING: The caller must ensure:
 * 1. The buffer remains valid for the duration of runSession
 * 2. The buffer size matches the expected input size
 * 3. The buffer data type matches the model input type (or use convert path)
 * 4. Proper cache coherency is maintained (use sync functions if needed)
 * 
 * @param interpreter Interpreter handle.
 * @param session Session handle.
 * @param buffer Pointer to existing buffer with input data.
 * @param in_shape Input shape array.
 * @param in_ndim Number of dimensions.
 * @param out_data Output buffer (float32).
 * @param max_out Max number of output elements.
 * @return Number of output elements written, or negative on error.
 */



// ── Get model input type ──────────────────────────────────────────────────

extern "C" int mnn_get_model_input_type(MnnInterpreter interpreter, MnnSession session, int* out_code, int* out_bits) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !out_code || !out_bits) return -1;

    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    auto model_type = in_tensor->getType();
    *out_code = model_type.code;
    *out_bits = model_type.bits;
    return 0;
}

/// Get expected input tensor element count.
/// Returns -1 on error, otherwise the number of elements.
extern "C" int mnn_get_model_input_elements(MnnInterpreter interpreter, MnnSession session) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return -1;
    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;
    return (int)in_tensor->elementSize();
}


// ── True zero-copy inference (direct host pointer) ──────────────────────
/**
 * True zero-copy inference: directly set input tensor's host pointer.
 * No allocation, no copy. The input buffer is used directly.
 * 
 * REQUIREMENTS:
 * - Buffer type must exactly match model input type (code + bits)
 * - Buffer must remain valid for duration of runSession
 * - Buffer size must match expected input size (shape)
 * - Caller must ensure proper cache coherency if needed
 * 
 * @param interpreter Interpreter handle.
 * @param session Session handle.
 * @param buffer Pointer to existing buffer with input data.
 * @param buffer_type_code Halide type code of buffer (0=int, 1=uint, 2=float).
 * @param buffer_type_bits Bit width of buffer (8, 16, 32).
 * @param in_shape Input shape array.
 * @param in_ndim Number of dimensions.
 * @param out_data Output buffer (float32).
 * @param max_out Max number of output elements.
 * @return Number of output elements written, or negative on error.
 */
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
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !buffer || !out_data) return -1;

    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    halide_type_t buffer_type = { (halide_type_code_t)buffer_type_code, (uint8_t)buffer_type_bits, 1 };
    auto model_type = in_tensor->getType();
    
    // Must match exactly for true zero-copy
    if (model_type.code != buffer_type.code || model_type.bits != buffer_type.bits) {
        return -2;  // Type mismatch
    }

    std::vector<int> shape(in_shape, in_shape + in_ndim);
    int total = 1;
    for (int i = 0; i < in_ndim; i++) total *= in_shape[i];

    // Verify tensor size matches
    size_t tensor_size = in_tensor->elementSize();
    if (tensor_size != (size_t)total) {
        // Try to resize session to match our shape
        // Note: This may allocate new memory, breaking zero-copy
        net->resizeSession(sess);
        in_tensor = net->getSessionInput(sess, nullptr);
        if (!in_tensor) return -3;
        tensor_size = in_tensor->elementSize();
        if (tensor_size != (size_t)total) {
            return -4;  // Size mismatch even after resize
        }
    }

    // TRUE ZERO-COPY: Directly set the host pointer
    // This bypasses all allocation and copy
    in_tensor->buffer().host = const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer));
    in_tensor->buffer().device = 0;  // Host memory
    // Note: type and dimensions are assumed to be already set correctly by MNN

    // ZERO-COPY OUTPUT: set host pointer on output tensor BEFORE runSession.
    // This eliminates the copyToHostTensor + memcpy overhead on every frame.
    auto* out_tensor = net->getSessionOutput(sess, nullptr);
    if (!out_tensor) return -1;

    // Set output host buffer directly — zero-copy
    out_tensor->buffer().host = reinterpret_cast<uint8_t*>(out_data);
    out_tensor->buffer().device = 0;

    // Run inference — output is written directly to out_data
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) {
        return static_cast<int>(run_ok);
    }

    // Get element count from resolved output tensor (shape is now valid)
    auto out_shape = out_tensor->shape();
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    int n = out_total < max_out ? out_total : max_out;
    if (n <= 0) return -5;

    return n;
}



/**
 * Run inference and copy a SPECIFIC named output tensor to out_data.
 * Same as mnn_run_true_zero_copy but uses an output tensor name instead
 * of getSessionOutput(nullptr).  Pass nullptr for output_name to get
 * first output (same as mnn_run_true_zero_copy).
 */
extern "C" int mnn_run_with_output(
    MnnInterpreter interpreter,
    MnnSession session,
    const void* buffer,
    int buffer_type_code,
    int buffer_type_bits,
    const int* in_shape,
    int in_ndim,
    const char* output_name,
    float* out_data,
    int max_out
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !buffer || !out_data) return -1;

    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    halide_type_t buffer_type = { (halide_type_code_t)buffer_type_code, (uint8_t)buffer_type_bits, 1 };
    auto model_type = in_tensor->getType();
    
    if (model_type.code != buffer_type.code || model_type.bits != buffer_type.bits) {
        return -2;
    }

    std::vector<int> shape(in_shape, in_shape + in_ndim);
    int total = 1;
    for (int i = 0; i < in_ndim; i++) total *= in_shape[i];

    // Set input host pointer — backend maps to device (zero-copy)
    in_tensor->buffer().host = const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer));
    in_tensor->buffer().device = 0;

    // Resize if input element count doesn't match model
    size_t tensor_size = in_tensor->elementSize();
    if (tensor_size != (size_t)total) {
        net->resizeSession(sess);
        in_tensor = net->getSessionInput(sess, nullptr);
        if (!in_tensor) return -3;
        in_tensor->buffer().host = const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer));
        in_tensor->buffer().device = 0;
        tensor_size = in_tensor->elementSize();
        if (tensor_size != (size_t)total) {
            return -4;
        }
    }

    // Set output host pointer — backend writes directly to this buffer
    auto* out_tensor = (output_name != nullptr)
        ? net->getSessionOutput(sess, output_name)
        : net->getSessionOutput(sess, nullptr);
    // Fallback: if named output not found, try first output
    if (!out_tensor && output_name != nullptr) {
        out_tensor = net->getSessionOutput(sess, nullptr);
    }
    if (!out_tensor) return -1;

    out_tensor->buffer().host = reinterpret_cast<uint8_t*>(out_data);
    out_tensor->buffer().device = 0;

    // Run inference: backend maps host buffers to device via DMA,
    // reads input, executes, writes output directly to host buffer.
    // No copyToHostTensor needed — data is already in out_data.
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) {
        return static_cast<int>(run_ok);
    }

    // Return element count for the output tensor
    auto out_shape = out_tensor->shape();
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    return out_total < max_out ? out_total : max_out;
}

// ── Model Info ──────────────────────────────────────────────────────────────
extern "C" int mnn_get_model_info(MnnInterpreter interpreter, MnnSession session, MnnModelInfo info_code, void* out) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !out) return -1;
    
    MNN::Interpreter::SessionInfoCode code;
    switch (info_code) {
        case MNN_MODEL_INFO_MEMORY:
            code = MNN::Interpreter::MEMORY;
            break;
        case MNN_MODEL_INFO_FLOPS:
            code = MNN::Interpreter::FLOPS;
            break;
        case MNN_MODEL_INFO_BACKENDS:
            code = MNN::Interpreter::BACKENDS;
            break;
        case MNN_MODEL_INFO_RESIZE_STATUS:
            code = MNN::Interpreter::RESIZE_STATUS;
            break;
        case MNN_MODEL_INFO_THREAD_NUMBER:
            code = MNN::Interpreter::THREAD_NUMBER;
            break;
        default:
            return -1;
    }
    
    // Use the standard C++ API: Interpreter::getSessionInfo
    // This is always available in MNN
    if (code == MNN::Interpreter::MEMORY) {
        float memory = 0;
        net->getSessionInfo(sess, code, &memory);
        *reinterpret_cast<float*>(out) = memory;
        return 0;
    } else if (code == MNN::Interpreter::FLOPS) {
        float flops = 0;
        net->getSessionInfo(sess, code, &flops);
        *reinterpret_cast<float*>(out) = flops;
        return 0;
    } else {
        int value = 0;
        net->getSessionInfo(sess, code, &value);
        *reinterpret_cast<int*>(out) = value;
        return 0;
    }
}

