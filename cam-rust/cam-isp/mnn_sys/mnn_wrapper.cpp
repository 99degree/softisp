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
#include <algorithm>
#include <cstring>
#include <cstdint>
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
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !buffer || !out_data) return -1;

    // Zero-copy INPUT: bind the caller's `buffer` as the session input tensor's
    // host memory. MNN reads from here directly instead of copying.
    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    // Type sanity check: if the caller declares the buffer's type, it must match
    // the model's input type, otherwise we would silently corrupt data.
    if (buffer_type_bits > 0 && in_tensor->getType().bits != buffer_type_bits) return -1;
    if (buffer_type_code > 0 && in_tensor->getType().code != buffer_type_code) return -1;

    // Optional dynamic input shape. Must happen before binding hosts, because
    // resizeTensor may reallocate the tensors.
    if (in_shape && in_ndim > 0) {
        std::vector<int> dims(in_shape, in_shape + in_ndim);
        net->resizeTensor(in_tensor, dims);
    }
    in_tensor->buffer().host = const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer));

    // Zero-copy OUTPUT: bind the caller's `out_data` as the session output
    // tensor's host. MNN writes the result here directly (no memcpy).
    auto* output = net->getSessionOutput(sess, nullptr);
    if (!output) return -1;
    if (max_out > 0 && (int)output->elementSize() > max_out) return -1; // would overrun out_data
    output->buffer().host = reinterpret_cast<uint8_t*>(out_data);

    // Run in place: input read from `buffer`, output written to `out_data`.
    net->runSession(sess);
    return 0;
}

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


/**
 * Run inference and copy a SPECIFIC named output tensor to out_data.
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
    fprintf(stderr, "[mnn_run_with_output] output '%s' shape=[", output_name ? output_name : "<null>");
    for (int i = 0; i < (int)out_shape.size(); i++) {
        fprintf(stderr, "%s%d", i > 0 ? "," : "", out_shape[i]);
    }
    fprintf(stderr, "] dims=%d\n", (int)out_shape.size());
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    return out_total < max_out ? out_total : max_out;
}

extern "C" int mnn_run_external_zero_copy(
    MnnInterpreter interpreter,
    MnnSession session,
    int64_t ext_handle,
    int memory_type,
    const int* in_shape,
    int in_ndim,
    float* out_data,
    int max_out
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || ext_handle == 0 || !out_data) return -1;

    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    // Bind external device memory (dma-buf fd / AHardwareBuffer) as the input
    // tensor's device pointer. On a Vulkan backend that honors setDevicePtr with
    // MNN_MEMORY_AHARDWAREBUFFER, this imports the camera's CMA/dma-buf pages as
    // device memory -- no CPU staging copy, no heap allocation for MIPI Bayer.
    in_tensor->setDevicePtr(reinterpret_cast<const void*>(static_cast<uintptr_t>(ext_handle)), memory_type);

    if (in_shape && in_ndim > 0) {
        std::vector<int> dims(in_shape, in_shape + in_ndim);
        net->resizeTensor(in_tensor, dims);
    }

    auto* output = net->getSessionOutput(sess, nullptr);
    if (!output) return -1;
    if (max_out > 0 && (int)output->elementSize() > max_out) return -1;
    output->buffer().host = reinterpret_cast<uint8_t*>(out_data);

    net->runSession(sess);
    return 0;
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

// ── Benchmark with GPU synchronization ─────────────────────────────────
//
// Uses Tensor::wait(MAP_TENSOR_READ, true) for clean GPU sync.
// This blocks the CPU until the GPU finishes — giving real latency.
//
// Note: MNN does NOT have MAP_TO_HOST.  MapType enum:
//   MAP_TENSOR_WRITE = 0, MAP_TENSOR_READ = 1

#include <sys/time.h>

static inline uint64_t bench_gettime_us() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<uint64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;
}

extern "C" int mnn_benchmark_sync(
    MnnInterpreter interpreter,
    MnnSession session,
    int warmup,
    int loops,
    int precision,
    float* out_costs_ms,
    int max_costs
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !out_costs_ms || loops <= 0) return -1;

    auto* output = net->getSessionOutput(sess, NULL);
    if (!output) return -2;

    // Warm-up iterations (not timed)
    for (int i = 0; i < warmup; i++) {
        net->runSession(sess);
        output->wait(MNN::Tensor::MAP_TENSOR_READ, true);
    }

    // Timed iterations
    int n = loops < max_costs ? loops : max_costs;
    for (int i = 0; i < n; i++) {
        uint64_t t0 = bench_gettime_us();
        net->runSession(sess);
        // GPU sync: blocks until output is ready
        output->wait(MNN::Tensor::MAP_TENSOR_READ, true);
        uint64_t t1 = bench_gettime_us();
        out_costs_ms[i] = static_cast<float>(t1 - t0) / 1000.0f;
    }

    return n;
}

