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
#include <cstdlib>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <MNN/MNNForwardType.h>
#include <cstring>
#include <dlfcn.h>
#include <mutex>

// ── Last inference error ───────────────────────────────────────────────
// `mnn_run_with_output` can fail for several distinct reasons that all
// collapse into one negative return code. The engine reports the code, but
// the app only has its own log file — not logcat — so the reason has to be
// readable from Rust. The message is set on every failure path and cleared
// at the top of each run.
static std::mutex g_last_error_mutex;
static std::string g_last_error;

static void set_last_error(const char* fmt, ...) {
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    std::lock_guard<std::mutex> lock(g_last_error_mutex);
    g_last_error = buf;
    MNN_PRINT("[mnn_run] %s\n", buf);
}

/// Returns the message from the most recent `mnn_run_with_output` failure,
/// or an empty string when the last run succeeded.
extern "C" const char* mnn_last_error(void) {
    std::lock_guard<std::mutex> lock(g_last_error_mutex);
    return g_last_error.c_str();
}

// ── Vulkan plugin loader ───────────────────────────────────────────────
// MNN_SEP_BUILD compiles backends as separate .so files. Each backend has
// a static initializer (gResistor) that calls MNNInsertExtraRuntimeCreator(),
// but this only fires when the .so is dlopen'd into the process. libMNN.so
// does NOT auto-dlopen backend plugins — the caller must do it.
static std::once_flag g_vulkan_load_flag;
static void ensure_vulkan_loaded() {
    std::call_once(g_vulkan_load_flag, []() {
        void* h = dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_LOCAL);
        if (!h) {
            MNN_PRINT("[mnn_wrapper] dlopen(libMNN_Vulkan.so) failed: %s\n", dlerror());
        } else {
            MNN_PRINT("[mnn_wrapper] libMNN_Vulkan.so loaded OK\n");
        }
    });
}

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

    // Ensure Vulkan backend plugin is loaded (MNN_SEP_BUILD requires dlopen)
    if (backend == MNN_BACKEND_VULKAN) {
        ensure_vulkan_loaded();
    }

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

extern "C" int mnn_session_get_input_count(MnnInterpreter interpreter, MnnSession session) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return 0;
    const auto& inputs = net->getSessionInputAll(sess);
    return (int)inputs.size();
}

extern "C" const char* mnn_session_get_input_name(MnnInterpreter interpreter, MnnSession session, int index) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return nullptr;
    const auto& inputs = net->getSessionInputAll(sess);
    if (index < 0 || index >= (int)inputs.size()) return nullptr;
    auto iter = inputs.begin();
    std::advance(iter, index);
    return iter->first.c_str();
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

// ── Runtime tensor upload (host → backend) ──────────────────────────────

extern "C" int mnn_tensor_upload(
    MnnTensor tensor,
    const void* data,
    size_t bytes,
    int type_code,
    int type_bits,
    const int* shape,
    int ndim
) {
    auto* t = reinterpret_cast<MNN::Tensor*>(tensor);
    if (!t || !data || bytes == 0) return -1;

    // Shape in MNN's own order. The caller passes the tensor's current shape,
    // so this is just a defensive copy.
    std::vector<int> host_shape(shape, shape + ndim);
    size_t expected = 1;
    for (int i = 0; i < ndim; i++) expected *= static_cast<size_t>(shape[i]);

    halide_type_t host_type = { (halide_type_code_t)type_code, (uint8_t)type_bits, 1 };
    auto* host = MNN::Tensor::create(
        host_shape, host_type, const_cast<void*>(data), MNN::Tensor::CAFFE);
    if (!host) return -2;
    if (host->elementSize() != expected) {
        delete host;
        return -3;
    }

    t->copyFromHostTensor(host);
    delete host;
    return 0;
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
    if (run_ok != 0) {
        fprintf(stderr, "[mnn_host] runSession FAIL code=%d\n", run_ok);
        fprintf(stderr, "[mnn_host] input shape: [");
        for (size_t i = 0; i < host_shape.size(); i++)
            fprintf(stderr, "%s%d", i ? "," : "", host_shape[i]);
        fprintf(stderr, "]\n");
        auto* in_s = net->getSessionInput(sess, nullptr);
        if (in_s) {
            auto is = in_s->shape();
            fprintf(stderr, "[mnn_host] session input shape: [");
            for (size_t i = 0; i < is.size(); i++)
                fprintf(stderr, "%s%d", i ? "," : "", is[i]);
            fprintf(stderr, "] type=%d\n", in_s->getType().code);
        }
        auto* out_s = net->getSessionOutput(sess, nullptr);
        if (out_s) {
            auto os = out_s->shape();
            fprintf(stderr, "[mnn_host] session output shape: [");
            for (size_t i = 0; i < os.size(); i++)
                fprintf(stderr, "%s%d", i ? "," : "", os[i]);
            fprintf(stderr, "]\n");
        }
        delete host_in;
        return -2;
    }

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

/// Get the declared shape of the model's first input tensor.
/// Writes up to `max_ndim` dims into `out_dims` and returns the rank,
/// or -1 on error. The engine uses dims[1] (channels) + element type to
/// pick the input convention: [1,2,H,W/2] split lanes vs [1,1,H,W] raw.
extern "C" int mnn_get_model_input_dims(MnnInterpreter interpreter, MnnSession session,
                                         int* out_dims, int max_ndim) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !out_dims || max_ndim <= 0) return -1;
    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;
    auto s = in_tensor->shape();
    int n = (int)s.size();
    if (n > max_ndim) n = max_ndim;
    for (int i = 0; i < n; i++) out_dims[i] = s[i];
    return (int)s.size();
}


/**
 * Run inference and copy a SPECIFIC named output tensor to out_data.
 * first output (same as mnn_run_true_zero_copy).
 */
extern "C" int mnn_run_with_output(
    MnnInterpreter interpreter,
    MnnSession session,
    const char* input_name,
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
    {
        std::lock_guard<std::mutex> lock(g_last_error_mutex);
        g_last_error.clear();
    }

    // The model may declare several inputs (frame + runtime-fed weights).
    // MNN's getSessionInput(nullptr) returns the alphabetically-first input
    // (std::map ordering), which is NOT necessarily the frame — so the caller
    // must pass the frame input's name explicitly.
    auto* in_tensor = (input_name != nullptr)
        ? net->getSessionInput(sess, input_name)
        : net->getSessionInput(sess, nullptr);
    if (!in_tensor) {
        set_last_error("frame input '%s' not found in session",
                       input_name ? input_name : "(first)");
        return -1;
    }

    halide_type_t buffer_type = { (halide_type_code_t)buffer_type_code, (uint8_t)buffer_type_bits, 1 };
    auto model_type = in_tensor->getType();
    
    if (model_type.code != buffer_type.code || model_type.bits != buffer_type.bits) {
        set_last_error("input type mismatch: model code=%d bits=%d, buffer code=%d bits=%d",
                       (int)model_type.code, (int)model_type.bits,
                       (int)buffer_type.code, (int)buffer_type.bits);
        return -2;
    }

    std::vector<int> shape(in_shape, in_shape + in_ndim);
    int total = 1;
    for (int i = 0; i < in_ndim; i++) total *= in_shape[i];

    // Hand the frame to the graph.
    //
    // Assigning `buffer().host`/`buffer().device = 0` only works for backends
    // that execute in place. The Vulkan backend allocates its own device
    // buffer for every session tensor and never reads that host pointer, so
    // the frame was silently dropped and the graph inferred on zeros.
    // `copyFromHostTensor` is the supported upload — it does the host→device
    // transfer for whichever backend the session runs on.
    if (in_tensor->elementSize() != (size_t)total) {
        set_last_error("input element mismatch before resize: model=%zu requested=%d, resizing",
                       in_tensor->elementSize(), total);
        // Set the new shape on the input tensor, then propagate it through the
        // graph. `tryResizeSession` is the variant that reports *why* it failed
        // (memory vs. compute-size vs. unsupported), which is exactly what an
        // opaque negative return code cannot convey.
        net->resizeTensor(in_tensor, shape);
        MNN::ErrorCode rc = net->tryResizeSession(sess);
        if (rc != MNN::NO_ERROR) {
            set_last_error("tryResizeSession failed: code=%d (%s)", (int)rc,
                           rc == MNN::OUT_OF_MEMORY      ? "OUT_OF_MEMORY"
                           : rc == MNN::NOT_SUPPORT      ? "NOT_SUPPORT"
                           : rc == MNN::COMPUTE_SIZE_ERROR ? "COMPUTE_SIZE_ERROR"
                           : rc == MNN::NO_EXECUTION     ? "NO_EXECUTION"
                                                         : "OTHER");
            return -6;
        }
        in_tensor = (input_name != nullptr) ? net->getSessionInput(sess, input_name)
                                            : net->getSessionInput(sess, nullptr);
        if (!in_tensor) {
            set_last_error("frame input vanished after tryResizeSession");
            return -7;
        }
        if (in_tensor->elementSize() != (size_t)total) {
            auto s = in_tensor->shape();
            char shapebuf[128] = {0};
            size_t off = 0;
            for (size_t i = 0; i < s.size() && off + 12 < sizeof(shapebuf); i++) {
                off += snprintf(shapebuf + off, sizeof(shapebuf) - off, "%s%d",
                                i ? "," : "", s[i]);
            }
            set_last_error("resize did not take: model=%zu requested=%d resulting shape=[%s]",
                           in_tensor->elementSize(), total, shapebuf);
            return -8;
        }
    }

    auto* host_in = MNN::Tensor::create(
        shape, buffer_type, const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer)),
        MNN::Tensor::CAFFE);
    if (!host_in) return -3;
    in_tensor->copyFromHostTensor(host_in);
    delete host_in;

    // Run inference.
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) {
        set_last_error("runSession failed: MNN error code=%d (%s)", run_ok,
                       run_ok == 1 ? "OUT_OF_MEMORY" : run_ok == 2 ? "NOT_SUPPORT"
                       : run_ok == 3 ? "COMPUTE_SIZE_ERROR" : run_ok == 4 ? "NO_EXECUTION"
                       : "OTHER");
        // MNN returns positive error codes (e.g. COMPUTE_SIZE_ERROR=3).
        // Normalize to a negative value so callers treat failures as errors
        // rather than mistaking a big positive code for a valid element count.
        return -std::abs(static_cast<int>(run_ok));
    }

    // Read the result back into the caller's buffer.
    //
    // Pointing `out_tensor->buffer().host` at `out_data` before the run only
    // works when the backend keeps the output in that same allocation. The
    // Vulkan backend allocates its own device-backed buffer for the session
    // output and writes there, so that pointer assignment is discarded and the
    // caller was handed a buffer that never received anything (all zeros).
    // `copyToHostTensor` is the supported readback — it also handles the
    // device→host transfer.
    auto* out_tensor = (output_name != nullptr)
        ? net->getSessionOutput(sess, output_name)
        : net->getSessionOutput(sess, nullptr);
    // Fallback: if named output not found, try first output
    if (!out_tensor && output_name != nullptr) {
        out_tensor = net->getSessionOutput(sess, nullptr);
    }
    if (!out_tensor) return -1;

    auto out_shape = out_tensor->shape();
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    if (out_total <= 0) return -5;

    auto out_type = out_tensor->getType();
    auto* host_out = MNN::Tensor::create(out_shape, out_type, out_data, MNN::Tensor::CAFFE);
    if (!host_out) return -3;
    out_tensor->copyToHostTensor(host_out);
    delete host_out;

    // Return element count for the output tensor
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

// ── Benchmark with GPU synchronization ─────────────────────────────────
//
// Uses Tensor::map(MAP_TENSOR_READ) + unmap for GPU sync.
// This is the pattern MNN's own benchmark.cpp uses — map() triggers a
// device→host transfer which blocks until the GPU finishes.
//
// CRITICAL: If Vulkan falls back to CPU, map/unmap on host tensors is a
// no-op and timing will be ~0ms.  We detect this by checking the actual
// backend via getSessionInfo(BACKENDS) and by checking the input tensor's
// device type.
//
// MapType: MAP_TENSOR_WRITE=0, MAP_TENSOR_READ=1
// BackendConfig::getDeviceType(): 0=CPU, 1=GPU, 2=... etc

#include <sys/time.h>

static inline uint64_t bench_gettime_us() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<uint64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;
}

// Return the actual forward type the session is using (0=CPU, 7=Vulkan, etc.)
static int get_session_backend(MNN::Interpreter* net, MNN::Session* sess) {
    int backend = -1;
    net->getSessionInfo(sess, MNN::Interpreter::BACKENDS, &backend);
    return backend;
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

    auto* input  = net->getSessionInput(sess, NULL);
    auto* output = net->getSessionOutput(sess, NULL);
    if (!input || !output) return -2;

    // Check what backend is ACTUALLY running (may differ from requested)
    int actual_backend = get_session_backend(net, sess);
    // Store in out_costs_ms[0] as negative sentinel: -(actual_backend+1)
    // The Rust side can read this to detect fallback.
    // For now, just proceed — the caller can query separately.
    (void)actual_backend;

    // Warm-up iterations (not timed)
    for (int i = 0; i < warmup; i++) {
        void* w = input->map(MNN::Tensor::MAP_TENSOR_WRITE, input->getDimensionType());
        if (w) input->unmap(MNN::Tensor::MAP_TENSOR_WRITE, input->getDimensionType(), w);
        net->runSession(sess);
        void* r = output->map(MNN::Tensor::MAP_TENSOR_READ, output->getDimensionType());
        if (r) output->unmap(MNN::Tensor::MAP_TENSOR_READ, output->getDimensionType(), r);
    }

    // Timed iterations
    int n = loops < max_costs ? loops : max_costs;
    for (int i = 0; i < n; i++) {
        uint64_t t0 = bench_gettime_us();
        void* w = input->map(MNN::Tensor::MAP_TENSOR_WRITE, input->getDimensionType());
        if (w) input->unmap(MNN::Tensor::MAP_TENSOR_WRITE, input->getDimensionType(), w);
        net->runSession(sess);
        // GPU sync: map for READ blocks until device→host transfer completes
        void* r = output->map(MNN::Tensor::MAP_TENSOR_READ, output->getDimensionType());
        if (r) output->unmap(MNN::Tensor::MAP_TENSOR_READ, output->getDimensionType(), r);
        uint64_t t1 = bench_gettime_us();
        out_costs_ms[i] = static_cast<float>(t1 - t0) / 1000.0f;
    }

    return n;
}

extern "C" int mnn_get_actual_backend(
    MnnInterpreter interpreter,
    MnnSession session
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess) return -1;
    return get_session_backend(net, sess);
}

