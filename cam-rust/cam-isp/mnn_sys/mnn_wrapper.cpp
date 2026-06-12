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
#include <MNN/MNNForwardType.h>
#include <cstring>

// ── Interpreter ──────────────────────────────────────────────────────────

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

MnnTensor mnn_session_get_input(MnnSession session, const char* name) {
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

    auto elementSize = t->elementSize();
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

int mnn_tensor_set_shape(MnnInterpreter interpreter, MnnSession session, MnnTensor tensor, const int* dims, int ndim) {
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

