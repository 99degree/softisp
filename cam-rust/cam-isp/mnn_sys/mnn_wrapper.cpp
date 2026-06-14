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
#include <MNN/expr/Expr.hpp>
#include <MNN/expr/Module.hpp>
#include <MNN/expr/ExprCreator.hpp>
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

// ── Host-tensor inference (copyFromHostTensor / copyToHostTensor) ──────

int mnn_run_host_tensors(MnnInterpreter interpreter, MnnSession session,
                          const float* in_data, const int* in_shape, int in_ndim,
                          float* out_data, int max_out) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !in_data || !out_data) return -1;

    // Get input tensor
    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    // Use the model input tensor's type for the host tensor
    auto model_type = in_tensor->getType();
    
    std::vector<int> shape(in_shape, in_shape + in_ndim);
    auto* host_in = MNN::Tensor::create(shape, model_type, nullptr, MNN::Tensor::CAFFE);
    if (!host_in) return -1;

    int total = 1;
    for (int i = 0; i < in_ndim; i++) total *= in_shape[i];
    
    // Copy float data and convert to proper type
    if (model_type.code == halide_type_float && model_type.bits == 32) {
        auto* ptr = host_in->host<float>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        memcpy(ptr, in_data, total * sizeof(float));
    } else if (model_type.code == halide_type_int && model_type.bits == 16) {
        auto* ptr = host_in->host<int16_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (int16_t)in_data[i];
    } else if (model_type.code == halide_type_uint && model_type.bits == 16) {
        auto* ptr = host_in->host<uint16_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (uint16_t)in_data[i];
    } else if (model_type.code == halide_type_int && model_type.bits == 32) {
        auto* ptr = host_in->host<int32_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (int32_t)in_data[i];
    } else if (model_type.code == halide_type_uint && model_type.bits == 8) {
        auto* ptr = host_in->host<uint8_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (uint8_t)in_data[i];
    } else {
        // Fallback: try float
        auto* ptr = host_in->host<float>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        memcpy(ptr, in_data, total * sizeof(float));
    }

    // Resize session to match host shape (handles symbolic dims)
    net->resizeSession(sess);
    auto copy_ok = in_tensor->copyFromHostTensor(host_in);
    if (!copy_ok) { MNN::Tensor::destroy(host_in); return -12; }
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) { MNN::Tensor::destroy(host_in); return -13; }

    // Get output
    auto* out_tensor = net->getSessionOutput(sess, nullptr);
    if (!out_tensor) { MNN::Tensor::destroy(host_in); return -1; }

    // Create host output tensor matching output type
    auto out_type = out_tensor->getType();
    auto* host_out = new MNN::Tensor(out_tensor, MNN::Tensor::CAFFE);
    out_tensor->copyToHostTensor(host_out);

    // Read output as float
    auto out_shape = host_out->shape();
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    int n = out_total < max_out ? out_total : max_out;
    
    if (out_type.code == halide_type_float && out_type.bits == 32) {
        auto* ptr = host_out->host<float>();
        if (ptr) memcpy(out_data, ptr, n * sizeof(float));
    } else if (out_type.code == halide_type_int && out_type.bits == 16) {
        auto* ptr = host_out->host<int16_t>();
        if (ptr) for (int i = 0; i < n; i++) out_data[i] = (float)ptr[i];
    } else if (out_type.code == halide_type_uint && out_type.bits == 16) {
        auto* ptr = host_out->host<uint16_t>();
        if (ptr) for (int i = 0; i < n; i++) out_data[i] = (float)ptr[i];
    } else {
        auto* ptr = host_out->host<float>();
        if (ptr) memcpy(out_data, ptr, n * sizeof(float));
    }

    MNN::Tensor::destroy(host_in);
    delete host_out;
    return n;
}


// ── Host tensors with u16 input ──────────────────────────────────────────

extern "C" int mnn_run_host_tensors_u16(MnnInterpreter interpreter, MnnSession session,
                          const uint16_t* in_data, const int* in_shape, int in_ndim,
                          float* out_data, int max_out) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !in_data || !out_data) return -1;

    // Get input tensor
    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    // Use the model input tensor's type for the host tensor
    auto model_type = in_tensor->getType();
    
    std::vector<int> shape(in_shape, in_shape + in_ndim);
    auto* host_in = MNN::Tensor::create(shape, model_type, nullptr, MNN::Tensor::CAFFE);
    if (!host_in) return -1;

    int total = 1;
    for (int i = 0; i < in_ndim; i++) total *= in_shape[i];
    
    // Copy u16 data and convert to proper type
    if (model_type.code == halide_type_int && model_type.bits == 16) {
        auto* ptr = host_in->host<int16_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (int16_t)in_data[i];
    } else if (model_type.code == halide_type_uint && model_type.bits == 16) {
        auto* ptr = host_in->host<uint16_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = in_data[i];
    } else if (model_type.code == halide_type_int && model_type.bits == 32) {
        auto* ptr = host_in->host<int32_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (int32_t)in_data[i];
    } else if (model_type.code == halide_type_uint && model_type.bits == 32) {
        auto* ptr = host_in->host<uint32_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (uint32_t)in_data[i];
    } else if (model_type.code == halide_type_float && model_type.bits == 32) {
        auto* ptr = host_in->host<float>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (float)in_data[i] / 1024.0f; // normalize
    } else {
        // Fallback: convert to float
        auto* ptr = host_in->host<float>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        for (int i = 0; i < total; i++) ptr[i] = (float)in_data[i] / 1024.0f;
    }

    // Resize session to match host shape (handles symbolic dims)
    net->resizeSession(sess);
    auto copy_ok = in_tensor->copyFromHostTensor(host_in);
    if (!copy_ok) { MNN::Tensor::destroy(host_in); return -12; }
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) { MNN::Tensor::destroy(host_in); return -13; }

    // Get output
    auto* out_tensor = net->getSessionOutput(sess, nullptr);
    if (!out_tensor) { MNN::Tensor::destroy(host_in); return -1; }

    // Create host output tensor matching output type
    auto out_type = out_tensor->getType();
    auto* host_out = new MNN::Tensor(out_tensor, MNN::Tensor::CAFFE);
    out_tensor->copyToHostTensor(host_out);

    // Read output as float
    auto out_shape = host_out->shape();
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    int n = out_total < max_out ? out_total : max_out;
    
    if (out_type.code == halide_type_float && out_type.bits == 32) {
        auto* ptr = host_out->host<float>();
        if (ptr) memcpy(out_data, ptr, n * sizeof(float));
    } else if (out_type.code == halide_type_int && out_type.bits == 16) {
        auto* ptr = host_out->host<int16_t>();
        if (ptr) for (int i = 0; i < n; i++) out_data[i] = (float)ptr[i];
    } else if (out_type.code == halide_type_uint && out_type.bits == 16) {
        auto* ptr = host_out->host<uint16_t>();
        if (ptr) for (int i = 0; i < n; i++) out_data[i] = (float)ptr[i];
    } else {
        auto* ptr = host_out->host<float>();
        if (ptr) memcpy(out_data, ptr, n * sizeof(float));
    }

    MNN::Tensor::destroy(host_in);
    delete host_out;
    return n;
}


// ── Zero-copy inference (wrap existing buffer directly) ──────────────────

int mnn_run_with_buffer(MnnInterpreter interpreter, MnnSession session,
                          const void* buffer, int buffer_type_code, int buffer_type_bits,
                          const int* in_shape, int in_ndim,
                          float* out_data, int max_out) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !buffer || !out_data) return -1;

    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;

    halide_type_t buffer_type = { (halide_type_code_t)buffer_type_code, (uint8_t)buffer_type_bits, 1 };
    auto model_type = in_tensor->getType();
    
    std::vector<int> shape(in_shape, in_shape + in_ndim);
    int total = 1;
    for (int i = 0; i < in_ndim; i++) total *= in_shape[i];

    MNN::Tensor* host_in = nullptr;

    // Zero-copy if types match: wrap buffer directly
    if (model_type.code == buffer_type.code && model_type.bits == buffer_type.bits) {
        host_in = MNN::Tensor::create(shape, buffer_type, const_cast<void*>(buffer), MNN::Tensor::CAFFE);
        if (!host_in) return -1;
    } else {
        // Types differ: allocate proper host tensor and convert
        host_in = MNN::Tensor::create(shape, model_type, nullptr, MNN::Tensor::CAFFE);
        if (!host_in) return -1;

        // Convert buffer data to model_type using a common float intermediate
        // First on stack if small, else allocated
        std::vector<float> float_buf;
        const float* src_float = nullptr;
        
        if (buffer_type.code == halide_type_float && buffer_type.bits == 32) {
            src_float = (const float*)buffer;
        } else {
            float_buf.resize(total);
            if (buffer_type.code == halide_type_int && buffer_type.bits == 16) {
                const int16_t* s = (const int16_t*)buffer;
                for (int i = 0; i < total; i++) float_buf[i] = (float)s[i];
            } else if (buffer_type.code == halide_type_uint && buffer_type.bits == 16) {
                const uint16_t* s = (const uint16_t*)buffer;
                for (int i = 0; i < total; i++) float_buf[i] = (float)s[i];
            } else if (buffer_type.code == halide_type_int && buffer_type.bits == 8) {
                const int8_t* s = (const int8_t*)buffer;
                for (int i = 0; i < total; i++) float_buf[i] = (float)s[i];
            } else if (buffer_type.code == halide_type_uint && buffer_type.bits == 8) {
                const uint8_t* s = (const uint8_t*)buffer;
                for (int i = 0; i < total; i++) float_buf[i] = (float)s[i];
            } else {
                memcpy(float_buf.data(), buffer, total * sizeof(float));
            }
            src_float = float_buf.data();
        }
        
        // Now convert from float to model_type
        if (model_type.code == halide_type_float && model_type.bits == 32) {
            memcpy(host_in->host<float>(), src_float, total * sizeof(float));
        } else if (model_type.code == halide_type_int && model_type.bits == 32) {
            auto* dst = host_in->host<int32_t>();
            for (int i = 0; i < total; i++) dst[i] = (int32_t)src_float[i];
        } else if (model_type.code == halide_type_int && model_type.bits == 16) {
            auto* dst = host_in->host<int16_t>();
            for (int i = 0; i < total; i++) dst[i] = (int16_t)src_float[i];
        } else if (model_type.code == halide_type_uint && model_type.bits == 16) {
            auto* dst = host_in->host<uint16_t>();
            for (int i = 0; i < total; i++) dst[i] = (uint16_t)src_float[i];
        } else {
            memcpy(host_in->host<float>(), src_float, total * sizeof(float));
        }
    }

    net->resizeSession(sess);
    auto copy_ok = in_tensor->copyFromHostTensor(host_in);
    if (!copy_ok) { MNN::Tensor::destroy(host_in); return -22; }
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) { MNN::Tensor::destroy(host_in); return -23; }

    auto* out_tensor = net->getSessionOutput(sess, nullptr);
    if (!out_tensor) { MNN::Tensor::destroy(host_in); return -1; }

    auto out_type = out_tensor->getType();
    auto* host_out = new MNN::Tensor(out_tensor, MNN::Tensor::CAFFE);
    out_tensor->copyToHostTensor(host_out);

    auto out_shape = host_out->shape();
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    int n = out_total < max_out ? out_total : max_out;

    if (out_type.code == halide_type_float && out_type.bits == 32) {
        auto* ptr = host_out->host<float>();
        if (ptr) memcpy(out_data, ptr, n * sizeof(float));
    } else if (out_type.code == halide_type_int && out_type.bits == 16) {
        auto* ptr = host_out->host<int16_t>();
        if (ptr) for (int i = 0; i < n; i++) out_data[i] = (float)ptr[i];
    } else if (out_type.code == halide_type_uint && out_type.bits == 16) {
        auto* ptr = host_out->host<uint16_t>();
        if (ptr) for (int i = 0; i < n; i++) out_data[i] = (float)ptr[i];
    } else {
        auto* ptr = host_out->host<float>();
        if (ptr) memcpy(out_data, ptr, n * sizeof(float));
    }

    MNN::Tensor::destroy(host_in);
    delete host_out;
    return n;
}

// ── Express Module API (C wrapper) ──────────────────────────────────────

using namespace MNN::Express;

MnnVARP* mnn_express_load_vars(const char* path, int* out_count) {
    if (!path || !out_count) return nullptr;
    try {
        auto vars = Variable::load(path);
        if (vars.empty()) return nullptr;
        int n = (int)vars.size();
        auto* arr = new MnnVARP[n];
        for (int i = 0; i < n; i++) {
            arr[i] = new VARP(vars[i]);
        }
        *out_count = n;
        return arr;
    } catch (...) { return nullptr; }
}

MnnExpressModule mnn_express_extract(MnnVARP* inputs, int n_inputs, MnnVARP* outputs, int n_outputs) {
    if (!inputs || !outputs || n_inputs < 1 || n_outputs < 1) return nullptr;
    try {
        std::vector<VARP> ins, outs;
        for (int i = 0; i < n_inputs; i++) ins.push_back(*(VARP*)inputs[i]);
        for (int i = 0; i < n_outputs; i++) outs.push_back(*(VARP*)outputs[i]);
        auto* module = Module::extract(ins, outs, false);
        return (MnnExpressModule)module;
    } catch (...) { return nullptr; }
}

void mnn_express_destroy_module(MnnExpressModule module) {
    auto* m = (Module*)module;
    if (m) Module::destroy(m);
}

MnnVARP mnn_express_create_input(const int* dims, int ndim, int format, int dtype) {
    if (!dims) return nullptr;
    try {
        std::vector<int> shape(dims, dims + ndim);
        auto fmt = (Dimensionformat)format;
        halide_type_t type;
        if (dtype == 4) type = halide_type_of<float>();
        else type = halide_type_of<int>();
        auto var = _Input(shape, fmt, type);
        if (!var.get()) return nullptr;
        return new VARP(var);
    } catch (...) { return nullptr; }
}

void* mnn_express_write_map(MnnVARP varp) {
    if (!varp) return nullptr;
    try { return ((VARP*)varp)->get()->writeMap<float>(); }
    catch (...) { return nullptr; }
}

const void* mnn_express_read_map(MnnVARP varp) {
    if (!varp) return nullptr;
    try { return ((VARP*)varp)->get()->readMap<float>(); }
    catch (...) { return nullptr; }
}

int mnn_express_var_info(MnnVARP varp, int* dims_out, int max_dims, int* out_format) {
    if (!varp || !dims_out || !out_format) return -1;
    try {
        auto* info = ((VARP*)varp)->get()->getInfo();
        if (!info) return -1;
        int n = (int)info->dim.size() < max_dims ? (int)info->dim.size() : max_dims;
        for (int i = 0; i < n; i++) dims_out[i] = info->dim[i];
        *out_format = (int)info->order;
        return n;
    } catch (...) { return -1; }
}

int mnn_express_var_resize(MnnVARP varp, const int* dims, int ndim) {
    if (!varp || !dims) return -1;
    try {
        std::vector<int> shape(dims, dims + ndim);
        return ((VARP*)varp)->get()->resize(shape) ? 0 : -1;
    } catch (...) { return -1; }
}

MnnVARP* mnn_express_forward(MnnExpressModule module, MnnVARP* inputs, int n_inputs, int* out_count) {
    if (!module || !inputs || !out_count) return nullptr;
    try {
        auto* m = (Module*)module;
        std::vector<VARP> ins;
        for (int i = 0; i < n_inputs; i++) ins.push_back(*(VARP*)inputs[i]);
        auto outs = m->onForward(ins);
        if (outs.empty()) { *out_count = 0; return nullptr; }
        int n = (int)outs.size();
        auto* arr = new MnnVARP[n];
        for (int i = 0; i < n; i++) {
            arr[i] = new VARP(outs[i]);
        }
        *out_count = n;
        return arr;
    } catch (...) { return nullptr; }
}

void mnn_varps_destroy(MnnVARP* varps, int count) {
    if (!varps) return;
    for (int i = 0; i < count; i++) {
        delete (VARP*)varps[i];
    }
    delete[] varps;
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
int mnn_run_zero_copy(
    MnnInterpreter interpreter,
    MnnSession session,
    const void* buffer,
    const int* in_shape,
    int in_ndim,
    float* out_data,
    int max_out
) {
    auto* net = reinterpret_cast<MNN::Interpreter*>(interpreter);
    auto* sess = reinterpret_cast<MNN::Session*>(session);
    if (!net || !sess || !buffer || !out_data) return -1;
    
    // Get input tensor
    auto* in_tensor = net->getSessionInput(sess, nullptr);
    if (!in_tensor) return -1;
    
    // Get shape and calculate total elements
    std::vector<int> shape(in_shape, in_shape + in_ndim);
    int total = 1;
    for (int i = 0; i < in_ndim; i++) total *= in_shape[i];
    
    // Get model input type
    auto model_type = in_tensor->getType();
    
    // Check if we can use zero-copy (types match)
    // For zero-copy, we need to set the host pointer directly on the input tensor
    // This works even if the tensor has no backend, as long as the pointer is valid
    
    // Try zero-copy first: directly set host pointer
    // Get the halide_buffer_t from the tensor
    // Note: This accesses internal MNN structure - may break with MNN updates
    // But it's the only way to make copyFromHostTensor work for tensors with null backend
    
    // For now, use copyFromHostTensor with a host tensor
    // This is the safe approach that works for most cases
    auto* host_in = MNN::Tensor::create(shape, model_type, nullptr, MNN::Tensor::CAFFE);
    if (!host_in) return -1;
    
    // Fill host tensor from buffer
    if (model_type.code == halide_type_float && model_type.bits == 32) {
        auto* ptr = host_in->host<float>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        memcpy(ptr, buffer, total * sizeof(float));
    } else if (model_type.code == halide_type_int && model_type.bits == 32) {
        auto* ptr = host_in->host<int32_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        memcpy(ptr, buffer, total * sizeof(int32_t));
    } else if (model_type.code == halide_type_int && model_type.bits == 16) {
        auto* ptr = host_in->host<int16_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        memcpy(ptr, buffer, total * sizeof(int16_t));
    } else if (model_type.code == halide_type_uint && model_type.bits == 16) {
        auto* ptr = host_in->host<uint16_t>();
        if (!ptr) { MNN::Tensor::destroy(host_in); return -1; }
        memcpy(ptr, buffer, total * sizeof(uint16_t));
    } else {
        MNN::Tensor::destroy(host_in);
        return -2;
    }
    
    // Resize session if needed
    net->resizeSession(sess);
    
    // Try copyFromHostTensor
    auto copy_ok = in_tensor->copyFromHostTensor(host_in);
    if (!copy_ok) {
        // copyFromHostTensor failed - try direct host pointer assignment
        // This is the zero-copy fallback for tensors with null backend
        in_tensor->buffer().host = const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer));
        // Note: We don't call resizeSession again as it may overwrite the host pointer
    }
    
    MNN::Tensor::destroy(host_in);
    
    // Run inference
    auto run_ok = net->runSession(sess);
    if (run_ok != 0) {
        return static_cast<int>(run_ok);
    }
    
    // Get output
    auto* out_tensor = net->getSessionOutput(sess, nullptr);
    if (!out_tensor) return -1;
    
    auto out_type = out_tensor->getType();
    auto* host_out = new MNN::Tensor(out_tensor, MNN::Tensor::CAFFE);
    out_tensor->copyToHostTensor(host_out);
    
    auto out_shape = host_out->shape();
    int out_total = 1;
    for (auto d : out_shape) out_total *= d;
    int n = out_total < max_out ? out_total : max_out;
    
    if (out_type.code == halide_type_float && out_type.bits == 32) {
        auto* ptr = host_out->host<float>();
        if (ptr) memcpy(out_data, ptr, n * sizeof(float));
    } else if (out_type.code == halide_type_int && out_type.bits == 16) {
        auto* ptr = host_out->host<int16_t>();
        if (ptr) for (int i = 0; i < n; i++) out_data[i] = (float)ptr[i];
    } else if (out_type.code == halide_type_uint && out_type.bits == 16) {
        auto* ptr = host_out->host<uint16_t>();
        if (ptr) for (int i = 0; i < n; i++) out_data[i] = (float)ptr[i];
    } else {
        auto* ptr = host_out->host<float>();
        if (ptr) memcpy(out_data, ptr, n * sizeof(float));
    }
    
    delete host_out;
    return n;
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

