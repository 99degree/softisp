#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <iostream>
using namespace MNN;
int main() {
    std::cout << "Loading...\n" << std::flush;
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Device type: code=" << (int)inType.code << " bits=" << (int)inType.bits << "\n" << std::flush;
    
    int H = 48, W = 64;
    
    // Check: what type does halide_type_of give us?
    auto floatType = halide_type_of<float>();
    auto int32Type = halide_type_of<int32_t>();
    auto uint16Type = halide_type_of<uint16_t>();
    std::cout << "float: code=" << (int)floatType.code << " bits=" << (int)floatType.bits << "\n";
    std::cout << "int32: code=" << (int)int32Type.code << " bits=" << (int)int32Type.bits << "\n";
    std::cout << "uint16: code=" << (int)uint16Type.code << " bits=" << (int)uint16Type.bits << "\n";
    
    // The ONNX model uses UINT16 input. MNN might convert this to INT32 on this platform.
    // let's try INT32 first (code=0 on this platform)
    halide_type_t expectedType = halide_type_t{0, 32}; // INT32 on this platform
    // Or try uint16: halide_type_t{1, 16}
    
    auto* hostIn = Tensor::create({1, 1, H, W}, expectedType, nullptr, Tensor::CAFFE);
    std::cout << "hostIn=" << (void*)hostIn << "\n" << std::flush;
    if (!hostIn) return 1;
    
    // INT32 data
    int32_t* d = (int32_t*)hostIn->host<void>();
    if (!d) { std::cerr << "null host\n"; return 1; }
    for (int i = 0; i < H*W; i++) d[i] = (int32_t)(i % 256);
    std::cout << "Data filled\n" << std::flush;
    
    in->copyFromHostTensor(hostIn);
    std::cout << "Copy OK\n" << std::flush;
    
    interp->runSession(sess);
    std::cout << "Run OK\n" << std::flush;
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    std::cout << "Output: " << (void*)out << "\n" << std::flush;
    if (!out) return 1;
    
    auto outType = out->getType();
    std::cout << "Output type: code=" << (int)outType.code << " bits=" << (int)outType.bits << "\n";
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1; for (auto d : s) n *= d;
    std::cout << "Shape: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
    
    if (n > 0) {
        float* f = (float*)hostOut->host<void>();
        std::cout << "First 4: " << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
