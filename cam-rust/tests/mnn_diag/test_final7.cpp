#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <iostream>
using namespace MNN;
int main() {
    std::cout << "Loading test_final2.mnn\n" << std::flush;
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed\n"; return 1; }
    
    auto* in = interp->getSessionInput(sess, nullptr);
    if (!in) { std::cerr << "no input\n"; return 1; }
    
    int H = 48, W = 64;
    // Use halide_type_t with explicit code=0 (INT32 on this platform)
    halide_type_t inputType{(halide_type_code_t)0, 32, 1}; // INT32, 32-bit, 1 lane
    std::cout << "Using type: code=" << (int)inputType.code << " bits=" << (int)inputType.bits << "\n" << std::flush;
    
    auto* hostIn = Tensor::create({1, 1, H, W}, inputType, nullptr, Tensor::CAFFE);
    if (!hostIn) { std::cerr << "hostIn null\n"; return 1; }
    std::cout << "hostIn=" << (void*)hostIn << "\n" << std::flush;
    
    int32_t* d = (int32_t*)hostIn->host<void>();
    if (!d) { std::cerr << "null host data\n"; return 1; }
    for (int i = 0; i < H*W; i++) d[i] = (int32_t)(i % 256);
    std::cout << "Wrote input data\n" << std::flush;
    
    in->copyFromHostTensor(hostIn);
    std::cout << "Copied to device\n" << std::flush;
    
    interp->runSession(sess);
    std::cout << "Session ran\n" << std::flush;
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    if (!out) { std::cerr << "no output\n"; return 1; }
    std::cout << "Output OK\n" << std::flush;
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output shape: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
    
    if (n > 0) {
        float* f = (float*)hostOut->host<void>();
        std::cout << "First 4: " << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    printf("Success!\n");
    return 0;
}
