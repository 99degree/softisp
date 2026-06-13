#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <iostream>
#include <vector>
#include <cstring>
using namespace MNN;

int main() {
    const char* path = "/data/data/com.termux/files/home/test_pipeline_fixed.mnn";
    
    auto* interp = Interpreter::createFromFile(path);
    if (!interp) { std::cerr << "load failed" << std::endl; return 1; }
    std::cout << "Interpreter loaded" << std::endl;
    
    ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_CPU;
    cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed" << std::endl; return 1; }
    std::cout << "Session created" << std::endl;
    
    auto* in = interp->getSessionInput(sess, nullptr);
    if (!in) { std::cerr << "no input" << std::endl; return 1; }
    
    auto inShape = in->shape();
    auto inType = in->getType();
    std::cout << "Input shape: [";
    for (auto d : inShape) std::cout << d << ",";
    std::cout << "] type code=" << inType.code << " bits=" << inType.bits << std::endl;
    
    // Resize
    int W = 64, H = 48;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    std::cout << "Resized to [1,1," << H << "," << W << "]" << std::endl;
    
    // Create host input (INT16)
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<int16_t>());
    auto* hostPtr = hostIn->host<int16_t>();
    for (int i = 0; i < H * W; i++) hostPtr[i] = (i % 256) * 256;
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    // Get output
    auto* out = interp->getSessionOutput(sess, nullptr);
    if (!out) { std::cerr << "no output" << std::endl; return 1; }
    
    auto outShape = out->shape();
    auto outType = out->getType();
    std::cout << "Output shape: [";
    for (auto d : outShape) std::cout << d << ",";
    std::cout << "] type code=" << outType.code << " bits=" << outType.bits << std::endl;
    
    // Copy to host
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto hostOutShape = hostOut->shape();
    std::cout << "Host output shape: [";
    for (auto d : hostOutShape) std::cout << d << ",";
    std::cout << "]" << std::endl;
    
    auto hostOutType = hostOut->getType();
    std::cout << "Host output type code=" << hostOutType.code << " bits=" << hostOutType.bits << std::endl;
    
    if (hostOutType == halide_type_of<uint8_t>()) {
        auto* data = hostOut->host<uint8_t>();
        int total = 1;
        for (auto d : hostOutShape) total *= d;
        std::cout << "UINT8 output, total=" << total << std::endl;
        std::cout << "First 16: ";
        for (int i = 0; i < 16 && i < total; i++) std::cout << (int)data[i] << " ";
        std::cout << std::endl;
    } else if (hostOutType == halide_type_of<float>()) {
        auto* data = hostOut->host<float>();
        int total = 1;
        for (auto d : hostOutShape) total *= d;
        std::cout << "FLOAT output, total=" << total << std::endl;
        std::cout << "First 8: ";
        for (int i = 0; i < 8 && i < total; i++) std::cout << data[i] << " ";
        std::cout << std::endl;
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    std::cout << "ALL PASSED" << std::endl;
    return 0;
}
