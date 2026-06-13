#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <iostream>
#include <vector>
#include <cstring>
using namespace MNN;

int main() {
    const char* path = "/data/data/com.termux/files/home/test_pipeline.mnn";
    
    auto* interp = Interpreter::createFromFile(path);
    if (!interp) { std::cerr << "load failed" << std::endl; return 1; }
    std::cout << "OK: Interpreter loaded" << std::endl;
    
    ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_CPU;
    cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed" << std::endl; return 1; }
    std::cout << "OK: Session created" << std::endl;
    
    // Get input tensor info
    auto* in = interp->getSessionInput(sess, nullptr);
    if (!in) { std::cerr << "no input" << std::endl; return 1; }
    
    auto inShape = in->shape();
    auto inType = in->getType();
    std::cout << "Input shape: [";
    for (auto d : inShape) std::cout << d << ",";
    std::cout << "] type(byte per element)=" << inType.bytes() << std::endl;
    
    // Resize input to 64x48 (tiny)
    int W = 64, H = 48;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    std::cout << "Resized to [1,1," << H << "," << W << "]" << std::endl;
    
    // Create host input tensor with INT16 data (raw sensor)
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<int16_t>());
    // Fill with pattern
    auto* hostPtr = hostIn->host<int16_t>();
    for (int i = 0; i < H * W; i++) {
        hostPtr[i] = (i % 256) * 256;  // simulated 12-bit raw data
    }
    
    // Copy to device tensor and run
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    // Get output
    auto* out = interp->getSessionOutput(sess, nullptr);
    if (!out) { std::cerr << "no output" << std::endl; return 1; }
    
    auto outShape = out->shape();
    auto outType = out->getType();
    std::cout << "Output shape: [";
    for (auto d : outShape) std::cout << d << ",";
    std::cout << "] type(byte)=" << outType.bytes() << std::endl;
    
    // Copy to host
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto hostOutShape = hostOut->shape();
    std::cout << "Host output shape: [";
    for (auto d : hostOutShape) std::cout << d << ",";
    std::cout << "]" << std::endl;
    
    if (outType.bytes() == 4) {
        auto* data = hostOut->host<float>();
        if (data) {
            int total = 1;
            for (auto d : hostOutShape) total *= d;
            std::cout << "First 8 floats: ";
            for (int i = 0; i < 8 && i < total; i++) std::cout << data[i] << " ";
            std::cout << std::endl;
        }
    } else if (outType == halide_type_of<uint8_t>()) {
        auto* data = hostOut->host<uint8_t>();
        if (data) {
            int total = 1;
            for (auto d : hostOutShape) total *= d;
            std::cout << "First 16 bytes: ";
            for (int i = 0; i < 16 && i < total; i++) std::cout << (int)data[i] << " ";
            std::cout << std::endl;
        }
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    std::cout << "ALL PASSED" << std::endl;
    return 0;
}
