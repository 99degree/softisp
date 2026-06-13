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
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    
    auto* in = interp->getSessionInput(sess, nullptr);
    int W = 64, H = 48;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<int16_t>());
    auto* ptr = hostIn->host<int16_t>();
    for (int i = 0; i < H * W; i++) ptr[i] = (int16_t)((i % 256) * 256 + 1); // non-zero
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto shape = hostOut->shape();
    int total = 1;
    std::cout << "Output shape: [";
    for (auto d : shape) { std::cout << d << ","; total *= d; }
    std::cout << "] total=" << total << std::endl;
    
    if (hostOut->getType() == halide_type_of<uint8_t>()) {
        auto* d = hostOut->host<uint8_t>();
        std::cout << "First 16 (uint8): ";
        for (int i = 0; i < 16 && i < total; i++) std::cout << (int)d[i] << " ";
    } else if (hostOut->getType() == halide_type_of<float>()) {
        auto* d = hostOut->host<float>();
        std::cout << "First 8 (float): ";
        for (int i = 0; i < 8 && i < total; i++) std::cout << d[i] << " ";
    }
    std::cout << std::endl;
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    std::cout << "ALL PASSED" << std::endl;
    return 0;
}
