#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;
int main() {
    auto* interp = Interpreter::createFromFile("test_nocast_new.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed\n"; return 1; }
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Input type: code=" << (int)inType.code << " bits=" << (int)inType.bits << "\n";
    
    int H = 48, W = 64;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    // Create host tensor with device type, CAFFE format
    auto* hostIn = Tensor::create({1, 1, H, W}, inType, nullptr, Tensor::CAFFE);
    if (inType.code == 2) { // FLOAT
        float* d = (float*)hostIn->host<void>();
        for (int i = 0; i < H*W; i++) d[i] = (float)(i % 256);
    } else {
        int32_t* d = (int32_t*)hostIn->host<void>();
        for (int i = 0; i < H*W; i++) d[i] = (int32_t)(i % 256);
    }
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1; for (auto d : s) n *= d;
    std::cout << "Output: n=" << n;
    for (auto d : s) std::cout << " " << d;
    std::cout << "\n";
    
    if (n > 0) {
        float* d = (float*)hostOut->host<void>();
        std::cout << "First 8: ";
        for (int i = 0; i < 8; i++) std::cout << d[i] << " ";
        std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
