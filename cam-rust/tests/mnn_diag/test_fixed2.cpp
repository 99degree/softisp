#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("fixed_pipeline.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Input: code=" << inType.code << " bits=" << (int)inType.bits << "\n";
    
    int H = 48, W = 64;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    auto* hostIn = Tensor::create({1, 1, H, W}, inType);
    float* fdata = (float*)hostIn->host<void>();
    for (int i = 0; i < H*W; i++) fdata[i] = (float)(i % 256);
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    auto ty = hostOut->getType();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output: [";
    for (auto d : s) std::cout << d << ",";
    std::cout << "] n=" << n << " type code=" << ty.code << " bits=" << (int)ty.bits << "\n";
    
    if (n > 0) {
        float* d = (float*)hostOut->host<void>();
        std::cout << "First 8: ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
