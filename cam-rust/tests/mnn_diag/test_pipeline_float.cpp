#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("test_nocast.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Input: code=" << inType.code << " bits=" << (int)inType.bits << "\n";
    
    int W = 64, H = 48;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    // Create FLOAT host tensor - let MNN convert
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<float>());
    float* p = (float*)hostIn->host<void>();
    for (int i = 0; i < H*W; i++) p[i] = (float)((i * 256 / W) % 256);  // 0..255 gradient
    
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
    
    if (ty.bytes() == 4) {
        float* d = (float*)hostOut->host<void>();
        std::cout << "First 8 (float): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\nExpected: input * 255.0 -> " << (p[0]*255.0f) << " " << (p[1]*255.0f) << " " << (p[2]*255.0f) << " ...\n";
    } else {
        uint8_t* d = (uint8_t*)hostOut->host<void>();
        std::cout << "First 16 (bytes): ";
        for (int i = 0; i < 16 && i < n * ty.bytes(); i++) std::cout << (int)d[i] << " ";
        std::cout << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
