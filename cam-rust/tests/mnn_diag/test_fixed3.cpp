#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("fixed_pipeline2.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Input: code=" << inType.code << " bits=" << (int)inType.bits << " bytes=" << inType.bytes() << "\n";
    
    int H = 48, W = 64;
    // Check existing shape
    auto ins = in->shape();
    std::cout << "Input shape: ["; for (auto d : ins) std::cout << d << ","; std::cout << "]\n";
    
    // Create FLOAT host tensor
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<float>());
    float* fdata = (float*)hostIn->host<void>();
    for (int i = 0; i < H*W; i++) fdata[i] = (float)(i % 256);
    std::cout << "Host input first: " << fdata[0] << " " << fdata[1] << " " << fdata[2] << " " << fdata[3] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    auto ty = hostOut->getType();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n;
    std::cout << " type code=" << ty.code << " bits=" << (int)ty.bits << "\n";
    
    if (n > 0 && ty.bytes() == 4) {
        float* d = (float*)hostOut->host<void>();
        std::cout << "First 8 (float): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
        
        int32_t* di = (int32_t*)d;
        std::cout << "First 8 (int32): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << di[i] << " ";
        std::cout << "\n";
    } else if (n > 0) {
        uint8_t* d = (uint8_t*)hostOut->host<void>();
        std::cout << "First 16 bytes: ";
        for (int i = 0; i < 16; i++) std::cout << (int)d[i] << " ";
        std::cout << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
