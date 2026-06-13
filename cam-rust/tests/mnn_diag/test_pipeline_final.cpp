#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("isp_pipeline_fixed.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    if (!in) { std::cerr << "no input\n"; return 1; }
    
    auto inType = in->getType();
    auto inshape = in->shape();
    std::cout << "Input shape: ["; for (auto d : inshape) std::cout << d << ","; std::cout << "]\n";
    std::cout << "Input type: code=" << (int)inType.code << " bits=" << (int)inType.bits << "\n";
    // On this platform: float=2, int=0
    
    int H = 48, W = 64;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    // Create CAFFE-format host tensor matching device type
    auto* hostIn = Tensor::create({1, 1, H, W}, inType, nullptr, Tensor::CAFFE);
    if (inType.code == 2) { // FLOAT on this platform
        float* d = (float*)hostIn->host<void>();
        for (int i = 0; i < H*W; i++) d[i] = (float)(i % 256);
    } else { // INT32
        int32_t* d = (int32_t*)hostIn->host<void>();
        for (int i = 0; i < H*W; i++) d[i] = (int32_t)(i % 256);
    }
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    if (!out) { std::cerr << "no output\n"; return 1; }
    
    auto outS = out->shape();
    std::cout << "Output shape: ["; for (auto d : outS) std::cout << d << ","; std::cout << "]\n";
    auto outType = out->getType();
    std::cout << "Output type: code=" << (int)outType.code << " bits=" << (int)outType.bits << "\n";
    
    // Read output using CAFFE
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1; for (auto d : s) n *= d;
    std::cout << "HostOut n=" << n;
    for (auto d : s) std::cout << " " << d;
    std::cout << "\n";
    
    if (n > 0) {
        float* d = (float*)hostOut->host<void>();
        std::cout << "First 8 (float): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
        
        int32_t* di = (int32_t*)d;
        std::cout << "First 8 (int32): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << di[i] << " ";
        std::cout << "\n";
        
        int nonZero = 0;
        for (int i = 0; i < n && i < 100; i++) if (d[i] != 0.0f) nonZero++;
        std::cout << "Non-zero in first 100: " << nonZero << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
