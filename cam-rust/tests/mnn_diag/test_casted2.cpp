#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    std::cout << "=== Testing casted.mnn (FLOAT Mul 255.0) ===\n";
    auto* interp = Interpreter::createFromFile("casted.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    int H = 4, W = 4;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    auto inType = in->getType();
    std::cout << "Input type: code=" << (int)inType.code << " bits=" << (int)inType.bits << "\n";
    std::cout << "NOTE: code=0=INT32, code=2=FLOAT on this platform\n";
    
    // Create CAFFE-format host tensor with the same type
    auto* hostIn = Tensor::create({1, 1, H, W}, inType, nullptr, Tensor::CAFFE);
    
    // Write data in the right type
    if (inType.code == 2) { // FLOAT
        float* d = (float*)hostIn->host<void>();
        for (int i = 0; i < H*W; i++) d[i] = (float)(i % 256);
    } else { // INT32
        int32_t* d = (int32_t*)hostIn->host<void>();
        for (int i = 0; i < H*W; i++) d[i] = (int32_t)(i % 256);
    }
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    std::cout << "Output shape: [";
    for (auto d : out->shape()) std::cout << d << ",";
    std::cout << "]\n";
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1; for (auto d : s) n *= d;
    std::cout << "HostOut n=" << n << "\n";
    
    if (n > 0) {
        float* d = (float*)hostOut->host<void>();
        std::cout << "First 4 (float): " << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << "\n";
        std::cout << "Expected FLOAT: 0 255 510 765\n";
        
        int32_t* di = (int32_t*)d;
        std::cout << "First 4 (int32): " << di[0] << " " << di[1] << " " << di[2] << " " << di[3] << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
