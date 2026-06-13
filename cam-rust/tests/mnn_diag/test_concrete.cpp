#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("concrete.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Input: code=" << inType.code << " bits=" << (int)inType.bits << " bytes=" << inType.bytes() << "\n";
    
    auto inshape = in->shape();
    std::cout << "Input shape: ["; for (auto d : inshape) std::cout << d << ","; std::cout << "]\n";
    
    int H = inshape[2], W = inshape[3];
    if (H < 0) { H = 48; W = 64; }
    
    // Create FLOAT host tensor for type propagation
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<float>());
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
    std::cout << "Output: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n;
    std::cout << " type code=" << ty.code << " bits=" << (int)ty.bits << "\n";
    
    if (n > 0 && ty.bytes() == 4) {
        float* d = (float*)hostOut->host<void>();
        std::cout << "First 4 (float): " << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << "\n";
        std::cout << "Expected: 0 255 510 765\n";
        
        // Check as int32 too
        int32_t* di = (int32_t*)d;
        std::cout << "First 4 (int32): " << di[0] << " " << di[1] << " " << di[2] << " " << di[3] << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
