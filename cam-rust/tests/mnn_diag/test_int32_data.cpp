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
    std::cout << " (NOTE: code=2 means INT32 on this platform)\n";
    
    // Create host tensor with SAME type as device (code=2 = INT32)
    // Write INT32 data directly
    auto* hostIn = Tensor::create({1, 1, 48, 64}, inType);
    int32_t* idata = (int32_t*)hostIn->host<void>();
    for (int i = 0; i < 48*64; i++) idata[i] = (int32_t)(i % 256);
    std::cout << "Host input first 4 (int32): " << idata[0] << " " << idata[1] << " " << idata[2] << " " << idata[3] << "\n";
    
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
        int32_t* d = (int32_t*)hostOut->host<void>();
        std::cout << "First 8 (int32): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
        
        float* df = (float*)d;
        std::cout << "First 4 (float): " << df[0] << " " << df[1] << " " << df[2] << " " << df[3] << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
