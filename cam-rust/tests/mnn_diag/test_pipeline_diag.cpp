#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;
int main() {
    std::cout << "1\n" << std::flush;
    auto* interp = Interpreter::createFromFile("isp_pipeline_fixed.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    std::cout << "2\n" << std::flush;
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed\n"; return 1; }
    std::cout << "3\n" << std::flush;
    auto* in = interp->getSessionInput(sess, nullptr);
    std::cout << "4\n" << std::flush;
    
    int H = 48, W = 64;
    interp->resizeTensor(in, {1, 1, H, W});
    std::cout << "5\n" << std::flush;
    interp->resizeSession(sess);
    std::cout << "6\n" << std::flush;
    
    auto inType = in->getType();
    std::cout << "7 type code=" << (int)inType.code << "\n" << std::flush;
    
    auto* hostIn = Tensor::create({1, 1, H, W}, inType, nullptr, Tensor::CAFFE);
    std::cout << "8 hostIn=" << (void*)hostIn << "\n" << std::flush;
    if (!hostIn) return 1;
    
    if (inType.code == 2) { // FLOAT
        float* d = (float*)hostIn->host<void>();
        std::cout << "9 d=" << (void*)d << "\n" << std::flush;
        if (!d) return 1;
        for (int i = 0; i < H*W; i++) d[i] = (float)(i % 256);
    } else {
        int32_t* d = (int32_t*)hostIn->host<void>();
        if (!d) return 1;
        for (int i = 0; i < H*W; i++) d[i] = (int32_t)(i % 256);
    }
    std::cout << "10\n" << std::flush;
    
    in->copyFromHostTensor(hostIn);
    std::cout << "11\n" << std::flush;
    
    interp->runSession(sess);
    std::cout << "12\n" << std::flush;
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    std::cout << "13 out=" << (void*)out << "\n" << std::flush;
    if (!out) return 1;
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    std::cout << "14\n" << std::flush;
    out->copyToHostTensor(hostOut);
    std::cout << "15\n" << std::flush;
    
    auto s = hostOut->shape();
    std::cout << "16 n=";
    int n = 1; for (auto d : s) { n *= d; std::cout << d << " "; }
    std::cout << "\n" << std::flush;
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    std::cout << "Done\n";
    return 0;
}
