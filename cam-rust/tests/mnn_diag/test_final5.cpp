#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;
int main() {
    std::cout << "Start\n" << std::flush;
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Input type: code=" << (int)inType.code << " bits=" << (int)inType.bits << "\n" << std::flush;
    std::cout << "Input shape empty: " << (in->shape().empty() ? "yes" : "no") << "\n" << std::flush;
    
    // Use device type to create host tensor
    auto* hostIn = Tensor::create({1, 1, 48, 64}, inType, nullptr, Tensor::CAFFE);
    std::cout << "hostIn=" << (void*)hostIn << "\n" << std::flush;
    if (inType.code == 2) { // FLOAT
        float* d = (float*)hostIn->host<void>();
        for (int i = 0; i < 48*64; i++) d[i] = (float)(i % 256);
    } else { // INT32
        int32_t* d = (int32_t*)hostIn->host<void>();
        for (int i = 0; i < 48*64; i++) d[i] = (int32_t)(i % 256);
    }
    std::cout << "host filled\n" << std::flush;
    
    in->copyFromHostTensor(hostIn);
    std::cout << "copy to device\n" << std::flush;
    
    interp->runSession(sess);
    std::cout << "run\n" << std::flush;
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    std::cout << "out=" << (void*)out << "\n" << std::flush;
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    std::cout << "output copied\n" << std::flush;
    
    auto s = hostOut->shape();
    int n = 1; for (auto d : s) n *= d;
    std::cout << "Output n=" << n << "\n" << std::flush;
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
