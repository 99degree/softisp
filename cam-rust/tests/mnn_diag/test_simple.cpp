#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("test_simple.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    auto inType = in->getType();
    std::cout << "Input code=" << inType.code << " bits=" << (int)inType.bits << "\n";
    
    // Create host tensor matching input type
    auto* hostIn = Tensor::create({1, 3, 224, 224}, inType);
    int total = 3 * 224 * 224;
    float* p = (float*)hostIn->host<void>();
    for (int i = 0; i < total; i++) p[i] = (i % 256) / 255.0f;
    
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
    std::cout << "] n=" << n << " code=" << ty.code << " bits=" << (int)ty.bits << "\n";
    
    float* d = (float*)hostOut->host<void>();
    std::cout << "First 8: ";
    for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
    std::cout << "\nExpected: input * 255.0\n";
    std::cout << "Sample: input[0]=" << p[0] << " expected output=" << p[0]*255.0f << " actual=" << d[0] << "\n";
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
