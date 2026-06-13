#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("test_simple.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    auto inType = in->getType();
    std::cout << "Model expects: code=" << inType.code << " bits=" << (int)inType.bits << " bytes=" << inType.bytes() << "\n";
    
    // Create host tensor with FLOAT (matches our data) — MNN will convert
    auto* hostIn = Tensor::create({1, 3, 4, 4}, halide_type_of<float>());  // small test
    int total = 3*4*4;
    float* p = (float*)hostIn->host<void>();
    for (int i = 0; i < total; i++) p[i] = (float)(i % 16);  // 0..15
    
    in->copyFromHostTensor(hostIn);
    interp->resizeTensor(in, {1, 3, 4, 4});
    interp->resizeSession(sess);
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    auto ty = hostOut->getType();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output shape: [";
    for (auto d : s) std::cout << d << ",";
    std::cout << "] n=" << n << "\n";
    std::cout << "Output type: code=" << ty.code << " bits=" << (int)ty.bits << "\n";
    
    // Check output
    if (ty.bytes() == 4) {
        int32_t* d = (int32_t*)hostOut->host<void>();
        float* df = (float*)hostOut->host<void>();
        std::cout << "First 8 (int32): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\nFirst 8 (float): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << df[i] << " ";
        std::cout << "\nExpected (input 0..15 * 255): 0 255 510 765 1020 1275 1530 1785\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
