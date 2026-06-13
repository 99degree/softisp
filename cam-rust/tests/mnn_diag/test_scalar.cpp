#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("test_scalar.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Input type code=" << inType.code << " bits=" << (int)inType.bits << "\n";
    
    // Create FLOAT host tensor
    auto* hostIn = Tensor::create({1, 1, 2, 2}, halide_type_of<float>());
    float* p = (float*)hostIn->host<void>();
    p[0] = 3.0f; p[1] = 0.5f; p[2] = -1.0f; p[3] = 7.0f;
    std::cout << "Input: " << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    auto ty = hostOut->getType();
    std::cout << "Output shape: [" << s[0] << "," << s[1] << "," << s[2] << "," << s[3] << "]\n";
    std::cout << "Output type code=" << ty.code << " bits=" << (int)ty.bits << "\n";
    
    // Read as raw bytes
    uint8_t* raw = (uint8_t*)hostOut->host<void>();
    std::cout << "Raw bytes: ";
    for (int i = 0; i < 16; i++) std::cout << (int)raw[i] << " ";
    std::cout << "\n";
    
    // Read as float (if 4-byte)
    if (ty.bytes() == 4) {
        float* d = (float*)raw;
        std::cout << "As float: " << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << "\n";
        int32_t* di = (int32_t*)raw;
        std::cout << "As int32: " << di[0] << " " << di[1] << " " << di[2] << " " << di[3] << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
