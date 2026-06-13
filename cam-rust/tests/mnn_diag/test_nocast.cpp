#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("test_nocast.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed\n"; return 1; }
    
    auto* in = interp->getSessionInput(sess, nullptr);
    int W = 64, H = 48;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    auto inType = in->getType();
    std::cout << "Input type code=" << inType.code << " bits=" << (int)inType.bits << " bytes=" << inType.bytes() << "\n";
    
    auto* hostIn = Tensor::create({1, 1, H, W}, inType);
    int total = H * W;
    auto* p = (int32_t*)hostIn->host<void>();
    for (int i = 0; i < total; i++) p[i] = (i * 256 / W) % 256;
    std::cout << "INT32: first=" << p[0] << " last=" << p[total-1] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    auto ty = hostOut->getType();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output: [" << s[0] << "," << s[1] << "," << s[2] << "," << s[3] << "] n=" << n
              << " type code=" << ty.code << " bits=" << (int)ty.bits << " bytes=" << ty.bytes() << "\n";
    
    // Read raw bytes
    auto* raw = (uint8_t*)hostOut->host<void>();
    std::cout << "First 32 bytes: ";
    for (int i = 0; i < 32 && i < n * ty.bytes(); i++) std::cout << (int)raw[i] << " ";
    std::cout << "\n";
    
    // Try as float (4 bytes)
    if (ty.bytes() == 4) {
        auto* d = (float*)raw;
        std::cout << "As float: ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\n";
    }
    
    // Try as int32
    if (ty.bytes() == 4) {
        auto* d = (int32_t*)raw;
        std::cout << "As int32: ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
