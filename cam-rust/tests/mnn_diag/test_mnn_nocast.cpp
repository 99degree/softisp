#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    const char* path = "/tmp/test_pipeline_nocast.mnn";
    auto* interp = Interpreter::createFromFile(path);
    if (!interp) { std::cerr << "load failed"; return 1; }
    
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed"; return 1; }
    
    auto* in = interp->getSessionInput(sess, nullptr);
    int W = 64, H = 48;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    auto inType = in->getType();
    std::cout << "Input type code=" << inType.code << " bits=" << (int)inType.bits << " bytes=" << inType.bytes() << std::endl;
    
    auto* hostIn = Tensor::create({1, 1, H, W}, inType);
    int total = H * W;
    if (inType.bytes() == 2) {
        auto* p = (int16_t*)hostIn->host<void>();
        for (int i = 0; i < total; i++) p[i] = (int16_t)(i * 256 / W);
        std::cout << "Filled as INT16" << std::endl;
    } else if (inType.bytes() == 4) {
        auto* p = (int32_t*)hostIn->host<void>();
        for (int i = 0; i < total; i++) p[i] = i * 256 / W;
        std::cout << "Filled as INT32" << std::endl;
    } else {
        auto* p = (uint8_t*)hostIn->host<void>();
        for (int i = 0; i < total; i++) p[i] = (uint8_t)(i * 256 / W);
        std::cout << "Filled as UINT8 bytes=" << inType.bytes() << std::endl;
    }
    
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
              << " type code=" << ty.code << " bits=" << (int)ty.bits << std::endl;
    
    if (ty == halide_type_of<float>() || (ty.bytes() == 4 && ty.code == 0)) {
        auto* d = (float*)hostOut->host<void>();
        std::cout << "First 16 (float): ";
        for (int i = 0; i < 16 && i < n; i++) std::cout << d[i] << " ";
        std::cout << std::endl;
        // Check if non-zero
        bool allZero = true;
        for (int i = 0; i < n && i < 100; i++) if (d[i] != 0.0f) { allZero = false; break; }
        std::cout << "All zero: " << (allZero ? "YES" : "NO") << std::endl;
    } else {
        auto* d = (uint8_t*)hostOut->host<void>();
        std::cout << "First 32 (raw): ";
        for (int i = 0; i < 32 && i < n; i++) std::cout << (int)d[i] << " ";
        std::cout << std::endl;
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
