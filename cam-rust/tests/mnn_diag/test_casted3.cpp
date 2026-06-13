#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    std::cout << "Starting...\n";
    auto* interp = Interpreter::createFromFile("casted.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    std::cout << "Loaded\n";
    
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    std::cout << "Session created\n";
    
    auto* in = interp->getSessionInput(sess, nullptr);
    if (!in) { std::cerr << "no input\n"; return 1; }
    std::cout << "Input OK\n";
    
    int H = 4, W = 4;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    std::cout << "Resized\n";
    
    auto inType = in->getType();
    std::cout << "Input type: code=" << (int)inType.code << " bits=" << (int)inType.bits << "\n";
    
    // Create host tensor
    auto* hostIn = Tensor::create({1, 1, H, W}, inType, nullptr, Tensor::CAFFE);
    if (!hostIn) { std::cerr << "hostIn null\n"; return 1; }
    std::cout << "hostIn host: " << (void*)hostIn->host<void>() << "\n";
    
    // Write data
    float* d = (float*)hostIn->host<void>();
    if (!d) { std::cerr << "host null!\n"; return 1; }
    for (int i = 0; i < H*W; i++) d[i] = (float)(i % 256);
    std::cout << "Wrote data, first=" << d[0] << "\n";
    
    in->copyFromHostTensor(hostIn);
    std::cout << "Copied to device\n";
    
    interp->runSession(sess);
    std::cout << "Run session\n";
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    if (!out) { std::cerr << "no output\n"; return 1; }
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    std::cout << "HostOut created\n";
    
    out->copyToHostTensor(hostOut);
    std::cout << "Copied from device\n";
    
    auto s = hostOut->shape();
    int n = 1; for (auto d : s) n *= d;
    std::cout << "Output n=" << n;
    for (auto d : s) std::cout << " " << d;
    std::cout << "\n";
    
    if (n > 0) {
        float* outd = (float*)hostOut->host<void>();
        std::cout << "First: " << outd[0] << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
