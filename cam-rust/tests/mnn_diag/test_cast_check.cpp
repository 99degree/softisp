#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("casted.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    auto inshape = in->shape();
    std::cout << "Input shape: ["; for (auto d : inshape) std::cout << d << ","; std::cout << "]\n";
    std::cout << "Input dimType: " << (in->getDimensionType() == Tensor::TENSORFLOW ? "TF" : "CAFFE") << "\n";
    
    int H = 48, W = 64;
    
    // Create host with CAFFE format (NCHW), explicitly pass halide_type_of<float>
    // halide_type_of<float> returns code=2 (FLOAT on this platform)
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<float>(), nullptr, Tensor::CAFFE);
    float* fdata = (float*)hostIn->host<void>();
    std::cout << "HostIn dimType: " << (hostIn->getDimensionType() == Tensor::TENSORFLOW ? "TF" : "CAFFE") << "\n";
    for (int i = 0; i < H*W; i++) fdata[i] = (float)(i % 256);
    std::cout << "Host input[0..3]: " << fdata[0] << " " << fdata[1] << " " << fdata[2] << " " << fdata[3] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    std::cout << "Output dimType: " << (out->getDimensionType() == Tensor::TENSORFLOW ? "TF" : "CAFFE") << "\n";
    
    // Try both CAFFE and TENSORFLOW for output reading
    for (int fmt = 0; fmt <= 1; fmt++) {
        auto outFmt = (fmt == 0) ? Tensor::CAFFE : Tensor::TENSORFLOW;
        auto* hostOut = new Tensor(out, outFmt);
        out->copyToHostTensor(hostOut);
        
        auto s = hostOut->shape();
        int n = 1;
        for (auto d : s) n *= d;
        std::cout << "Output (fmt=" << (outFmt == Tensor::CAFFE ? "CAFFE" : "TF") << ") shape: [";
        for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
        
        float* d = (float*)hostOut->host<void>();
        std::cout << "  First 8: ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\n";
        std::cout << "  Expected: 0 255 510 765 1020 1275 1530 1785\n";
        
        delete hostOut;
    }
    
    Tensor::destroy(hostIn);
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
