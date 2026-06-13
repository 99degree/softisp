#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("test_nocast.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    int H = 48, W = 64;
    
    // RESIZE the input tensor to concrete dims
    in->resize({1, 1, H, W});
    interp->resizeSession(sess);
    
    auto inshape = in->shape();
    std::cout << "Input shape: ["; for (auto d : inshape) std::cout << d << ","; std::cout << "]\n";
    auto inType = in->getType();
    std::cout << "Input type: code=" << (int)inType.code << " bits=" << (int)inType.bits << "\n";
    // code=0 on this platform = INT32 (since float=2, int=0)
    
    // Since device expects INT32 (code=0), create host tensor with INT32 data
    // BUT use CAFFE format for proper NCHW handling
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<int32_t>(), nullptr, Tensor::CAFFE);
    int32_t* idata = (int32_t*)hostIn->host<void>();
    for (int i = 0; i < H*W; i++) idata[i] = (int32_t)(i % 256);
    std::cout << "Host input[0..3]: " << idata[0] << " " << idata[1] << " " << idata[2] << " " << idata[3] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    std::cout << "Output shape: ["; for (auto d : out->shape()) std::cout << d << ","; std::cout << "]\n";
    auto outType = out->getType();
    std::cout << "Output type: code=" << (int)outType.code << " bits=" << (int)outType.bits << "\n";
    
    // Use CAFFE to read output
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "HostOut shape: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
    
    if (n > 0) {
        auto hostType = hostOut->getType();
        std::cout << "HostOut type: code=" << (int)hostType.code << " bits=" << (int)hostType.bits << "\n";
        
        // Read as float
        float* d = (float*)hostOut->host<void>();
        std::cout << "First 8 (float): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
        std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
        
        // As int32
        int32_t* di = (int32_t*)d;
        std::cout << "First 8 (int32): ";
        for (int i = 0; i < 8 && i < n; i++) std::cout << di[i] << " ";
        std::cout << "\n";
    }
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
