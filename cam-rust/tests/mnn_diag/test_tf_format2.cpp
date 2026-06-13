#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("concrete.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    auto inshape = in->shape();
    std::cout << "Input shape: ["; for (auto d : inshape) std::cout << d << ","; std::cout << "]\n";
    
    // Create host tensor matching input shape
    // Use the same dim format as the model
    int H = 48, W = 64;
    
    // Create with default format (TENSORFLOW per getDimensionType)
    auto* hostIn = Tensor::create({1, H, W, 1});  // creates with TENSORFLOW default
    float* fdata = (float*)hostIn->host<void>();
    std::cout << "HostIn dim type: " << (hostIn->getDimensionType() == Tensor::TENSORFLOW ? "TF" : "CAFFE") << "\n";
    for (int i = 0; i < H*W; i++) fdata[i] = (float)(i % 256);
    std::cout << "Host input[0..3]: " << fdata[0] << " " << fdata[1] << " " << fdata[2] << " " << fdata[3] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    
    // Also try reading output as-is
    auto* hostOut = Tensor::create(out->shape());
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output shape: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
    
    float* d = (float*)hostOut->host<void>();
    std::cout << "First 8 (float): ";
    for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
    std::cout << "\n";
    
    // Compare: what if we write FLOAT data directly using INT32 interpretation?
    // Since halide_type_of<float> returns code=2, and device expects code=2,
    // the types match. But what if we need to use halide_type_of<int32_t> (code=0)?
    std::cout << "\n=== Alternative: host with int32 type ===\n";
    auto* hostIn2 = Tensor::create({1, H, W, 1}); // TENSORFLOW default
    int32_t* idata = (int32_t*)hostIn2->host<void>();
    for (int i = 0; i < H*W; i++) idata[i] = (int32_t)(i % 256);
    std::cout << "Host input[0..3] (int32): " << idata[0] << " " << idata[1] << " " << idata[2] << " " << idata[3] << "\n";
    std::cout << "HostIn2 dim type: " << (hostIn2->getDimensionType() == Tensor::TENSORFLOW ? "TF" : "CAFFE") << "\n";
    
    // Check what type the host tensor reports
    auto hType = hostIn2->getType();
    std::cout << "HostIn2 type: code=" << (int)hType.code << " bits=" << (int)hType.bits << "\n";
    
    // copyFromHostTensor should convert int32→float (if device expects float)
    in->copyFromHostTensor(hostIn2);
    interp->runSession(sess);
    
    auto* hostOut2 = Tensor::create(out->shape());
    out->copyToHostTensor(hostOut2);
    
    float* d2 = (float*)hostOut2->host<void>();
    std::cout << "Output2 First 8 (float): ";
    for (int i = 0; i < 8 && i < n; i++) std::cout << d2[i] << " ";
    std::cout << "\n";
    
    Tensor::destroy(hostIn);
    Tensor::destroy(hostIn2);
    delete hostOut;
    delete hostOut2;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
