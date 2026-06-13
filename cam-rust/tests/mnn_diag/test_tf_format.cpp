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
    std::cout << "Input inside type: code=" << (int)in->getType().code << "\n";
    
    // Check what dimension format the model uses
    auto dimType = in->getDimensionType();
    std::cout << "Input dim type: " << (dimType == Tensor::TENSORFLOW ? "TENSORFLOW" : "CAFFE") << "\n";
    
    int H = 48, W = 64;
    
    // Create host tensor with TENSORFLOW format (NHWC) to match model
    auto* hostIn = Tensor::create({1, H, W, 1}, halide_type_of<float>(), Tensor::TENSORFLOW);
    float* fdata = (float*)hostIn->host<void>();
    for (int i = 0; i < H*W; i++) fdata[i] = (float)(i % 256);
    std::cout << "Host input[0..3]: " << fdata[0] << " " << fdata[1] << " " << fdata[2] << " " << fdata[3] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto outDimType = out->getDimensionType();
    std::cout << "Output dim type: " << (outDimType == Tensor::TENSORFLOW ? "TENSORFLOW" : "CAFFE") << "\n";
    
    // Read output with TENSORFLOW format too
    auto* hostOut = new Tensor(out, Tensor::TENSORFLOW);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output shape: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
    
    float* d = (float*)hostOut->host<void>();
    std::cout << "First 8 (float): ";
    for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
    std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
