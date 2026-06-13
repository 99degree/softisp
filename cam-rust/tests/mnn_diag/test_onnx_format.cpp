#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("concrete.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    // Create host tensor matching input shape with FLOAT data
    // Use ONNX format for proper NCHW handling
    auto inshape = in->shape();
    std::cout << "Input shape: ["; for (auto d : inshape) std::cout << d << ","; std::cout << "]\n";
    
    int N = inshape[0], C = inshape[1], H = inshape[2], W = inshape[3];
    if (H < 0) { H = 48; W = 64; }
    
    // Create host tensor with ONNX format (NCHW)
    auto* hostIn = Tensor::create({N, C, H, W}, halide_type_of<float>(), Tensor::ONNX);
    float* fdata = (float*)hostIn->host<void>();
    for (int i = 0; i < H*W; i++) fdata[i] = (float)(i % 256);
    std::cout << "Host input[0..3]: " << fdata[0] << " " << fdata[1] << " " << fdata[2] << " " << fdata[3] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    
    // Read output with ONNX format too
    auto* hostOut = new Tensor(out, Tensor::ONNX);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output shape: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
    
    // Read both ways
    float* d = (float*)hostOut->host<void>();
    std::cout << "First 8 (float): ";
    for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
    std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
    
    // Also check: what if we read input data through output to verify data path
    // Try creating a simple identity model: just input → output
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
