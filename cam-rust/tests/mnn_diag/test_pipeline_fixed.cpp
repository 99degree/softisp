#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    auto* interp = Interpreter::createFromFile("test_nocast.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    auto inshape = in->shape();
    std::cout << "Input shape: ["; for (auto d : inshape) std::cout << d << ","; std::cout << "]\n";
    std::cout << "Input dimType: " << (in->getDimensionType() == Tensor::CAFFE ? "CAFFE" : "TF") << "\n";
    auto inType = in->getType();
    std::cout << "Input type: code=" << (int)inType.code << " bits=" << (int)inType.bits << "\n";
    
    int H = 48, W = 64;
    
    // Create CAFFE-format host tensor with FLOAT data
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<float>(), nullptr, Tensor::CAFFE);
    float* fdata = (float*)hostIn->host<void>();
    for (int i = 0; i < H*W; i++) fdata[i] = (float)(i % 256);
    std::cout << "Host input[0..3]: " << fdata[0] << " " << fdata[1] << " " << fdata[2] << " " << fdata[3] << "\n";
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto outType = out->getType();
    std::cout << "Output dimType: " << (out->getDimensionType() == Tensor::CAFFE ? "CAFFE" : "TF") << "\n";
    std::cout << "Output type: code=" << (int)outType.code << " bits=" << (int)outType.bits << "\n";
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Output shape: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
    
    float* d = (float*)hostOut->host<void>();
    std::cout << "First 8: ";
    for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
    std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
    
    int nonZero = 0;
    for (int i = 0; i < n; i++) if (d[i] != 0.0f) nonZero++;
    std::cout << "Non-zero elements: " << nonZero << "/" << n << "\n";
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
