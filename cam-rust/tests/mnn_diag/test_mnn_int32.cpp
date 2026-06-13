#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    const char* path = "/data/data/com.termux/files/home/test_pipeline_fixed.mnn";
    auto* interp = Interpreter::createFromFile(path);
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    int W = 64, H = 48;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    auto inType = in->getType();
    std::cout << "Input type code=" << inType.code << " bits=" << (int)inType.bits << std::endl;
    std::cout << "int32_t: code=" << halide_type_of<int32_t>().code << " bits=" << (int)halide_type_of<int32_t>().bits << std::endl;
    std::cout << "int16_t: code=" << halide_type_of<int16_t>().code << " bits=" << (int)halide_type_of<int16_t>().bits << std::endl;
    std::cout << "int:     code=" << halide_type_of<int>().code << " bits=" << (int)halide_type_of<int>().bits << std::endl;
    
    // Create host tensor with raw shape and type
    auto* hostIn = Tensor::create({1, 1, H, W}, inType);
    int total = H * W;
    
    // Fill with data based on element size
    int elemSize = inType.bytes();
    std::cout << "Element size: " << elemSize << std::endl;
    
    if (elemSize == 4) {
        auto* ptr = (int32_t*)hostIn->host<void>();
        for (int i = 0; i < total; i++) ptr[i] = i * 256 / W;
        std::cout << "Filled as INT32" << std::endl;
    } else if (elemSize == 2) {
        auto* ptr = (int16_t*)hostIn->host<void>();
        for (int i = 0; i < total; i++) ptr[i] = (int16_t)(i * 256 / W);
        std::cout << "Filled as INT16" << std::endl;
    } else {
        std::cerr << "Unexpected element size" << std::endl;
        return 1;
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
    
    std::cout << "Output: [" << s[0] << "," << s[1] << "," << s[2] << "," << s[3] << "] n=" << n << std::endl;
    
    if (ty == halide_type_of<uint8_t>()) {
        auto* d = hostOut->host<uint8_t>();
        std::cout << "First 32 (uint8): ";
        for (int i = 0; i < 32 && i < n; i++) std::cout << (int)d[i] << " ";
    } else if (ty == halide_type_of<float>()) {
        auto* d = hostOut->host<float>();
        std::cout << "First 16 (float): ";
        for (int i = 0; i < 16 && i < n; i++) std::cout << d[i] << " ";
    } else {
        auto* d = (uint8_t*)hostOut->host<void>();
        std::cout << "First 32 (raw): ";
        for (int i = 0; i < 32 && i < n; i++) std::cout << (int)d[i] << " ";
    }
    std::cout << std::endl;
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
