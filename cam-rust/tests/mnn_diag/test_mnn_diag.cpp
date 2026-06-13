#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
#include <vector>
#include <map>
using namespace MNN;

int main() {
    const char* path = "/data/data/com.termux/files/home/test_pipeline_fixed.mnn";
    
    auto* interp = Interpreter::createFromFile(path);
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    
    // Dump all I/O tensors
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    auto inShape = in->shape();
    std::cout << "Input: [" << inShape[0] << "," << inShape[1] << "," << inShape[2] << "," << inShape[3] << "] "
              << "type code=" << inType.code << " bits=" << (int)inType.bits << std::endl;
    
    // Now test with real data
    int W = 64, H = 48;
    interp->resizeTensor(in, {1, 1, H, W});
    interp->resizeSession(sess);
    
    // Fill with INT16 data: simple ramp
    auto* hostIn = Tensor::create({1, 1, H, W}, halide_type_of<int16_t>());
    auto* ptr = hostIn->host<int16_t>();
    for (int i = 0; i < H * W; i++) {
        ptr[i] = (int16_t)(i * 256 / W);  // gradient 0..191
    }
    
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    // Get named output
    auto* out = interp->getSessionOutput(sess, "DisplayBlock/frame");
    if (!out) {
        // Try nullptr (default output)
        out = interp->getSessionOutput(sess, nullptr);
    }
    if (!out) { std::cerr << "no output" << std::endl; return 1; }
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    auto s = hostOut->shape();
    auto ty = hostOut->getType();
    int total = 1;
    for (auto d : s) total *= d;
    
    std::cout << "Output shape: [" << s[0] << "," << s[1] << "," << s[2] << "," << s[3] << "] "
              << "total=" << total << " type code=" << ty.code << " bits=" << (int)ty.bits << std::endl;
    
    if (ty == halide_type_of<uint8_t>()) {
        auto* d = hostOut->host<uint8_t>();
        std::cout << "First 32 (uint8): ";
        for (int i = 0; i < 32 && i < total; i++) std::cout << (int)d[i] << " ";
        // Also show some middle values
        std::cout << std::endl << "Mid 8 at 1500: ";
        for (int i = 1500; i < 1508 && i < total; i++) std::cout << (int)d[i] << " ";
    } else if (ty == halide_type_of<float>()) {
        auto* d = hostOut->host<float>();
        std::cout << "First 16 (float): ";
        for (int i = 0; i < 16 && i < total; i++) std::cout << d[i] << " ";
    } else {
        std::cout << "Unhandled type code=" << ty.code << " bits=" << (int)ty.bits << std::endl;
    }
    std::cout << std::endl;
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
