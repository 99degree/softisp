#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <iostream>
using namespace MNN;

int main() {
    // Check type codes on this platform
    std::cout << "=== Type codes on this platform ===\n";
    auto tf = halide_type_of<float>();
    auto ti = halide_type_of<int32_t>();
    auto ti16 = halide_type_of<int16_t>();
    std::cout << "halide_type_of<float>: code=" << (int)tf.code << " bits=" << (int)tf.bits << "\n";
    std::cout << "halide_type_of<int32>: code=" << (int)ti.code << " bits=" << (int)ti.bits << "\n";
    std::cout << "halide_type_of<int16>: code=" << (int)ti16.code << " bits=" << (int)ti16.bits << "\n";
    
    // MNN standard codes (from MNN's HalideRuntime.h):
    // halide_type_float = 0, halide_type_int = 2, halide_type_uint = 1
    // On this platform: float=2, int32=0, int16=0 -- seems like they're swapped!
    // So code=2 means FLOAT and code=0 means INT32 on this platform
    
    std::cout << "\n=== Testing concrete.mnn (FLOAT input * 255.0) ===\n";
    auto* interp = Interpreter::createFromFile("concrete.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 4;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    auto inType = in->getType();
    std::cout << "Device input type: code=" << (int)inType.code << " bits=" << (int)inType.bits << " bytes=" << inType.bytes() << "\n";
    std::cout << "Device input type = FLOAT (code=2 on this platform)\n";
    
    // Since device expects FLOAT (code=2), and halide_type_of<float> returns code=2,
    // create host tensor with halide_type_of<float> and write float data
    auto* hostIn = Tensor::create({1, 1, 48, 64}, halide_type_of<float>());
    float* fdata = (float*)hostIn->host<void>();
    for (int i = 0; i < 48*64; i++) fdata[i] = (float)(i % 256);
    std::cout << "Host input: " << fdata[0] << " " << fdata[1] << " " << fdata[2] << " " << fdata[3] << "\n";
    
    // copyFromHostTensor: host=code=2 (float), device=code=2 (float), same type = memcpy
    in->copyFromHostTensor(hostIn);
    interp->runSession(sess);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    auto outType = out->getType();
    std::cout << "Output type: code=" << (int)outType.code << " bits=" << (int)outType.bits << "\n";
    std::cout << "Output type = FLOAT (code=2)\n";
    
    // Read output. Since output type is code=2 (FLOAT), use float* to read
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto hOutType = hostOut->getType();
    std::cout << "HostOut type: code=" << (int)hOutType.code << " bits=" << (int)hOutType.bits << "\n";
    
    auto s = hostOut->shape();
    int n = 1;
    for (auto d : s) n *= d;
    std::cout << "Shape: ["; for (auto d : s) std::cout << d << ","; std::cout << "] n=" << n << "\n";
    
    float* d = (float*)hostOut->host<void>();
    std::cout << "First 8 as float: ";
    for (int i = 0; i < 8 && i < n; i++) std::cout << d[i] << " ";
    std::cout << "\nExpected: 0 255 510 765 1020 1275 1530 1785\n";
    
    Tensor::destroy(hostIn);
    delete hostOut;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
