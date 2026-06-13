#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <iostream>
using namespace MNN;
int main() {
    std::cout << "Loading...\n" << std::flush;
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    int H = 48, W = 64;
    
    // Try different types
    halide_type_t types[] = {
        halide_type_type{(halide_type_code_t)0, 32, 1}, // INT32  
        halide_type_type{(halide_type_code_t)1, 16, 1}, // UINT16
        halide_type_type{(halide_type_code_t)2, 32, 1}, // FLOAT on this platform
        halide_type_type{(halide_type_code_t)0, 16, 1}, // INT16
    };
    const char* typeNames[] = {"INT32", "UINT16", "FLOAT(0,32)", "INT16"};
    
    for (int t = 0; t < 4; t++) {
        printf("Trying type[%d] %s: code=%d bits=%d\n", t, typeNames[t], types[t].code, types[t].bits);
        auto* hostIn = Tensor::create({1, 1, H, W}, types[t], nullptr, Tensor::CAFFE);
        if (!hostIn) { printf("  null\n"); continue; }
        void* data = hostIn->host<void>();
        if (!data) { printf("  null data\n"); Tensor::destroy(hostIn); continue; }
        if (types[t].bytes() == 2) {
            uint16_t* p = (uint16_t*)data;
            for (int i = 0; i < H*W; i++) p[i] = (uint16_t)(i % 256);
        } else {
            int32_t* p = (int32_t*)data;
            for (int i = 0; i < H*W; i++) p[i] = (int32_t)(i % 256);
        }
        in->copyFromHostTensor(hostIn);
        printf("  copyFromHostTensor OK\n");
        
        interp->runSession(sess);
        printf("  runSession OK\n");
        
        auto* out = interp->getSessionOutput(sess, nullptr);
        if (out) {
            auto* hostOut = new Tensor(out, Tensor::CAFFE);
            out->copyToHostTensor(hostOut);
            auto s = hostOut->shape();
            int n = 1; for (auto d : s) n *= d;
            printf("  output n=%d\n", n);
            delete hostOut;
        }
        
        Tensor::destroy(hostIn);
        break; // Only try first type for now
    }
    
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    printf("Done\n");
    return 0;
}
