#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <iostream>
using namespace MNN;
int main() {
    printf("Loading test_final2.mnn...\n");
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    int H = 48, W = 64;
    
    // Try UINT16 (code=1, bits=16) — matches the ONNX input type
    printf("Trying UINT16 (code=1 bits=16)...\n");
    halide_type_t ut16{(halide_type_code_t)1, 16, 1};
    auto* hostIn = Tensor::create({1, 1, H, W}, ut16, nullptr, Tensor::CAFFE);
    if (hostIn) {
        uint16_t* p = (uint16_t*)hostIn->host<void>();
        if (p) {
            for (int i = 0; i < H*W; i++) p[i] = (uint16_t)(i % 256);
            printf("  data written\n");
            in->copyFromHostTensor(hostIn);
            printf("  copy OK\n");
            interp->runSession(sess);
            printf("  run OK\n");
            auto* out = interp->getSessionOutput(sess, nullptr);
            if (out) {
                auto* hostOut = new Tensor(out, Tensor::CAFFE);
                out->copyToHostTensor(hostOut);
                auto s = hostOut->shape();
                int n = 1; for (auto d : s) n *= d;
                printf("  output n=%d\n", n);
                if (n > 0) {
                    float* f = (float*)hostOut->host<void>();
                    printf("  first: %f %f %f %f\n", f[0], f[1], f[2], f[3]);
                }
                delete hostOut;
            }
        }
        Tensor::destroy(hostIn);
    }
    
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    printf("Done\n");
    return 0;
}
