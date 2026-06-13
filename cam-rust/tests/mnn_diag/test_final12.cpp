#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <cstdio>
#include <cstdint>
using namespace MNN;
int main() {
    printf("1\n");
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    printf("2\n");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    printf("3\n");
    auto* in = interp->getSessionInput(sess, nullptr);
    printf("4\n");
    
    int H = 48, W = 64;
    // inType is (0,0) - invalid, so we must use explicit type
    // The ONNX model has UINT16 input. On this platform, that's code=1 bits=16
    halide_type_t htype;
    htype.code = (halide_type_code_t)1;
    htype.bits = 16;
    htype.lanes = 1;
    
    printf("5 Trying UINT16 host tensor\n");
    auto* hostIn = Tensor::create({1, 1, H, W}, htype, nullptr, Tensor::CAFFE);
    printf("6 hostIn=%p\n", hostIn);
    if (!hostIn) { printf("null hostIn\n"); return 1; }
    
    uint16_t* p = (uint16_t*)hostIn->host<void>();
    printf("7 host data=%p\n", p);
    if (!p) { printf("null data\n"); return 1; }
    for (int i = 0; i < H*W; i++) p[i] = (uint16_t)(i % 256);
    printf("8 data filled\n");
    
    in->copyFromHostTensor(hostIn);
    printf("9 copy OK\n");
    
    interp->runSession(sess);
    printf("10 run OK\n");
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    printf("11 out=%p\n", out);
    
    if (out) {
        auto* hostOut = new Tensor(out, Tensor::CAFFE);
        out->copyToHostTensor(hostOut);
        printf("12 copy out OK\n");
        auto s = hostOut->shape();
        int n = 1; for (auto d : s) n *= d;
        printf("13 output n=%d\n  shape:", n);
        for (auto d : s) printf(" %d", d);
        printf("\n");
        if (n > 0) {
            float* f = (float*)hostOut->host<void>();
            printf("14 first: %f %f %f %f\n", f[0], f[1], f[2], f[3]);
            int nz = 0;
            for (int i = 0; i < n && i < 100; i++) if (f[i] != 0.0f) nz++;
            printf("15 non-zero: %d/100\n", nz);
        }
        delete hostOut;
    }
    
    Tensor::destroy(hostIn);
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    printf("16 Success!\n");
    return 0;
}
