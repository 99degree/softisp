#include <MNN/Interpreter.hpp>
#include <cstdio>
#include <cstdint>
using namespace MNN;
int main() {
    printf("A\n"); fflush(stdout);
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    printf("B\n"); fflush(stdout);
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    printf("C\n"); fflush(stdout);
    auto* in = interp->getSessionInput(sess, nullptr);
    printf("D\n"); fflush(stdout);
    
    int H = 48, W = 64;
    printf("E\n"); fflush(stdout);
    
    // Use halide_type_t constructor
    halide_type_t htype((halide_type_code_t)1, 16);  // UINT16
    printf("F htype code=%d bits=%d\n", htype.code, htype.bits); fflush(stdout);
    
    auto* hostIn = Tensor::create({1, 1, H, W}, htype, nullptr, Tensor::CAFFE);
    printf("G hostIn=%p\n", hostIn); fflush(stdout);
    if (!hostIn) return 1;
    
    uint16_t* p = (uint16_t*)hostIn->host<void>();
    printf("H p=%p\n", p); fflush(stdout);
    if (!p) return 1;
    for (int i = 0; i < H*W; i++) p[i] = (uint16_t)(i % 256);
    printf("I\n"); fflush(stdout);
    
    in->copyFromHostTensor(hostIn);
    printf("J\n"); fflush(stdout);
    
    interp->runSession(sess);
    printf("K\n"); fflush(stdout);
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    printf("L out=%p\n", out); fflush(stdout);
    
    if (out) {
        auto* hostOut = new Tensor(out, Tensor::CAFFE);
        out->copyToHostTensor(hostOut);
        auto s = hostOut->shape();
        int n = 1; for (auto d : s) n *= d;
        printf("M n=%d\n", n);
        if (n > 0) {
            float* f = (float*)hostOut->host<void>();
            printf("N first: %f %f %f %f\n", f[0], f[1], f[2], f[3]);
        }
        delete hostOut;
    }
    
    Tensor::destroy(hostIn);
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    printf("Done\n");
    return 0;
}
