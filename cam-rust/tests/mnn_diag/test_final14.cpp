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
    printf("D shape empty=%d\n", (int)in->shape().empty()); fflush(stdout);
    
    int H = 48, W = 64;
    // Resize to set shape and type
    interp->resizeTensor(in, {1, 1, H, W});
    printf("E\n"); fflush(stdout);
    interp->resizeSession(sess);
    printf("F\n"); fflush(stdout);
    
    auto t = in->getType();
    printf("G type code=%d bits=%d\n", t.code, t.bits); fflush(stdout);
    
    // Now try copy with the device-reported type
    if (t.bits == 0) { printf("type still invalid\n"); return 1; }
    
    auto* hostIn = Tensor::create({1, 1, H, W}, t, nullptr, Tensor::CAFFE);
    printf("H hostIn=%p\n", hostIn); fflush(stdout);
    if (!hostIn) return 1;
    
    void* data = hostIn->host<void>();
    if (!data) { printf("null data\n"); return 1; }
    if (t.bytes() == 2) {
        uint16_t* p = (uint16_t*)data;
        for (int i = 0; i < H*W; i++) p[i] = (uint16_t)(i % 256);
    } else {
        int32_t* p = (int32_t*)data;
        for (int i = 0; i < H*W; i++) p[i] = (int32_t)(i % 256);
    }
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
        printf("M n=%d", n);
        for (auto d : s) printf(" %d", d);
        printf("\n");
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
