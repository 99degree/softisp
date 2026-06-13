#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <cstdio>
using namespace MNN;
int main() {
    printf("1\n");
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    if (!interp) { printf("load fail\n"); return 1; }
    printf("2\n");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    printf("3 sess=%p\n", sess);
    if (!sess) return 1;
    auto* in = interp->getSessionInput(sess, nullptr);
    printf("4 in=%p\n", in);
    if (!in) return 1;
    
    // Use the inType reported by device
    auto inType = in->getType();
    printf("5 inType code=%d bits=%d\n", inType.code, inType.bits);
    
    // If bits is 0, use int32 as default
    halide_type_t useType = inType;
    printf("6\n");
    if (useType.bits == 0) {
        useType.code = (halide_type_code_t)0;
        useType.bits = 32;
        useType.lanes = 1;
        printf("7 override type: code=%d bits=%d\n", useType.code, useType.bits);
    }
    printf("8\n");
    
    int H = 48, W = 64;
    auto* hostIn = Tensor::create({1, 1, H, W}, useType, nullptr, Tensor::CAFFE);
    printf("9 hostIn=%p\n", hostIn);
    if (!hostIn) return 1;
    
    void* data = hostIn->host<void>();
    printf("10 data=%p\n", data);
    if (!data) return 1;
    
    if (useType.bytes() == 2) {
        uint16_t* p = (uint16_t*)data;
        for (int i = 0; i < H*W; i++) p[i] = (uint16_t)(i % 256);
    } else {
        int32_t* p = (int32_t*)data;
        for (int i = 0; i < H*W; i++) p[i] = (int32_t)(i % 256);
    }
    printf("11 data filled\n");
    
    in->copyFromHostTensor(hostIn);
    printf("12 copy OK\n");
    
    interp->runSession(sess);
    printf("13 run OK\n");
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    printf("14 out=%p\n", out);
    if (!out) return 0;
    
    auto* hostOut = new Tensor(out, Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    
    auto s = hostOut->shape();
    int n = 1; for (auto d : s) n *= d;
    printf("15 output n=%d\n", n);
    for (auto d : s) printf(" %d", d);
    printf("\n");
    
    if (n > 0) {
        float* f = (float*)hostOut->host<void>();
        printf("16 first: %f %f %f %f\n", f[0], f[1], f[2], f[3]);
    }
    
    delete hostOut;
    Tensor::destroy(hostIn);
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    printf("17 Success!\n");
    return 0;
}
