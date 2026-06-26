// test_pipeline_opset.cpp — Full 6-stage ISP pipeline via IspPipelineBuilder
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <chrono>
#include <functional>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "isp_opset.h"

static std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.good()) return {};
    size_t sz = f.tellg(); f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

struct SpvSource {
    const char* path;
    std::function<isp::OpDesc(int,int)> desc_fn;
} stages[6] = {
    {"shader1_unpack_blc.spv", isp::UnpackBlc},
    {"shader2_demosaic_ccm.spv", isp::DemosaicCcm},
    {"shader3_fcs.spv", [](int w,int h){return isp::Fcs(w,h);}},
    {"shader4_ee.spv", [](int w,int h){return isp::Ee(w,h);}},
    {"shader5_ldci.spv", [](int w,int h){return isp::Ldci(w,h);}},
    {"shader6_display_simple.spv", [](int w,int h){return isp::Display(w,h);}},
};

int main(int argc, char** argv) {
    int W = argc > 1 ? atoi(argv[1]) : 480;
    int H = argc > 2 ? atoi(argv[2]) : 360;
    printf("Pipeline: %dx%d\n", W, H);

    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";

    // Load SPIR-V for each opset
    std::vector<std::vector<int8_t>> spv(6);
    for (int i = 0; i < 6; i++) {
        auto raw = readFile((base + stages[i].path).c_str());
        if (raw.empty()) { fprintf(stderr, "Missing: %s\n", stages[i].path); return 1; }
        spv[i].resize(raw.size());
        memcpy(spv[i].data(), raw.data(), raw.size());
        printf("  [%d] %s: %zu bytes\n", i, stages[i].path, raw.size());
    }

    // Build pipeline via IspPipelineBuilder
    isp::IspPipelineBuilder pipe;
    for (int i = 0; i < 6; i++)
        pipe.addStage(stages[i].desc_fn(W, H), spv[i]);

    size_t model_size;
    const uint8_t* model_data = pipe.build(&model_size);
    printf("Model: %zu bytes\n", model_size);

    // Run
    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();

    auto interp = MNN::Interpreter::createFromBuffer(model_data, model_size);
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    cfg.numThread = 1;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;

    auto startT = std::chrono::high_resolution_clock::now();
    auto sess = interp->createSession(cfg);
    printf("Session create: %.1f ms\n",
           std::chrono::duration<double, std::milli>(
               std::chrono::high_resolution_clock::now() - startT).count());

    auto input = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(input, {1, 1, H, W});
    interp->resizeSession(sess);

    // Synthetic Bayer pattern
    std::vector<int32_t> inData(W * H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            inData[y*W + x] = 100 + (x + y) % 200;

    auto hostIn = MNN::Tensor::create({1, 1, H, W}, input->getType(), inData.data(), MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);

    // Warmup + benchmark
    interp->runSession(sess);
    auto run_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10; i++) interp->runSession(sess);
    auto run_end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(run_end - run_start).count() / 10.0;

    // Verify
    auto out = interp->getSessionOutput(sess, pipe.outputTensorName().c_str());
    float* outData = new float[out->elementSize()]();
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++) if (fabsf(outData[i]) > 1e-6f) nz++;

    printf("Result: %d/%d non-zero | %.2f ms/frame (%.0f FPS)%s\n",
           nz, (int)out->elementSize(), ms, 1000.0/ms,
           ms < 30.0 ? " ✅ TARGET <30ms" : " ⚠️  >30ms");

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
