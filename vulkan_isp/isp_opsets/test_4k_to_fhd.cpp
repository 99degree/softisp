// test_4k_to_fhd.cpp — 4K Bayer → 1920×1080 FHD ISP Pipeline
//
// Pipeline flow:
//   3840×2160 Bayer INT32
//       │
//   isp.unpack_blc (w=3840,h=2160) → 4×RGGB at 1920×1080 (FHD)
//       │
//   isp.demosaic_noscale (w=1920,h=1080) → 3×RGB at 1920×1080 (no upscale)
//       │
//   isp.fcs → isp.ee → isp.ldci → isp.display (all at 1920×1080)
//       │
//   Output: FHD RGB float 1920×1080×3
//
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
    if (!f.good()) { fprintf(stderr, "Missing: %s\n", path); return {}; }
    size_t sz = f.tellg(); f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

int main() {
    const int BAYER_W = 3840, BAYER_H = 2160;  // 4K UHD
    const int FHD_W = BAYER_W / 2, FHD_H = BAYER_H / 2;  // 1920×1080

    printf("╔══════════════════════════════════════════╗\n");
    printf("║  4K Bayer → FHD RGB ISP Pipeline        ║\n");
    printf("╠══════════════════════════════════════════╣\n");
    printf("║ Input:  %4dx%-4d Bayer (%d MP)         ║\n", BAYER_W, BAYER_H, BAYER_W*BAYER_H/1000000);
    printf("║ Unpack: 4×RGGB at %dx%d                ║\n", FHD_W, FHD_H);
    printf("║ Output: 3×RGB  at %dx%d (FHD)          ║\n", FHD_W, FHD_H);
    printf("╚══════════════════════════════════════════╝\n\n");

    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";

    // Load SPIR-V for all 6 stages + demosaic_noscale
    auto spv_unpack = readFile((base + "shader1_unpack_blc.spv").c_str());
    auto spv_demo   = readFile((base + "isp_opsets/demosaic_noscale.spv").c_str());
    auto spv_fcs    = readFile((base + "shader3_fcs.spv").c_str());
    auto spv_ee     = readFile((base + "shader4_ee.spv").c_str());
    auto spv_ldci   = readFile((base + "shader5_ldci.spv").c_str());
    auto spv_disp   = readFile((base + "shader6_display_simple.spv").c_str());

    auto to_i8 = [](const std::vector<uint8_t>& raw) {
        std::vector<int8_t> v(raw.size());
        memcpy(v.data(), raw.data(), raw.size());
        return v;
    };

    // Build pipeline using IspPipelineBuilder
    // Unpack uses 4K dimensions; remaining stages use FHD dimensions
    isp::IspPipelineBuilder pipe(FHD_W, FHD_H, 6);

    // Stage 0: Unpack at 4K → FHD-size RGGB
    pipe.addStage(0, isp::UnpackBlc(BAYER_W, BAYER_H), to_i8(spv_unpack));
    // Stage 1: Demosaic (no upscale) at FHD
    pipe.addStage(1, isp::DemosaicNoscale(FHD_W, FHD_H), to_i8(spv_demo));
    // Stages 2-5 at FHD
    pipe.addStage(2, isp::Fcs(FHD_W, FHD_H), to_i8(spv_fcs));
    pipe.addStage(3, isp::Ee(FHD_W, FHD_H), to_i8(spv_ee));
    pipe.addStage(4, isp::Ldci(FHD_W, FHD_H), to_i8(spv_ldci));
    pipe.addStage(5, isp::Display(FHD_W, FHD_H), to_i8(spv_disp));

    size_t model_size;
    const uint8_t* model_data = pipe.build(&model_size);
    printf("Model: %zu bytes (%d opsets)\n\n", model_size, 6);

    // Init Vulkan backend
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

    double t0 = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    auto sess = interp->createSession(cfg);
    double t1 = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    printf("Session create: %.1f ms\n", t1 - t0);

    auto input = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(input, {1, 1, BAYER_H, BAYER_W});
    interp->resizeSession(sess);

    // Fill 4K Bayer with synthetic pattern
    printf("Generating %dx%d Bayer (%.1f MB)...\n", BAYER_W, BAYER_H,
           (double)BAYER_W*BAYER_H*4/1048576.0);
    std::vector<int32_t> inData(BAYER_W * BAYER_H);
    for (int y = 0; y < BAYER_H; y++)
        for (int x = 0; x < BAYER_W; x++) {
            if      (y%2==0 && x%2==0) inData[y*BAYER_W+x] = 100 + (x+y)*0;  // R
            else if (y%2==0 && x%2==1) inData[y*BAYER_W+x] = 200 + (x+y)*0;  // Gr
            else if (y%2==1 && x%2==0) inData[y*BAYER_W+x] = 300 + (x+y)*0;  // Gb
            else                       inData[y*BAYER_W+x] = 400 + (x+y)*0;  // B
        }

    printf("Uploading to GPU...\n");
    auto hostIn = MNN::Tensor::create({1, 1, BAYER_H, BAYER_W}, input->getType(), inData.data(), MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);

    // Warmup
    printf("Warmup...\n");
    interp->runSession(sess);

    // Benchmark
    const int ITERS = 10;
    printf("Benchmark (%d runs)...\n", ITERS);
    auto run_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < ITERS; i++) interp->runSession(sess);
    auto run_end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(run_end - run_start).count() / (double)ITERS;

    // Readback FHD output
    auto out = interp->getSessionOutput(sess, "tensor_6");
    printf("Output: [");
    for (int d = 0; d < out->shape().size(); d++)
        printf("%d%s", out->shape()[d], d+1<out->shape().size()?",":"");
    printf("] = %d elements\n", (int)out->elementSize());

    size_t rb_bytes = out->elementSize() * sizeof(float);
    printf("Readback %.1f MB...\n", rb_bytes / 1048576.0);
    t0 = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    float* outData = new float[out->elementSize()];
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);
    t1 = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    double read_ms = t1 - t0;

    // Validate
    int tot = out->elementSize();
    int nz = 0;
    float vmin = 1e9, vmax = -1e9;
    for (int i = 0; i < tot; i++) {
        if (fabsf(outData[i]) > 1e-6f) nz++;
        if (outData[i] < vmin) vmin = outData[i];
        if (outData[i] > vmax) vmax = outData[i];
    }

    printf("\n──────────────────────────────────────────\n");
    printf(" RESULTS: 4K Bayer → FHD RGGB via 6 opsets\n");
    printf("──────────────────────────────────────────\n");
    printf(" Pipeline: %.2f ms (%.0f FPS)\n", ms, 1000.0/ms);
    printf(" Readback: %.1f ms (%.1f MB/s)\n", read_ms, rb_bytes/1048576.0/read_ms*1000);
    printf(" Output:   %d/%d valid (%.1f%%) [%.4f, %.4f]\n", nz, tot, 100.0*nz/tot, vmin, vmax);

    // Sample center pixel (FHD coords)
    int cx = FHD_W/2, cy = FHD_H/2;
    printf(" Center:   (r=%d,c=%d) RGB = (%.4f, %.4f, %.4f)\n", cy, cx,
           outData[cy*FHD_W*3 + cx*3 + 0],
           outData[cy*FHD_W*3 + cx*3 + 1],
           outData[cy*FHD_W*3 + cx*3 + 2]);

    double total = ms + read_ms + (t1 - t0);
    printf("\n Total:    %.1f ms (upload+process+readback)\n", total);
    printf(" Budget:   %.1f ms (30ms target for %d×%d FHD)\n", 30.0, FHD_W, FHD_H);
    printf(" Status:   %s\n", ms < 30.0 ? "✅ UNDER TARGET" : "⚠️  OVER TARGET");

    // Memory
    int64_t input_mb = (int64_t)BAYER_W * BAYER_H * 4;
    int64_t rggb_mb  = (int64_t)FHD_W * FHD_H * 4 * 4;  // 4-channel float
    int64_t rgb_mb   = (int64_t)FHD_W * FHD_H * 3 * 4;  // 3-channel float
    printf("\n GPU Memory:\n");
    printf("  Input (INT32):  %.0f KB\n", input_mb/1024.0);
    printf("  RGGB (FLOAT):   %.0f KB\n", rggb_mb/1024.0);
    printf("  RGB×4 (FLOAT):  %.0f KB\n", rgb_mb*4/1024.0);
    printf("  Total:          ~%.1f MB\n", (input_mb + rggb_mb + rgb_mb*4)/1048576.0);

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
