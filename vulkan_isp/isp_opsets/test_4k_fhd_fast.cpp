// test_4k_fhd_fast.cpp — 4K→FHD with simpler FCS+Display (no EE/LDCI)
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
auto to_i8 = [](const std::vector<uint8_t>& r) {
    std::vector<int8_t> v(r.size()); memcpy(v.data(), r.data(), r.size()); return v;
};

int main() {
    const int BW = 3840, BH = 2160;
    const int FW = 1920, FH = 1080;
    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";
    
    // Load SPIR-V
    auto spv_u = readFile((base + "shader1_unpack_blc.spv").c_str());
    auto spv_d = readFile((base + "isp_opsets/demosaic_noscale.spv").c_str());
    auto spv_f = readFile((base + "shader3_fcs.spv").c_str());
    auto spv_s = readFile((base + "shader6_display_simple.spv").c_str());

    // 4-stage pipeline: unpack → demosaic_noscale → fcs → display
    isp::IspPipelineBuilder pipe;
    pipe.addStage( isp::UnpackBlc(BW, BH), to_i8(spv_u));
    pipe.addStage( isp::DemosaicNoscale(FW, FH), to_i8(spv_d));
    pipe.addStage( isp::Fcs(FW, FH, 1.0f), to_i8(spv_f));
    pipe.addStage( isp::Display(FW, FH, 2.2f), to_i8(spv_s));

    size_t ms;
    const uint8_t* md = pipe.build(&ms);
    printf("Model: %zu bytes (4 opsets)\n", ms);

    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();

    auto interp = MNN::Interpreter::createFromBuffer(md, ms);
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN; cfg.numThread = 1;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High; cfg.backendConfig = bc;
    auto sess = interp->createSession(cfg);

    auto input = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(input, {1, 1, BH, BW});
    interp->resizeSession(sess);

    std::vector<int32_t> data(BW * BH);
    for (int y = 0; y < BH; y++)
        for (int x = 0; x < BW; x++)
            data[y*BW+x] = (y%2==0 && x%2==0) ? 100 : ((y%2==0 && x%2==1) ? 200 : ((y%2==1 && x%2==0) ? 300 : 400));
    auto hostIn = MNN::Tensor::create({1,1,BH,BW}, input->getType(), data.data(), MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);

    interp->runSession(sess);
    auto rs = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10; i++) interp->runSession(sess);
    auto re = std::chrono::high_resolution_clock::now();
    double ms_f = std::chrono::duration<double,std::milli>(re-rs).count() / 10.0;

    auto out = interp->getSessionOutput(sess, "tensor_4");
    float* od = new float[out->elementSize()];
    auto ho = MNN::Tensor::create(out->shape(), out->getType(), od, MNN::Tensor::CAFFE);
    out->copyToHostTensor(ho);

    int nz = 0, tot = out->elementSize();
    for (int i = 0; i < tot; i++) if (fabsf(od[i]) > 1e-6f) nz++;
    
    printf("\n─── 4K→FHD (4 opsets: unpack+demo+fcs+display) ───\n");
    printf("Pipeline: %.2f ms (%.0f FPS)\n", ms_f, 1000.0/ms_f);
    printf("Valid:    %d/%d (%.1f%%)\n", nz, tot, 100.0*nz/tot);

    int cx = FW/2, cy = FH/2;
    printf("Center:   RGB = (%.4f, %.4f, %.4f)\n",
           od[cy*FW*3+cx*3+0], od[cy*FW*3+cx*3+1], od[cy*FW*3+cx*3+2]);
    printf("Budget:   30ms — %s\n", ms_f < 30.0 ? "✅" : "⚠️");
    delete[] od; delete bc; delete interp;
    return 0;
}
