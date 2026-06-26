// Debug: 4K→FHD, read back a strip around center
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
auto to_i8 = [](const std::vector<uint8_t>& r) {
    std::vector<int8_t> v(r.size());
    memcpy(v.data(), r.data(), r.size());
    return v;
};

int main() {
    const int BW = 3840, BH = 2160, FW = 1920, FH = 1080;
    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";

    auto spv_u = readFile((base + "shader1_unpack_blc.spv").c_str());
    auto spv_d = readFile((base + "isp_opsets/demosaic_noscale.spv").c_str());
    auto spv_s = readFile((base + "shader6_display_simple.spv").c_str());

    // Build: unpack(4K) -> demosaic_noscale(FHD) -> display(FHD)
    isp::IspPipelineBuilder pipe(FW, FH, 3);
    pipe.addStage(0, isp::UnpackBlc(BW, BH), to_i8(spv_u));
    pipe.addStage(1, isp::DemosaicNoscale(FW, FH), to_i8(spv_d));
    pipe.addStage(2, isp::Display(FW, FH, 2.2f), to_i8(spv_s));

    size_t ms; const uint8_t* md = pipe.build(&ms);
    printf("Model: %zu bytes (3 opsets)\n", ms);

    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();

    auto interp = MNN::Interpreter::createFromBuffer(md, ms);
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN; cfg.numThread = 1;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;
    auto sess = interp->createSession(cfg);

    auto input = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(input, {1, 1, BH, BW});
    interp->resizeSession(sess);

    std::vector<int32_t> data(BW * BH);
    for (int y = 0; y < BH; y++)
        for (int x = 0; x < BW; x++)
            data[y*BW+x] = (y%2==0&&x%2==0) ? 100 :
                           ((y%2==0&&x%2==1) ? 200 :
                            ((y%2==1&&x%2==0) ? 300 : 400));
    auto hostIn = MNN::Tensor::create({1,1,BH,BW}, input->getType(), data.data(), MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);
    interp->runSession(sess);

    // Read back all intermediate tensors
    for (int ti = 1; ti <= 3; ti++) {
        char tname[16]; snprintf(tname, 16, "tensor_%d", ti);
        auto tout = interp->getSessionOutput(sess, tname);
        if (!tout) { printf("tensor_%d NOT FOUND\n", ti); continue; }
        float* od = new float[tout->elementSize()];
        auto ho = MNN::Tensor::create(tout->shape(), tout->getType(), od, MNN::Tensor::CAFFE);
        tout->copyToHostTensor(ho);

        int tot = tout->elementSize();
        int nz = 0; float mn=1e9,mx=-1e9;
        for (int i = 0; i < tot; i++) { if (fabsf(od[i])>1e-6f) nz++; if (od[i]<mn) mn=od[i]; if (od[i]>mx) mx=od[i]; }

        int cx = FW/2, cy = FH/2;
        int sh = tout->shape().size() >= 2 ?
                 (tout->shape()[1] == 3 ? 3 : 4) : 3;

        printf("\ntensor_%d shape: [", ti);
        for (int d=0;d<tout->shape().size();d++)
            printf("%d%c", tout->shape()[d], d+1<tout->shape().size()?',':']');
        printf(" [%d/%d valid] [%.4f, %.4f]\n", nz, tot, mn, mx);

        if (ti == 1) {
            // RGGB output from unpack (4 channels)
            float ch0 = od[cy*FW*4 + cx*4 + 0];
            float ch1 = od[cy*FW*4 + cx*4 + 1];
            float ch2 = od[cy*FW*4 + cx*4 + 2];
            float ch3 = od[cy*FW*4 + cx*4 + 3];
            printf("  RGGB center(%d,%d): ch0=%f ch1=%f ch2=%f ch3=%f\n",
                   cx, cy, ch0, ch1, ch2, ch3);
        } else {
            int stride = FW * sh;
            printf("  RGB center(%d,%d): (%f,%f,%f)\n", cx, cy,
                   od[cy*stride + cx*sh + 0], od[cy*stride + cx*sh + 1],
                   od[cy*stride + cx*sh + 2]);
            // 3x3 strip
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int pos = (cy+dy)*stride + (cx+dx)*sh;
                    printf("    (%d,%d): (%.4f,%.4f,%.4f)\n",
                           cx+dx, cy+dy, od[pos], od[pos+1], od[pos+2]);
                }
            }
        }
        delete[] od;
    }
    delete bc; delete interp;
    return 0;
}
