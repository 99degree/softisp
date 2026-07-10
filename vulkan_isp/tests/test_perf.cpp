// test_perf.cpp - Performance test at 480x360
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <chrono>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/MNNForwardType.h>
#include "schema/current/MNN_generated.h"

static std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.good()) return {};
    size_t sz = f.tellg(); f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

int main(int argc, char** argv) {
    int W = argc > 1 ? atoi(argv[1]) : 480;
    int H = argc > 2 ? atoi(argv[2]) : 360;
    printf("Testing %dx%d...\n", W, H);

    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto spv1 = readFile((base + "shader1_unpack_blc.spv").c_str());
    auto spv2 = readFile((base + "shader2_demosaic_ccm.spv").c_str());
    auto spv3 = readFile((base + "shader3_fcs.spv").c_str());
    auto spv4 = readFile((base + "shader4_ee.spv").c_str());
    auto spv5 = readFile((base + "shader5_ldci.spv").c_str());
    auto spv6 = readFile((base + "shader6_display_simple.spv").c_str());
    std::vector<int8_t> spv1_i8(spv1.size()), spv2_i8(spv2.size()), spv3_i8(spv3.size()), spv4_i8(spv4.size()), spv5_i8(spv5.size()), spv6_i8(spv6.size());
    memcpy(spv1_i8.data(), spv1.data(), spv1.size());
    memcpy(spv2_i8.data(), spv2.data(), spv2.size());
    memcpy(spv3_i8.data(), spv3.data(), spv3.size());
    memcpy(spv4_i8.data(), spv4.data(), spv4.size());
    memcpy(spv5_i8.data(), spv5.data(), spv5.size());
    memcpy(spv6_i8.data(), spv6.data(), spv6.size());

    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "PerfISP";
    char tname[7][32];
    for (int i = 0; i < 7; i++) snprintf(tname[i], 32, "tensor_%d", i);
    net->tensorName = {tname[0],tname[1],tname[2],tname[3],tname[4],tname[5],tname[6]};

    auto addStage = [&](int si, const std::vector<int8_t>& spv, const char* type,
                        const std::vector<int>& shape, const std::vector<float>& uniforms) {
        auto op = std::unique_ptr<MNN::OpT>(new MNN::OpT);
        op->type = MNN::OpType_Extra;
        op->main.type = MNN::OpParameter_Extra;
        op->main.value = new MNN::ExtraT();
        auto* extra = static_cast<MNN::ExtraT*>(op->main.value);
        extra->type = type;
        op->inputIndexes.push_back(si == 0 ? 0 : si);
        op->outputIndexes.push_back(si + 1);

        auto addAttr = [&](const char* key, std::function<void(MNN::AttributeT*)> setup) {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = key;
            setup(a.get());
            extra->attr.push_back(std::move(a));
        };

        addAttr("spirv", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT8;
            a->tensor->int8s = spv;
        });
        addAttr("output_shape", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = shape;
        });
        addAttr("global_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            if (si == 0) a->tensor->int32s = {W/2, H/2, 1};
            else a->tensor->int32s = {W, H, 1};
        });
        addAttr("group_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {1, 1, 1};
        });
        addAttr("optimized_dispatch", [&](MNN::AttributeT* a) { a->b = true; });
        addAttr("input", [&](MNN::AttributeT* a) {
            a->i = 0; a->list.reset(new MNN::ListValueT); a->list->i = {0, 1};
        });
        addAttr("input", [&](MNN::AttributeT* a) {
            a->i = 0; a->list.reset(new MNN::ListValueT); a->list->i = {1, 2};
        });
        addAttr("const", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_FLOAT;
            a->tensor->float32s = uniforms;
            a->b = false;
        });
        net->oplists.push_back(std::move(op));
    };

    // Stage 0: unpack_blc (input WxH -> RGGB W/2xH/2)
    addStage(0, spv1_i8, "unpack_blc", {1, 4, H/2, W/2},
             {float(W), float(H), float(W/2), float(H/2), 1023.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f});
    // Stage 1: demosaic_ccm
    addStage(1, spv2_i8, "demosaic_ccm", {1, 3, H, W},
             {float(W/2), float(H/2), float(W), float(H), 1023.0f,
              1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f, 0.0f,0.0f,1.0f, 0.0f,0.0f,0.0f,0.0f});
    // Stage 2: fcs
    addStage(2, spv3_i8, "fcs", {1, 3, H, W},
             {float(W), float(H), 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    // Stage 3: ee
    addStage(3, spv4_i8, "ee", {1, 3, H, W},
             {float(W), float(H), 0.5f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f});
    // Stage 4: ldci
    addStage(4, spv5_i8, "ldci", {1, 3, H, W},
             {float(W), float(H), 0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    // Stage 5: display
    addStage(5, spv6_i8, "display", {1, 3, H, W},
             {float(W), float(H), 0.0f, 1.0f, 1.0f, 2.2f, 0.0f, 0.0f});

    auto offset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(offset);

    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();

    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    cfg.numThread = 1;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;
    auto sess = interp->createSession(cfg);

    auto input = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(input, {1, 1, H, W});
    interp->resizeSession(sess);

    // Fill with synthetic Bayer
    std::vector<int32_t> inData(W * H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int v = 100 + (x + y) % 200;
            inData[y*W + x] = v;
        }

    auto hostIn = MNN::Tensor::create({1, 1, H, W}, input->getType(), inData.data(), MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);

    // Warmup
    interp->runSession(sess);
    
    // Benchmark
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10; i++) interp->runSession(sess);
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count() / 10.0;

    // Verify output
    auto out = interp->getSessionOutput(sess, "tensor_6");
    float* outData = new float[out->elementSize()]();
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++) if (fabsf(outData[i]) > 1e-6f) nz++;
    printf("Time: %.2f ms/frame | Output: %d/%d non-zero | FPS: %.1f\n", 
           ms, nz, (int)out->elementSize(), 1000.0/ms);

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
