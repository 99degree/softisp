// test_6stage_full.cpp - Complete 6-stage ISP pipeline test
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

void checkStage(int idx, const char* name, MNN::Interpreter* interp, MNN::Session* sess) {
    char tname[32];
    snprintf(tname, 32, "tensor_%d", idx);
    auto out = interp->getSessionOutput(sess, tname);
    if (!out) { printf("Stage%d (%s): NOT FOUND\n", idx, name); return; }
    float* data = new float[out->elementSize()]();
    auto host = MNN::Tensor::create(out->shape(), out->getType(), data, MNN::Tensor::CAFFE);
    out->copyToHostTensor(host);
    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++)
        if (fabsf(data[i]) > 1e-6f) nz++;
    printf("Stage%d (%s): %d/%d non-zero", idx, name, nz, (int)out->elementSize());
    if (nz > 0) {
        printf(" | first 4: %.3f %.3f %.3f %.3f", data[0], data[1], data[2], data[3]);
    }
    printf("\n");
    delete[] data;
}

int main() {
    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";

    // Stage configs: {spv_file, type, output_shape, uniform_data, use_ssbo}
    struct StageConfig {
        std::string spv;
        const char* type;
        std::vector<int> shape;
        std::vector<float> uniforms;
        bool ssbo;
    };

    StageConfig stages[] = {
        {base + "shader1_unpack_blc.spv", "unpack_blc", {1, 4, 4, 4},
         {8.0f, 8.0f, 4.0f, 4.0f, 1023.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f}, false},
        {base + "shader2_demosaic_ccm.spv", "demosaic_ccm", {1, 3, 8, 8},
         {4.0f, 4.0f, 8.0f, 8.0f, 1023.0f, 1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f, 0.0f,0.0f,1.0f, 0.0f,0.0f,0.0f,0.0f}, false},
        {base + "shader3_fcs.spv", "fcs", {1, 3, 8, 8},
         {8.0f, 8.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, false},
        {base + "shader4_ee.spv", "ee", {1, 3, 8, 8},
         {8.0f, 8.0f, 0.5f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f}, false},
        {base + "shader5_ldci.spv", "ldci", {1, 3, 8, 8},
         {8.0f, 8.0f, 0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, false},
        {base + "shader6_display_simple.spv", "display", {1, 3, 8, 8},
         {8.0f, 8.0f, 0.0f, 1.0f, 1.0f, 2.2f, 0.0f, 0.0f}, false},
    };

    std::vector<std::vector<int8_t>> spv_data(6);
    for (int i = 0; i < 6; i++) {
        auto raw = readFile(stages[i].spv.c_str());
        printf("Stage%d SPIR-V: %zu bytes\n", i+1, raw.size());
        spv_data[i].resize(raw.size());
        memcpy(spv_data[i].data(), raw.data(), raw.size());
    }

    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "Full6StageISP";
    net->tensorName = {"tensor_0","tensor_1","tensor_2","tensor_3","tensor_4","tensor_5","tensor_6"};

    for (int si = 0; si < 6; si++) {
        auto op = std::unique_ptr<MNN::OpT>(new MNN::OpT);
        op->type = MNN::OpType_Extra;
        op->main.type = MNN::OpParameter_Extra;
        op->main.value = new MNN::ExtraT();
        auto* extra = static_cast<MNN::ExtraT*>(op->main.value);
        extra->type = stages[si].type;
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
            a->tensor->int8s = spv_data[si];
        });

        addAttr("output_shape", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = stages[si].shape;
        });

        addAttr("global_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            if (si == 0) a->tensor->int32s = {4, 4, 1};
            else a->tensor->int32s = {8, 8, 1};
        });

        addAttr("group_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {1, 1, 1};
        });

        addAttr("optimized_dispatch", [&](MNN::AttributeT* a) {
            a->b = true;
        });

        // Input binding
        addAttr("input", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->list.reset(new MNN::ListValueT);
            a->list->i = {0, 1};
        });
        // Output binding
        addAttr("input", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->list.reset(new MNN::ListValueT);
            a->list->i = {1, 2};
        });

        // Const buffer
        addAttr("const", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_FLOAT;
            a->tensor->float32s = stages[si].uniforms;
            a->b = stages[si].ssbo ? false : true;
        });

        net->oplists.push_back(std::move(op));
    }

    auto offset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(offset);
    printf("Model: %zu bytes\n", fbb.GetSize());

    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();

    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    if (!interp) { fprintf(stderr, "FAIL Interpreter\n"); return 1; }

    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    cfg.numThread = 1;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;

    auto startT = std::chrono::high_resolution_clock::now();
    auto sess = interp->createSession(cfg);
    auto endT = std::chrono::high_resolution_clock::now();
    printf("Session create: %.2f ms\n",
           std::chrono::duration<double, std::milli>(endT - startT).count());
    if (!sess) { fprintf(stderr, "Session failed\n"); return 1; }

    auto input = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(input, {1, 1, 8, 8});
    interp->resizeSession(sess);

    // Fill with Bayer pattern
    int32_t inData[64];
    for (int y = 0; y < 8; y++)
        for (int x = 0; x < 8; x++) {
            if (y%2==0 && x%2==0) inData[y*8+x] = 100;
            else if (y%2==0 && x%2==1) inData[y*8+x] = 200;
            else if (y%2==1 && x%2==0) inData[y*8+x] = 300;
            else inData[y*8+x] = 400;
        }

    auto hostIn = MNN::Tensor::create({1, 1, 8, 8}, input->getType(), inData, MNN::Tensor::CAFFE);
    printf("CopyFromHost: %d\n", input->copyFromHostTensor(hostIn));

    startT = std::chrono::high_resolution_clock::now();
    interp->runSession(sess);
    endT = std::chrono::high_resolution_clock::now();
    printf("Pipeline run: %.2f ms\n",
           std::chrono::duration<double, std::milli>(endT - startT).count());

    for (int i = 1; i <= 6; i++) checkStage(i, stages[i-1].type, interp, sess);

    delete bc;
    delete interp;
    return 0;
}
