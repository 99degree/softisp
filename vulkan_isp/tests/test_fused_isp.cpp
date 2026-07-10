// // test_fused_isp.cpp - Pipeline version aligned with @../mesa_test/* shader set
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <chrono>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/MNNForwardType.h>
#include "schema/current/MNN_generated.h"

static std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.good()) { fprintf(stderr, "Cannot read: %s\n", path); return {}; }
    size_t sz = f.tellg(); f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

void checkOutput(int idx, MNN::Interpreter* interp, MNN::Session* sess, int expectedDims) {
    const char* tname = idx == 0 ? "tensor_0" : (idx == 1 ? "tensor_1" : "tensor_2");
    auto out = interp->getSessionOutput(sess, tname);
    if (!out) {
        printf("stage%d: NOT FOUND\n", idx);
        return;
    }
    int dims = out->buffer().dimensions;
    printf("stage%d: ", idx);
    for (int i = 0; i < dims; i++) printf(" %d", out->length(i));
    printf("\n");
    auto count = out->elementSize();
    float* data = new float[count];
    auto host = MNN::Tensor::create(out->shape(), out->getType(), data, MNN::Tensor::CAFFE);
    out->copyToHostTensor(host);
    int nz = 0;
    for (int i = 0; i < count && i < 64; i++) if (data[i] > 1e-6f) nz++;
    printf("  non-zero(first%zu)=%d\n", std::min<size_t>(count, 64), nz);
    delete[] data;
}

int main() {
    std::string base = "/data/data/com.termux/files/home/mesa_test/";

    std::vector<uint8_t> raw;
    std::vector<int8_t> spv[3];

    raw = readFile((base + "cs_blc_wb.spv").c_str());
    std::copy(raw.begin(), raw.end(), std::back_inserter(spv[0]));
    raw = readFile((base + "cs_bayer_to_rgb_f32.spv").c_str());
    std::copy(raw.begin(), raw.end(), std::back_inserter(spv[1]));
    raw = readFile((base + "cs_tone.spv").c_str());
    std::copy(raw.begin(), raw.end(), std::back_inserter(spv[2]));

    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "MesaAlignedISP";
    net->tensorName = {"tensor_0","tensor_1","tensor_2"};

    for (int si = 0; si < 3; si++) {
        auto op = std::unique_ptr<MNN::OpT>(new MNN::OpT);
        op->type = MNN::OpType_Extra;
        op->main.type = MNN::OpParameter_Extra;
        op->main.value = new MNN::ExtraT();
        auto* extra = static_cast<MNN::ExtraT*>(op->main.value);
        extra->type = (si == 0 ? "blc_wb" : (si == 1 ? "bayer_to_rgb" : "tone"));
        op->inputIndexes.push_back(si == 0 ? 0 : si);
        op->outputIndexes.push_back(si + 1);

        auto a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
        a->key = "spirv";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        a->tensor->int8s = spv[si];
        extra->attr.push_back(std::move(a));

        a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
        a->key = "output_shape";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        if (si == 0) a->tensor->int32s = {1, 4, 4, 4};
        else if (si == 1) a->tensor->int32s = {1, 3, 4, 4};
        else a->tensor->int32s = {1, 3, 4, 4};
        extra->attr.push_back(std::move(a));

        a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
        a->key = "global_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {4, 4, 1};
        extra->attr.push_back(std::move(a));

        a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
        a->key = "group_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
        extra->attr.push_back(std::move(a));

        a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
        a->key = "optimized_dispatch";
        a->b = true;
        extra->attr.push_back(std::move(a));

        std::vector<int> binding = (si == 0) ? std::vector<int>{1,2} : std::vector<int>{1,2};
        for (int bi = 0; bi < 2; bi++) {
            auto bn = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
            bn->key = "input";
            bn->i = 0;
            bn->list.reset(new MNN::ListValueT);
            bn->list->i.push_back(bi);
            bn->list->i.push_back(binding[bi]);
            extra->attr.push_back(std::move(bn));
        }

        auto bn = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
        bn->key = "const";
        bn->i = 0;
        bn->tensor.reset(new MNN::BlobT);
        bn->tensor->dataType = MNN::DataType_DT_FLOAT;
        if (si == 0) bn->tensor->float32s = {8.0f, 8.0f, 4.0f, 4.0f, 1023.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        else if (si == 1) bn->tensor->float32s = {4.0f, 4.0f, 4.0f, 4.0f, 1023.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f};
        else bn->tensor->float32s = {0.0f, 0.0f, 1.0f};
        bn->b = (si == 0 ? false : true);
        extra->attr.push_back(std::move(bn));

        net->oplists.push_back(std::move(op));
    }

    auto offset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(offset);

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
    printf("session creation: %f ms\n", std::chrono::duration<double, std::milli>(endT - startT).count());
    if (!sess) { fprintf(stderr, "Session failed\n"); return 1; }

    auto in = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(in, {1, 4, 4, 4});
    interp->resizeSession(sess);

    float inData[64];
    for (int i = 0; i < 64; i++) inData[i] = 50.0f + i * 5.0f;
    auto hostIn = MNN::Tensor::create({1, 4, 4, 4}, in->getType(), inData, MNN::Tensor::CAFFE);
    if (!hostIn) {
        fprintf(stderr, "create input tensor failed\n");
        return 1;
    }
    printf("copyFromHost: %d\n", in->copyFromHostTensor(hostIn));

    startT = std::chrono::high_resolution_clock::now();
    interp->runSession(sess);
    endT = std::chrono::high_resolution_clock::now();
    printf("run: %f ms\n", std::chrono::duration<double, std::milli>(endT - startT).count());

    for (int idx = 0; idx <= 2; idx++) checkOutput(idx, interp, sess, 4);

    delete bc;
    delete interp;
    return 0;
}
