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

int main() {
    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto spv1 = readFile((base + "shader1_unpack_blc.spv").c_str());
    auto spv2 = readFile((base + "shader2_demosaic_ccm.spv").c_str());
    auto spv3 = readFile((base + "shader3_fcs.spv").c_str());
    auto spv4 = readFile((base + "shader4_ee.spv").c_str());
    std::vector<int8_t> spv1_i8(spv1.size()), spv2_i8(spv2.size()), spv3_i8(spv3.size()), spv4_i8(spv4.size());
    memcpy(spv1_i8.data(), spv1.data(), spv1.size());
    memcpy(spv2_i8.data(), spv2.data(), spv2.size());
    memcpy(spv3_i8.data(), spv3.data(), spv3.size());
    memcpy(spv4_i8.data(), spv4.data(), spv4.size());

    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "4StageISP";
    net->tensorName = {"tensor_0","tensor_1","tensor_2","tensor_3","tensor_4"};

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
            if (si == 0) a->tensor->int32s = {4, 4, 1};
            else a->tensor->int32s = {8, 8, 1};
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

    addStage(0, spv1_i8, "unpack_blc", {1, 4, 4, 4},
             {8.0f, 8.0f, 4.0f, 4.0f, 1023.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f});
    addStage(1, spv2_i8, "demosaic_ccm", {1, 3, 8, 8},
             {4.0f, 4.0f, 8.0f, 8.0f, 1023.0f, 1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f, 0.0f,0.0f,1.0f, 0.0f,0.0f,0.0f,0.0f});
    addStage(2, spv3_i8, "fcs", {1, 3, 8, 8},
             {8.0f, 8.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    addStage(3, spv4_i8, "ee", {1, 3, 8, 8},
             {8.0f, 8.0f, 0.5f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f});

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
    interp->resizeTensor(input, {1, 1, 8, 8});
    interp->resizeSession(sess);

    int32_t inData[64];
    for (int y = 0; y < 8; y++)
        for (int x = 0; x < 8; x++) {
            if (y%2==0 && x%2==0) inData[y*8+x] = 100;
            else if (y%2==0 && x%2==1) inData[y*8+x] = 200;
            else if (y%2==1 && x%2==0) inData[y*8+x] = 300;
            else inData[y*8+x] = 400;
        }

    auto hostIn = MNN::Tensor::create({1, 1, 8, 8}, input->getType(), inData, MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);

    interp->runSession(sess);

    auto out = interp->getSessionOutput(sess, "tensor_4");
    if (!out) { printf("Output NOT FOUND\n"); return 1; }
    float* outData = new float[out->elementSize()]();
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++)
        if (fabsf(outData[i]) > 1e-6f) nz++;
    printf("Non-zero: %d/%d\n", nz, (int)out->elementSize());
    if (nz > 0) {
        printf("First 16: ");
        for (int i = 0; i < 16; i++) printf("%.3f ", outData[i]);
        printf("\n");
    }

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
