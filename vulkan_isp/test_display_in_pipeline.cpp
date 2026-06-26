#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
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
    auto spv5 = readFile((base + "shader5_ldci.spv").c_str());
    auto spv6 = readFile((base + "shader6_display_simple.spv").c_str());
    std::vector<int8_t> spv5_i8(spv5.size()), spv6_i8(spv6.size());
    memcpy(spv5_i8.data(), spv5.data(), spv5.size());
    memcpy(spv6_i8.data(), spv6.data(), spv6.size());

    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "DisplayInPipeline";
    net->tensorName = {"tensor_0","tensor_1","tensor_2"};

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
            a->tensor->int32s = {8, 8, 1};
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

    // Stage 5: LDCI output (simulated input to display)
    addStage(0, spv5_i8, "ldci", {1, 3, 8, 8},
             {8.0f, 8.0f, 0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    // Stage 6: Display
    addStage(1, spv6_i8, "display", {1, 3, 8, 8},
             {8.0f, 8.0f, 0.0f, 1.0f, 1.0f, 2.2f, 0.0f, 0.0f});

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
    interp->resizeTensor(input, {1, 3, 8, 8});
    interp->resizeSession(sess);

    // Simulate LDCI output (all ~0.1 values)
    float inData[192];
    for (int i = 0; i < 192; i++) inData[i] = 0.1f;

    auto hostIn = MNN::Tensor::create({1, 3, 8, 8}, input->getType(), inData, MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);

    interp->runSession(sess);

    auto out = interp->getSessionOutput(sess, "tensor_2");
    if (!out) { printf("Output NOT FOUND\n"); return 1; }
    float* outData = new float[out->elementSize()]();
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++)
        if (fabsf(outData[i]) > 1e-6f) nz++;
    printf("Non-zero: %d/%d\n", nz, (int)out->elementSize());
    if (nz > 0) {
        printf("First 16 out: ");
        for (int i = 0; i < 16; i++) printf("%.3f ", outData[i]);
        printf("\nFirst 16 in:  ");
        for (int i = 0; i < 16; i++) printf("%.3f ", inData[i]);
        printf("\n");
    }

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
