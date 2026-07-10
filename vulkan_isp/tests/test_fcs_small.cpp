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
    auto spv = readFile("/data/data/com.termux/files/home/softisp/vulkan_isp/shader3_fcs.spv");
    printf("SPIR-V: %zu bytes\n", spv.size());
    std::vector<int8_t> spv_i8(spv.size());
    memcpy(spv_i8.data(), spv.data(), spv.size());

    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "TestFCSSmall";
    net->tensorName = {"tensor_0", "tensor_1"};

    auto op = std::unique_ptr<MNN::OpT>(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto* extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = "fcs";
    op->inputIndexes = {0};
    op->outputIndexes = {1};

    auto a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "spirv";
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_INT8;
    a->tensor->int8s = spv_i8;
    extra->attr.push_back(std::move(a));

    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "output_shape";
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_INT32;
    a->tensor->int32s = {1, 3, 8, 8};
    extra->attr.push_back(std::move(a));

    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "global_size";
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_INT32;
    a->tensor->int32s = {8, 8, 1};
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

    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "input";
    a->i = 0;
    a->list.reset(new MNN::ListValueT);
    a->list->i = {0, 1};
    extra->attr.push_back(std::move(a));

    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "input";
    a->i = 0;
    a->list.reset(new MNN::ListValueT);
    a->list->i = {1, 2};
    extra->attr.push_back(std::move(a));

    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "const";
    a->i = 0;
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_FLOAT;
    // width=8, height=8, gain=1.0, center_th=0.5, suppression=0.1 -> clamp [0.9, 1.1]
    a->tensor->float32s = {8.0f, 8.0f, 1.0f, 0.5f, 0.1f, 0.0f, 0.0f, 0.0f};
    a->b = false;
    extra->attr.push_back(std::move(a));

    net->oplists.push_back(std::move(op));

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

    auto sess = interp->createSession(cfg);
    if (!sess) { fprintf(stderr, "FAIL Session\n"); return 1; }

    auto input = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(input, {1, 3, 8, 8});
    interp->resizeSession(sess);

    // Fill with normalized [0,1] values
    float inData[192];
    for (int i = 0; i < 192; i++) inData[i] = i / 191.0f;

    auto hostIn = MNN::Tensor::create({1, 3, 8, 8}, input->getType(), inData, MNN::Tensor::CAFFE);
    printf("CopyFromHost: %d\n", input->copyFromHostTensor(hostIn));

    interp->runSession(sess);

    auto out = interp->getSessionOutput(sess, "tensor_1");
    float* outData = new float[out->elementSize()]();
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++)
        if (fabsf(outData[i]) > 1e-6f) nz++;
    printf("Non-zero: %d/%d\n", nz, (int)out->elementSize());

    printf("First 16 out: ");
    for (int i = 0; i < 16; i++) printf("%.3f ", outData[i]);
    printf("\nFirst 16 in:  ");
    for (int i = 0; i < 16; i++) printf("%.3f ", inData[i]);
    printf("\n");

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
