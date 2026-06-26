// test_demosaic_float.cpp - Test demosaic_ccm with float32 RGGB input
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
    auto spv = readFile("/data/data/com.termux/files/home/softisp/vulkan_isp/shader2_demosaic_ccm.spv");
    printf("SPIR-V: %zu bytes\n", spv.size());
    std::vector<int8_t> spv_i8(spv.size());
    memcpy(spv_i8.data(), spv.data(), spv.size());

    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "TestDemosaicFloat";
    net->tensorName = {"tensor_0", "tensor_1"};

    auto op = std::unique_ptr<MNN::OpT>(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto* extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = "demosaic_ccm";
    op->inputIndexes = {0};
    op->outputIndexes = {1};

    auto a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "spirv";
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_INT8;
    a->tensor->int8s = spv_i8;
    extra->attr.push_back(std::move(a));

    // output_shape
    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "output_shape";
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_INT32;
    a->tensor->int32s = {1, 3, 8, 8};
    extra->attr.push_back(std::move(a));

    // global_size
    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "global_size";
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_INT32;
    a->tensor->int32s = {4, 4, 1};
    extra->attr.push_back(std::move(a));

    // group_size
    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "group_size";
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_INT32;
    a->tensor->int32s = {1, 1, 1};
    extra->attr.push_back(std::move(a));

    // optimized_dispatch
    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "optimized_dispatch";
    a->b = true;
    extra->attr.push_back(std::move(a));

    // input binding: tensor 0 -> binding 1
    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "input";
    a->i = 0;
    a->list.reset(new MNN::ListValueT);
    a->list->i = {0, 1};
    extra->attr.push_back(std::move(a));

    // output binding: result -> binding 2
    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "input";
    a->i = 0;
    a->list.reset(new MNN::ListValueT);
    a->list->i = {1, 2};
    extra->attr.push_back(std::move(a));

    // const uniform: binding 0, UBO
    a = std::unique_ptr<MNN::AttributeT>(new MNN::AttributeT);
    a->key = "const";
    a->i = 0;
    a->tensor.reset(new MNN::BlobT);
    a->tensor->dataType = MNN::DataType_DT_FLOAT;
    a->tensor->float32s = {
        4.0f, 4.0f, 8.0f, 8.0f, 1023.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    a->b = true; // UBO
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
    interp->resizeTensor(input, {1, 4, 4, 4});
    interp->resizeSession(sess);
    printf("Input dims: ");
    for (int i = 0; i < input->buffer().dimensions; i++) printf(" %d", input->length(i));
    printf("\n");
    printf("Input type check skipped\n");

    // Fill with synthetic float RGGB pattern
    float inData[64];
    for (int ch = 0; ch < 4; ch++)
        for (int y = 0; y < 4; y++)
            for (int x = 0; x < 4; x++)
                inData[ch*16 + y*4 + x] = 100.0f + ch * 50.0f + x * 10.0f;

    auto hostIn = MNN::Tensor::create({1, 4, 4, 4}, input->getType(), inData, MNN::Tensor::CAFFE);
    printf("CopyFromHost: %d\n", input->copyFromHostTensor(hostIn));

    interp->runSession(sess);
    printf("Run complete\n");

    auto out = interp->getSessionOutput(sess, "tensor_1");
    if (!out) { printf("Output NOT FOUND\n"); return 1; }
    printf("Output dims: ");
    for (int i = 0; i < out->buffer().dimensions; i++) printf(" %d", out->length(i));
    printf("\n");

    auto outSize = out->elementSize();
    float* outData = new float[outSize]();
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    int nz = 0;
    for (int i = 0; i < outSize; i++)
        if (fabsf(outData[i]) > 1e-6f) nz++;
    printf("Non-zero: %d/%d\n", nz, (int)outSize);

    // Print first row of each channel
    for (int ch = 0; ch < 3; ch++) {
        printf("Channel %d first 8: ", ch);
        for (int x = 0; x < 8; x++) printf("%.1f ", outData[ch*64 + x]);
        printf("\n");
    }

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
