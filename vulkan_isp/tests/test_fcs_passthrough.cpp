// test_fcs_passthrough.cpp - Passthrough test for FCS data path
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

// Embedded simple passthrough shader SPIR-V would be complex, so we write the .comp and compile
int main() {
    // Just compile and run the existing shader3_fcs.spv with corrected uniform data
    auto spv = readFile("/data/data/com.termux/files/home/softisp/vulkan_isp/shader3_fcs.spv");
    printf("SPIR-V: %zu bytes\n", spv.size());
    std::vector<int8_t> spv_i8(spv.size());
    memcpy(spv_i8.data(), spv.data(), spv.size());

    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "TestFCSPass";
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
    // Shader expects: width, height, gain, center_th, suppression, pad[3] = 8 floats
    a->tensor->float32s = {8.0f, 8.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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
    printf("Input: elementSize=%d\n", input->elementSize());

    // Fill with known pattern
    float inData[192];
    for (int i = 0; i < 192; i++) inData[i] = 100.0f + i * 0.1f;

    auto hostIn = MNN::Tensor::create({1, 3, 8, 8}, input->getType(), inData, MNN::Tensor::CAFFE);
    printf("CopyFromHost: %d\n", input->copyFromHostTensor(hostIn));

    interp->runSession(sess);
    printf("Run complete\n");

    auto out = interp->getSessionOutput(sess, "tensor_1");
    if (!out) { printf("Output NOT FOUND\n"); return 1; }

    float* outData = new float[out->elementSize()]();
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++)
        if (fabsf(outData[i] - inData[i]) > 1e-4f) { nz++; break; }
    if (nz == 0) {
        printf("SUCCESS: Output matches input (passthrough)\n");
    } else {
        printf("FAIL: Output differs from input\n");
        int nz2 = 0;
        for (int i = 0; i < out->elementSize(); i++)
            if (fabsf(outData[i]) > 1e-6f) nz2++;
        printf("Non-zero in output: %d/%d\n", nz2, (int)out->elementSize());
        for (int ch = 0; ch < 3; ch++) {
            printf("Ch%d first 8 out: ", ch);
            for (int x = 0; x < 8; x++) printf("%.1f ", outData[ch*64 + x]);
            printf(" | in: ");
            for (int x = 0; x < 8; x++) printf("%.1f ", inData[ch*64 + x]);
            printf("\n");
        }
    }

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
