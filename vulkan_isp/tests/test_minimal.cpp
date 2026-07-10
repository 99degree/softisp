#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <iostream>
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
    auto spv = readFile("test_minimal.spv");
    printf("SPIR-V: %zu bytes\n", spv.size());
    if (spv.empty()) return 1;
    std::vector<int8_t> spv_i8(spv.size());
    memcpy(spv_i8.data(), spv.data(), spv.size());
    
    flatbuffers::FlatBufferBuilder fbb(65536);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "Minimal";
    net->tensorName.push_back("tensor_0");
    net->tensorName.push_back("tensor_1");
    
    std::unique_ptr<MNN::OpT> op(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = "Minimal";
    op->inputIndexes.push_back(0);
    op->outputIndexes.push_back(1);
    op->name = "minimal";
    
    // spirv
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "spirv";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        a->tensor->int8s = spv_i8;
        extra->attr.push_back(std::move(a));
    }
    // output_shape
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "output_shape";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 4, 1, 1};
        extra->attr.push_back(std::move(a));
    }
    // global_size
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "global_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
        extra->attr.push_back(std::move(a));
    }
    // group_size
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "group_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
        extra->attr.push_back(std::move(a));
    }
    // input binding
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "input";
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {0, 1};
        extra->attr.push_back(std::move(a));
    }
    // output binding
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "input";
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {1, 2};
        extra->attr.push_back(std::move(a));
    }
    
    net->oplists.push_back(std::move(op));
    
    auto netOffset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(netOffset);
    printf("Model: %zu bytes\n", fbb.GetSize());
    
    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();
    
    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;
    auto sess = interp->createSession(cfg);
    
    auto input = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(input, {1,1,1,1});
    interp->resizeSession(sess);
    
    int32_t val = 42;
    auto hostIn = MNN::Tensor::create({1,1,1,1}, input->getType(), &val, MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);
    
    interp->runSession(sess);
    
    auto out = interp->getSessionOutput(sess, "tensor_1");
    float outData[4] = {0};
    auto hostOut = MNN::Tensor::create({1,4,1,1}, out->getType(), outData, MNN::Tensor::CAFFE);
    bool ok = out->copyToHostTensor(hostOut);
    printf("Copy: %d\n", ok);
    printf("Output: [%f, %f, %f, %f]\n", outData[0], outData[1], outData[2], outData[3]);
    
    interp->releaseSession(sess);
    delete interp;
    return 0;
}
