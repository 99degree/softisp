//
// test_unpack_blc.cpp - Test the unpack_blc shader in isolation
// Build with same flags as test_raw_model.cpp
//
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
    if (!f.good()) { fprintf(stderr, "Cannot read: %s\n", path); return {}; }
    size_t sz = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

int main(int argc, const char* argv[]) {
    auto spv = readFile("/data/data/com.termux/files/home/softisp/vulkan_isp/shader1_unpack_blc.spv");
    printf("SPIR-V: %zu bytes\n", spv.size());
    if (spv.empty()) return 1;

    // Convert SPIR-V bytes to int8_t for flatbuffers
    std::vector<int8_t> spv_i8(spv.size());
    memcpy(spv_i8.data(), spv.data(), spv.size());

    // Build model
    flatbuffers::FlatBufferBuilder fbb(1024 * 1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "TestUnpackBLC";

    net->tensorName.push_back("tensor_0");  // input: INT32 [1,1,8,8]
    net->tensorName.push_back("tensor_1");  // output: FLOAT [1,4,4,4]

    std::unique_ptr<MNN::OpT> op(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = "TestUnpackBLC";
    op->inputIndexes.push_back(0);
    op->outputIndexes.push_back(1);
    op->name = "unpack_blc";

    // spirv
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "spirv";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        a->tensor->int8s = spv_i8;
        extra->attr.push_back(std::move(a));
    }
    // output_shape: [1,4,4,4]
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "output_shape";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 4, 4, 4};
        extra->attr.push_back(std::move(a));
    }
    // global_size: [4, 4, 1]
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "global_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {4, 4, 1};
        extra->attr.push_back(std::move(a));
    }
    // group_size: [1,1,1] to bypass auto-tuning
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "group_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
        extra->attr.push_back(std::move(a));
    }
    // input binding: [io_type=0, binding=1]
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "input";
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {0, 1};
        extra->attr.push_back(std::move(a));
    }
    // output binding: [io_type=1, binding=2]
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "input";
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {1, 2};
        extra->attr.push_back(std::move(a));
    }
    // const uniform: binding=0, storage buffer
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "const";
        a->i = 0;  // binding 0 matches shader
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_FLOAT;
        a->tensor->float32s = {8, 8, 4, 4, 1023.0f, 0,0,0,0, 1,1,1,1};
        a->b = false;  // storage buffer
        extra->attr.push_back(std::move(a));
    }

    net->oplists.push_back(std::move(op));

    auto netOffset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(netOffset);
    printf("Model: %zu bytes\n", fbb.GetSize());

    // Load Vulkan backend
    void* vkLib = dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    if (!vkLib) { fprintf(stderr, "dlopen failed: %s\n", dlerror()); return 1; }
    typedef void (*RegFunc)();
    RegFunc reg = (RegFunc)dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) { reg(); }
    
    // Create interpreter
    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    if (!interp) { fprintf(stderr, "Failed Interpreter\n"); return 1; }

    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    cfg.numThread = 1;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;

    auto sess = interp->createSession(cfg);
    if (!sess) { fprintf(stderr, "Failed Session\n"); return 1; }

    // Set input
    auto input = interp->getSessionInput(sess, "tensor_0");
    std::vector<int> inDims = {1, 1, 8, 8};
    interp->resizeTensor(input, inDims);
    interp->resizeSession(sess);
    printf("Input: elementSize=%d\n", input->elementSize());

    // Fill input with Bayer pattern: R=100, Gr=200, Gb=300, B=400
    int32_t* inData = new int32_t[64];
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            if (y % 2 == 0 && x % 2 == 0) inData[y*8+x] = 100;    // R
            else if (y % 2 == 0 && x % 2 == 1) inData[y*8+x] = 200; // Gr  
            else if (y % 2 == 1 && x % 2 == 0) inData[y*8+x] = 300; // Gb
            else inData[y*8+x] = 400;  // B
        }
    }
    
    auto hostIn = MNN::Tensor::create(inDims, input->getType(), inData, MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);
    printf("Input copied\n");

    // Run
    int code = interp->runSession(sess);
    printf("Run: code=%d\n", code);

    // Read output
    auto out = interp->getSessionOutput(sess, "tensor_1");
    printf("Output: dims=[");
    for (int i = 0; i < out->buffer().dimensions; i++)
        printf("%s%d", i?",":"", out->length(i));
    printf("] host=%p\n", (void*)out->buffer().host);

    // Create host output
    float* outData = new float[64]();
    auto hostOut = MNN::Tensor::create({1, 4, 4, 4}, out->getType(), outData, MNN::Tensor::CAFFE);
    
    bool ok = out->copyToHostTensor(hostOut);
    printf("Copy: %d\n", ok);

    // Count non-zero
    int nz = 0;
    for (int i = 0; i < 64; i++) if (outData[i] != 0.0f) nz++;
    printf("Non-zero: %d/64\n", nz);
    
    // Show R channel (first 16 floats = plane 0)
    printf("R plane:\n");
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++)
            printf("%8.2f ", outData[y*4+x]);
        printf("\n");
    }
    // Show Gr/Gb/B averages
    float avg_g = 0, avg_b = 0;
    for (int i = 16; i < 32; i++) avg_g += outData[i];
    for (int i = 32; i < 48; i++) avg_g += outData[i];
    for (int i = 48; i < 64; i++) avg_b += outData[i];
    avg_g /= 32; avg_b /= 16;
    printf("Avg Gr+Gb: %.2f  Avg B: %.2f\n", avg_g, avg_b);

    interp->releaseSession(sess);
    delete interp;
    delete[] inData;
    delete[] outData;
    return 0;
}
