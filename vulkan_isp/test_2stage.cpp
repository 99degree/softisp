#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <dlfcn.h>
#include <functional>
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

// struct for packing uniform data as raw bytes (to avoid type issues)


int main() {
    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";
    
    // Read SPIR-V
    auto spv1 = readFile((base + "shader1_unpack_blc.spv").c_str());
    auto spv2 = readFile((base + "shader2_demosaic_ccm.spv").c_str());
    printf("SPIR-V: stage1=%zu, stage2=%zu\n", spv1.size(), spv2.size());
    if (spv1.empty() || spv2.empty()) return 1;
    
    std::vector<int8_t> spv1_i8(spv1.size()), spv2_i8(spv2.size());
    memcpy(spv1_i8.data(), spv1.data(), spv1.size());
    memcpy(spv2_i8.data(), spv2.data(), spv2.size());
    
    // Build model: 3 tensors (input → stage1 → stage2)
    flatbuffers::FlatBufferBuilder fbb(1024 * 1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "2StageISP";
    net->tensorName.push_back("tensor_0");  // INT32 input [1,1,8,8]
    net->tensorName.push_back("tensor_1");  // FLOAT unpack output [1,4,4,4]
    net->tensorName.push_back("tensor_2");  // FLOAT demosaic output [1,3,8,8]
    
    // Stage 1: UnpackBLC
    {
        std::unique_ptr<MNN::OpT> op(new MNN::OpT);
        op->type = MNN::OpType_Extra;
        op->main.type = MNN::OpParameter_Extra;
        op->main.value = new MNN::ExtraT();
        auto extra = static_cast<MNN::ExtraT*>(op->main.value);
        extra->type = "UnpackBLC";
        op->inputIndexes.push_back(0);
        op->outputIndexes.push_back(1);
        op->name = "unpack";
        
        auto addAttr = [&](const char* key, std::function<void(MNN::AttributeT*)> setup) {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = key;
            setup(a.get());
            extra->attr.push_back(std::move(a));
        };
        
        addAttr("spirv", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT8;
            a->tensor->int8s = spv1_i8;
        });
        addAttr("output_shape", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {1, 4, 4, 4};
        });
        addAttr("global_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {4, 4, 1};
        });
        addAttr("group_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {1, 1, 1};
        });
        addAttr("input", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->list.reset(new MNN::ListValueT);
            a->list->i = {0, 1};  // [io_type=0, binding=1]
        });
        addAttr("input", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->list.reset(new MNN::ListValueT);
            a->list->i = {1, 2};  // [io_type=1, binding=2]
        });
        // const at binding 0, SSBO (buffer keyword)
        addAttr("const", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_FLOAT;
            a->tensor->float32s = {8.0f, 8.0f, 4.0f, 4.0f, 1023.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f,
                                    1.0f, 1.0f, 1.0f, 1.0f};
            a->b = false;  // SSBO
        });
        net->oplists.push_back(std::move(op));
    }
    
    // Stage 2: DemosaicCCM
    {
        std::unique_ptr<MNN::OpT> op(new MNN::OpT);
        op->type = MNN::OpType_Extra;
        op->main.type = MNN::OpParameter_Extra;
        op->main.value = new MNN::ExtraT();
        auto extra = static_cast<MNN::ExtraT*>(op->main.value);
        extra->type = "DemosaicCCM";
        op->inputIndexes.push_back(1);  // input = tensor_1
        op->outputIndexes.push_back(2); // output = tensor_2
        op->name = "demosaic";
        
        auto addAttr = [&](const char* key, std::function<void(MNN::AttributeT*)> setup) {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = key;
            setup(a.get());
            extra->attr.push_back(std::move(a));
        };
        
        addAttr("spirv", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT8;
            a->tensor->int8s = spv2_i8;
        });
        addAttr("output_shape", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {1, 3, 8, 8};
        });
        addAttr("global_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {4, 4, 1};
        });
        addAttr("group_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {1, 1, 1};
        });
        addAttr("input", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->list.reset(new MNN::ListValueT);
            a->list->i = {0, 1};  // input at binding 1
        });
        addAttr("input", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->list.reset(new MNN::ListValueT);
            a->list->i = {1, 2};  // output at binding 2
        });
        // const at binding 0, UBO (uniform keyword)
        addAttr("const", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_FLOAT;
            // float32s: input_w=4, input_h=4, output_w=8, output_h=8, sensor_max=1023,
            // ccm identity, pad
            a->tensor->float32s = {4.0f, 4.0f, 8.0f, 8.0f, 1023.0f,
                                    1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f, 0.0f,0.0f,1.0f,
                                    0.0f,0.0f,0.0f,0.0f};
            a->b = true;  // UBO
        });
        net->oplists.push_back(std::move(op));
    }
    
    // Pack
    auto netOffset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(netOffset);
    printf("Model: %zu bytes\n", fbb.GetSize());
    
    // Load Vulkan
    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();
    
    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    if (!interp) { fprintf(stderr, "FAIL Interpreter\n"); return 1; }
    
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;
    
    auto sess = interp->createSession(cfg);
    if (!sess) { fprintf(stderr, "FAIL Session\n"); return 1; }
    
    // Set input
    auto input = interp->getSessionInput(sess, "tensor_0");
    std::vector<int> inDims = {1, 1, 8, 8};
    interp->resizeTensor(input, inDims);
    interp->resizeSession(sess);
    printf("Input: elementSize=%d\n", input->elementSize());
    
    // Fill with Bayer pattern
    int32_t* inData = new int32_t[64];
    for (int y = 0; y < 8; y++)
        for (int x = 0; x < 8; x++) {
            if (y%2==0 && x%2==0) inData[y*8+x] = 100;
            else if (y%2==0 && x%2==1) inData[y*8+x] = 200;
            else if (y%2==1 && x%2==0) inData[y*8+x] = 300;
            else inData[y*8+x] = 400;
        }
    auto hostIn = MNN::Tensor::create(inDims, input->getType(), inData, MNN::Tensor::CAFFE);
    printf("CopyFromHost: %d\n", input->copyFromHostTensor(hostIn));
    
    // Run
    interp->runSession(sess);
    
    // Read outputs
    auto checkOutput = [&](int idx, const char* name) {
        char tname[32]; snprintf(tname, sizeof(tname), "tensor_%d", idx);
        auto out = interp->getSessionOutput(sess, tname);
        if (!out) { printf("  %s: NOT FOUND\n", name); return; }
        
        printf("  %s dims:", name);
        for (int j = 0; j < out->buffer().dimensions; j++) printf(" %d", out->length(j));
        printf(" elSize=%d\n", out->elementSize());
        
        if (out->elementSize() == 0) {
            printf("    Zero elements\n");
            return;
        }
        
        float* buf = new float[out->elementSize()];
        auto shape = out->shape();
        auto hostOut = MNN::Tensor::create(shape, out->getType(), buf, MNN::Tensor::CAFFE);
        bool ok = out->copyToHostTensor(hostOut);
        
        int nz = 0;
        for (int i = 0; i < std::min(out->elementSize(), 16); i++)
            if (buf[i] != 0.0f) nz++;
        printf("    Non-zero (first 16): %d/16\n", nz);
        printf("    First 4: %.3f %.3f %.3f %.3f\n", buf[0], buf[1], buf[2], buf[3]);
        delete[] buf;
    };
    
    checkOutput(1, "Stage1 (unpack)");
    checkOutput(2, "Stage2 (demosaic)");
    
    interp->releaseSession(sess);
    delete interp;
    delete[] inData;
    return 0;
}
