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

int main() {
    auto spv = readFile("/data/data/com.termux/files/home/softisp/vulkan_isp/shader1_unpack_blc.spv");
    printf("SPIR-V: %zu bytes\n", spv.size());
    if (spv.empty()) return 1;

    std::vector<int8_t> spv_i8(spv.size());
    memcpy(spv_i8.data(), spv.data(), spv.size());

    flatbuffers::FlatBufferBuilder fbb(1024 * 1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "TestUnpackBLCv2";
    net->tensorName.push_back("tensor_0");
    net->tensorName.push_back("tensor_1");

    std::unique_ptr<MNN::OpT> op(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = "TestUnpackBLCv2";
    op->inputIndexes.push_back(0);
    op->outputIndexes.push_back(1);
    op->name = "unpack_blc";

    // 1. spirv
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "spirv";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        a->tensor->int8s = spv_i8;
        extra->attr.push_back(std::move(a));
    }
    // 2. output_shape
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "output_shape";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 4, 4, 4};
        extra->attr.push_back(std::move(a));
    }
    // 3. global_size
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "global_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {4, 4, 1};
        extra->attr.push_back(std::move(a));
    }
    // 4. group_size (disable auto-tuning)
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "group_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
        extra->attr.push_back(std::move(a));
    }
    // 5. input binding: tensor 0 → binding 1
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "input";
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {0, 1};
        extra->attr.push_back(std::move(a));
    }
    // 6. output binding: result → binding 2
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "input";
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {1, 2};
        extra->attr.push_back(std::move(a));
    }
    // 7. const uniform at binding=0, as int8 raw bytes matching shader layout
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "const";
        a->i = 0;  // binding 0
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        // Pack 52 bytes: 5 uints + 8 floats
        struct UniformData {
            uint32_t input_width;
            uint32_t input_height;
            uint32_t output_width;
            uint32_t output_height;
            uint32_t sensor_max;
            float blc_r, blc_gr, blc_gb, blc_b;
            float wb_r, wb_gr, wb_gb, wb_b;
        } data = {
            8, 8, 4, 4, 1023,
            0.0f, 0.0f, 0.0f, 0.0f,
            1.0f, 1.0f, 1.0f, 1.0f
        };
        a->tensor->int8s.resize(sizeof(data));
        memcpy(a->tensor->int8s.data(), &data, sizeof(data));
        a->b = false;  // storage buffer (matches 'buffer' keyword)
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
    std::vector<int> inDims = {1, 1, 8, 8};
    interp->resizeTensor(input, inDims);
    interp->resizeSession(sess);
    printf("Input: elementSize=%d\n", input->elementSize());

    // Fill with Bayer pattern
    int32_t inData[64];
    for (int y = 0; y < 8; y++)
        for (int x = 0; x < 8; x++) {
            if (y%2==0 && x%2==0) inData[y*8+x] = 100;
            else if (y%2==0 && x%2==1) inData[y*8+x] = 200;
            else if (y%2==1 && x%2==0) inData[y*8+x] = 300;
            else inData[y*8+x] = 400;
        }

    auto hostIn = MNN::Tensor::create(inDims, input->getType(), inData, MNN::Tensor::CAFFE);
    printf("CopyFromHost: %d\n", input->copyFromHostTensor(hostIn));

    interp->runSession(sess);

    auto out = interp->getSessionOutput(sess, "tensor_1");
    float* outData = new float[64]();
    auto hostOut = MNN::Tensor::create({1, 4, 4, 4}, out->getType(), outData, MNN::Tensor::CAFFE);
    bool ok = out->copyToHostTensor(hostOut);
    printf("Copy: %d\n", ok);

    int nz = 0;
    for (int i = 0; i < 64; i++) if (outData[i] != 0.0f) nz++;
    printf("Non-zero: %d/64\n", nz);

    printf("R plane:\n");
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) printf("%8.2f ", outData[y*4+x]);
        printf("\n");
    }

    interp->releaseSession(sess);
    delete interp;
    delete[] outData;
    return 0;
}
