#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
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

int main() {
    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto spv = readFile((base + "shader1_unpack_blc.spv").c_str());
    printf("SPIR-V: %zu bytes\n", spv.size());
    if (spv.empty()) return 1;
    std::vector<int8_t> spv_i8(spv.size());
    memcpy(spv_i8.data(), spv.data(), spv.size());
    
    flatbuffers::FlatBufferBuilder fbb(65536);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "Stage1Diag";
    net->tensorName.push_back("tensor_0");
    net->tensorName.push_back("tensor_1");
    
    std::unique_ptr<MNN::OpT> op(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = "Stage1Diag";
    op->inputIndexes.push_back(0);
    op->outputIndexes.push_back(1);
    op->name = "stage1";
    
    // Add attributes
    auto addAttr = [&](const char* key, std::function<void(MNN::AttributeT*)> setup) {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = key;
        setup(a.get());
        extra->attr.push_back(std::move(a));
    };
    
    addAttr("spirv", [&](MNN::AttributeT* a) {
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        a->tensor->int8s = spv_i8;
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
        a->list->i = {0, 1};
    });
    addAttr("input", [&](MNN::AttributeT* a) {
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {1, 2};
    });
    // const with float32s
    addAttr("const", [&](MNN::AttributeT* a) {
        a->i = 0;
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_FLOAT;
        a->tensor->float32s = {8.0f, 8.0f, 4.0f, 4.0f, 1023.0f, 0,0,0,0, 1,1,1,1};
        a->b = false;  // SSBO
    });
    
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
    auto hostIn = MNN::Tensor::create({1,1,8,8}, input->getType(), inData, MNN::Tensor::CAFFE);
    printf("CopyFromHost: %d\n", input->copyFromHostTensor(hostIn));
    
    interp->runSession(sess);
    
    auto out = interp->getSessionOutput(sess, "tensor_1");
    if (!out) { printf("Output NOT FOUND\n"); return 1; }
    printf("Output: dims=%d", out->buffer().dimensions);
    for (int j = 0; j < out->buffer().dimensions; j++) printf(" %d", out->length(j));
    printf(" elSize=%d\n", out->elementSize());
    
    float* buf = new float[out->elementSize()];
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), buf, MNN::Tensor::CAFFE);
    bool ok = out->copyToHostTensor(hostOut);
    printf("Copy: %d\n", ok);
    
    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++) if (buf[i] != 0.0f) nz++;
    printf("Non-zero: %d/%d\n", nz, out->elementSize());
    printf("R[0..3]: %.3f %.3f %.3f %.3f\n", buf[0], buf[1], buf[2], buf[3]);
    
    delete[] buf;
    interp->releaseSession(sess);
    delete interp;
    return 0;
}
