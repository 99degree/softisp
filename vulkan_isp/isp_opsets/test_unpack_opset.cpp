// test_unpack_opset.cpp — Standalone opset test: isp.unpack_blc
// Verifies the op in isolation without pipeline chaining.
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
#include "isp_opset.h"

static std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.good()) { fprintf(stderr, "Cannot read: %s\n", path); return {}; }
    size_t sz = f.tellg(); f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

int main() {
    const int W = 8, H = 8;

    // Load SPIR-V for this opset
    auto spv = readFile("/data/data/com.termux/files/home/softisp/vulkan_isp/shader1_unpack_blc.spv");
    printf("SPIR-V: %zu bytes\n", spv.size());
    std::vector<int8_t> spv_i8(spv.size());
    memcpy(spv_i8.data(), spv.data(), spv.size());

    // Build single-op model using opset API
    flatbuffers::FlatBufferBuilder fbb(1024*1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "UnpackBlcOpset";
    net->tensorName = {"tensor_0", "tensor_1"};

    isp::OpDesc desc = isp::UnpackBlc(W, H);
    auto op = std::unique_ptr<MNN::OpT>(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto* extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = desc.type;
    op->inputIndexes = {0};
    op->outputIndexes = {1};

    auto addA = [&](const char* key, std::function<void(MNN::AttributeT*)> fn) {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = key; fn(a.get());
        extra->attr.push_back(std::move(a));
    };

    addA("spirv", [&](MNN::AttributeT* a) {
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        a->tensor->int8s = spv_i8;
    });
    addA("output_shape", [&](MNN::AttributeT* a) {
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = desc.output_shape;
    });
    addA("global_size", [&](MNN::AttributeT* a) {
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = desc.global_size;
    });
    addA("group_size", [&](MNN::AttributeT* a) {
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
    });
    addA("optimized_dispatch", [&](MNN::AttributeT* a) { a->b = true; });
    addA("input", [&](MNN::AttributeT* a) {
        a->i = 0; a->list.reset(new MNN::ListValueT); a->list->i = {0, 1};
    });
    addA("input", [&](MNN::AttributeT* a) {
        a->i = 0; a->list.reset(new MNN::ListValueT); a->list->i = {1, 2};
    });
    addA("const", [&](MNN::AttributeT* a) {
        a->i = 0;
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_FLOAT;
        a->tensor->float32s = desc.uniforms;
        a->b = false;
    });
    net->oplists.push_back(std::move(op));

    auto offset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(offset);
    printf("Model: %zu bytes | op: %s\n", fbb.GetSize(), desc.type);

    // Run
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
    interp->resizeTensor(input, {1, 1, H, W});
    interp->resizeSession(sess);

    int32_t inData[W*H];
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            if (y%2==0 && x%2==0) inData[y*W+x] = 100;
            else if (y%2==0 && x%2==1) inData[y*W+x] = 200;
            else if (y%2==1 && x%2==0) inData[y*W+x] = 300;
            else inData[y*W+x] = 400;
        }

    auto hostIn = MNN::Tensor::create({1, 1, H, W}, input->getType(), inData, MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);

    auto start = std::chrono::high_resolution_clock::now();
    interp->runSession(sess);
    auto end = std::chrono::high_resolution_clock::now();
    printf("Run: %.2f ms\n", std::chrono::duration<double, std::milli>(end-start).count());

    auto out = interp->getSessionOutput(sess, "tensor_1");
    float* outData = new float[out->elementSize()]();
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    int nz = 0;
    for (int i = 0; i < out->elementSize(); i++) if (fabsf(outData[i]) > 1e-6f) nz++;
    printf("Output: %d/%d non-zero | type=%s\n\n", nz, (int)out->elementSize(), desc.type);

    delete[] outData;
    delete bc;
    delete interp;
    return 0;
}
