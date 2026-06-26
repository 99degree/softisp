// test_all_opsets.cpp — Verify all 6 ISP opsets individually
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
    if (!f.good()) return {};
    size_t sz = f.tellg(); f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

struct SpvFile {
    const char* path;
    const char* name;
} spv_files[6] = {
    {"shader1_unpack_blc.spv", "isp.unpack_blc"},
    {"shader2_demosaic_ccm.spv", "isp.demosaic_ccm"},
    {"shader3_fcs.spv", "isp.fcs"},
    {"shader4_ee.spv", "isp.ee"},
    {"shader5_ldci.spv", "isp.ldci"},
    {"shader6_display_simple.spv", "isp.display"},
};

int main() {
    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";

    // Test each opset in isolation
    for (int si = 0; si < 6; si++) {
        auto spv = readFile((base + spv_files[si].path).c_str());
        if (spv.empty()) { printf("SKIP: %s not found\n", spv_files[si].name); continue; }
        std::vector<int8_t> spv_i8(spv.size());
        memcpy(spv_i8.data(), spv.data(), spv.size());

        // Build model: single op
        flatbuffers::FlatBufferBuilder fbb(1024*1024);
        std::unique_ptr<MNN::NetT> net(new MNN::NetT);
        net->bizCode = spv_files[si].name;
        net->tensorName = {"tensor_0", "tensor_1"};

        isp::OpDesc desc;
        if (si == 0) desc = isp::UnpackBlc(8, 8);
        else if (si == 1) desc = isp::DemosaicCcm(8, 8);
        else if (si == 2) desc = isp::Fcs(8, 8);
        else if (si == 3) desc = isp::Ee(8, 8);
        else if (si == 4) desc = isp::Ldci(8, 8);
        else desc = isp::Display(8, 8);

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
        if (si == 0) {
            // Bayer INT32 input
            interp->resizeTensor(input, {1, 1, 8, 8});
            interp->resizeSession(sess);
            int32_t data[64];
            for (int y = 0; y < 8; y++)
                for (int x = 0; x < 8; x++)
                    data[y*8+x] = (y%2==0 && x%2==0) ? 100 : ((y%2==0 && x%2==1) ? 200 : ((y%2==1 && x%2==0) ? 300 : 400));
            auto hostIn = MNN::Tensor::create({1, 1, 8, 8}, input->getType(), data, MNN::Tensor::CAFFE);
            input->copyFromHostTensor(hostIn);
        } else {
            // FLOAT RGB input [0,1]
            interp->resizeTensor(input, {1, 3, 8, 8});
            interp->resizeSession(sess);
            float data[192];
            for (int i = 0; i < 192; i++) data[i] = 0.1f;
            auto hostIn = MNN::Tensor::create({1, 3, 8, 8}, input->getType(), data, MNN::Tensor::CAFFE);
            input->copyFromHostTensor(hostIn);
        }

        interp->runSession(sess);

        auto out = interp->getSessionOutput(sess, "tensor_1");
        float* outData = new float[out->elementSize()]();
        auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outData, MNN::Tensor::CAFFE);
        out->copyToHostTensor(hostOut);

        int nz = 0, tot = out->elementSize();
        for (int i = 0; i < tot; i++) if (fabsf(outData[i]) > 1e-6f) nz++;
        printf("[%s] %s — %d/%d non-zero %s\n",
               nz > 0 ? "PASS" : "FAIL",
               desc.type, nz, tot,
               nz > 0 ? "✅" : "❌");

        delete[] outData;
        delete bc;
        delete interp;
    }
    return 0;
}
