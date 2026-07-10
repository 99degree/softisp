#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <flatbuffers/flatbuffers.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cstring>
#include "MNN_generated.h"

int main() {
    const char* spvPath = "/data/data/com.termux/files/home/softisp/vulkan_isp/test_constant.spv";
    std::ifstream spvFile(spvPath, std::ios::binary);
    std::vector<uint8_t> spv((std::istreambuf_iterator<char>(spvFile)), std::istreambuf_iterator<char>());
    std::cout << "SPIR-V: " << spv.size() << " bytes\n";

    flatbuffers::FlatBufferBuilder fbb(1024);
    auto s_spirv = fbb.CreateString("spirv");
    auto s_output_shape = fbb.CreateString("output_shape");
    auto s_global_size = fbb.CreateString("global_size");
    auto s_input = fbb.CreateString("input");
    auto s_name_op = fbb.CreateString("TestConst");
    auto s_t0 = fbb.CreateString("tensor_0");
    auto s_t1 = fbb.CreateString("tensor_1");
    auto s_net = fbb.CreateString("TestNet");
    std::vector<int8_t> spv_i8(spv.begin(), spv.end());

    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "TestNet";
    net->tensorName.push_back("tensor_0");
    net->tensorName.push_back("tensor_1");

    std::unique_ptr<MNN::OpT> op(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = "TestConst";
    op->inputIndexes.push_back(0);
    op->outputIndexes.push_back(1);
    op->name = "TestConst";

    auto add_attr = [&](const std::string& key, auto&& setup) {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = key;
        setup(a.get());
        extra->attr.push_back(std::move(a));
    };
    add_attr("spirv", [&](MNN::AttributeT* a) {
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        a->tensor->int8s = spv_i8;
    });
    add_attr("output_shape", [](MNN::AttributeT* a) {
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 4, 1, 1};
    });
    add_attr("global_size", [](MNN::AttributeT* a) {
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
    });
    add_attr("input", [](MNN::AttributeT* a) {
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {0, 1};
    });
    add_attr("input", [](MNN::AttributeT* a) {
        a->i = 0;
        a->list.reset(new MNN::ListValueT);
        a->list->i = {1, 2};
    });

    net->oplists.push_back(std::move(op));
    auto netOffset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(netOffset);

    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    std::cout << "Interpreter OK\n";

    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_CPU;
    auto sess = interp->createSession(cfg);
    std::cout << "Session (CPU) " << (sess ? "OK" : "FAIL") << "\n";
    if (!sess) return 1;

    auto in = interp->getSessionInput(sess, "tensor_0");
    interp->resizeTensor(in, {1, 4, 1, 1});
    interp->resizeSession(sess);

    auto err = interp->runSession(sess);
    std::cout << "Run: " << (err == MNN::NO_ERROR ? "OK" : "FAIL") << "\n";

    auto out = interp->getSessionOutput(sess, "tensor_1");
    MNN::Tensor hostOut(out, MNN::Tensor::CAFFE);
    out->copyToHostTensor(&hostOut);
    int nonZero = 0;
    for (int i = 0; i < hostOut.elementSize(); ++i) {
        float v = hostOut.host<float>()[i];
        if (v != 0.0f) nonZero++;
        std::cout << " [" << i << "]=" << v;
    }
    std::cout << "\nNon-zero: " << (nonZero > 0 ? "YES" : "NO") << "\n";

    interp->releaseSession(sess);
    delete interp;
    return 0;
}
