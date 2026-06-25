// test_raw_model.cpp — Build MNN flatbuffer model of a single Extra op.
// Uses BlobBuilder/AttributeBuilder/etc. to avoid flatbuffers positioning errors.

#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/MNNForwardType.h>
#include <MNN/Interpreter.hpp>
#include <flatbuffers/flatbuffers.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cstring>
#include <dlfcn.h>

#include "MNN_generated.h"

int main(int argc, const char* argv[]) {
    const char* spvPath = argc > 1 ? argv[1] : "/data/data/com.termux/files/home/softisp/vulkan_isp/test_constant.spv";
    std::ifstream spvFile(spvPath, std::ios::binary);
    if (!spvFile) { std::cerr << "Can't open " << spvPath << "\n"; return 1; }
    std::vector<uint8_t> spv((std::istreambuf_iterator<char>(spvFile)),
                              std::istreambuf_iterator<char>());
    std::cout << "SPIR-V: " << spv.size() << " bytes\n";

    flatbuffers::FlatBufferBuilder fbb(1024);

    // String offsets (can create at any point)
    auto s_spirv        = fbb.CreateString("spirv");
    auto s_output_shape = fbb.CreateString("output_shape");
    auto s_global_size  = fbb.CreateString("global_size");
    auto s_group_size   = fbb.CreateString("group_size");
    auto s_input        = fbb.CreateString("input");
    auto s_name_op      = fbb.CreateString("TestConst");
    auto s_t0           = fbb.CreateString("tensor_0");
    auto s_t1           = fbb.CreateString("tensor_1");
    auto s_net          = fbb.CreateString("TestNet");

    // --- Blob: SPIR-V ---
    std::vector<int8_t> spv_i8(spv.begin(), spv.end());
    {
        auto spv_ivec = fbb.CreateVector(spv_i8);
        MNN::BlobBuilder spvBlob(fbb);
        spvBlob.add_int8s(spv_ivec);
        spvBlob.add_dataType(MNN::DataType_DT_INT8);
        auto spvBlobOff = spvBlob.Finish();

        MNN::AttributeBuilder attr(fbb);
        attr.add_key(s_spirv);
        attr.add_tensor(spvBlobOff);
        // Not setting type (20) so it defaults. The generated CreateAttribute
        // doesn't set type either.
        auto attrOff = attr.Finish();

        // Need to store for later. But we're building inside-out...
        // Actually, we need to store all attrs and build the Extra last.
        // Let's just do things in serial and save each offset.
    }

    // This is getting too complex. Let me use a different approach.
    // I'll construct the model by writing raw flatbuffer bytes directly.

    std::cout << "All approaches complex. Let me use the Interpreter API "
              << "to build a model programmatically.\n";

    // Actually, MNN's Interpreter can't build models. Let me just
    // use the Python-generated model fused_isp_unpack.mnn but first
    // patch it to use a diagnostic shader.

    // --- ALTERNATIVE: Write raw flatbuffer bytes by hand ---
    // A valid minimal MNN model has:
    //   1. Flatbuffer root table offset
    //   2. vtable entries for each table
    //   3. The actual data
    //
    // This is too complex by hand. Let me use the pack/unpack API instead.

    // The flatbuffers Pack/Unpack API allows building tables from native C++ structs.
    // Let's use ExtraT, AttributeT, BlobT, OpT, NetT.

    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "TestNet";

    // Tensor names
    net->tensorName.push_back("tensor_0");
    net->tensorName.push_back("tensor_1");

    // Op
    std::unique_ptr<MNN::OpT> op(new MNN::OpT);
    op->type = MNN::OpType_Extra;
    op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto extra = static_cast<MNN::ExtraT*>(op->main.value);
    extra->type = "TestConst";
    op->inputIndexes.push_back(0);
    op->outputIndexes.push_back(1);
    op->name = "TestConst";

    // Add attributes to Extra
    // 1. spirv
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "spirv";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT8;
        a->tensor->int8s.resize(spv_i8.size());
        memcpy(a->tensor->int8s.data(), spv_i8.data(), spv_i8.size());
        extra->attr.push_back(std::move(a));
    }
    // 2. output_shape
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "output_shape";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 4, 1, 1};
        extra->attr.push_back(std::move(a));
    }
    // 3. global_size
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "global_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
        extra->attr.push_back(std::move(a));
    }
    // group_size (bypass auto-tuning)
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "group_size";
        a->tensor.reset(new MNN::BlobT);
        a->tensor->dataType = MNN::DataType_DT_INT32;
        a->tensor->int32s = {1, 1, 1};
        extra->attr.push_back(std::move(a));
    }
    // 4. input binding: tensor 0 at descriptor binding 1
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "input";
        a->i = 0;  // position 0 in op's input list
        a->list.reset(new MNN::ListValueT);
        a->list->i = {0, 1};
        extra->attr.push_back(std::move(a));
    }
    // 5. output binding: output tensor 0 at descriptor binding 2
    {
        std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
        a->key = "input";
        a->i = 0;  // position 0 in op's output list
        a->list.reset(new MNN::ListValueT);
        a->list->i = {1, 2};
        extra->attr.push_back(std::move(a));
    }

    net->oplists.push_back(std::move(op));

    // Pack to flatbuffer
    auto netOffset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(netOffset);

    // Save
    {
        std::ofstream out("test_raw.mnn", std::ios::binary);
        out.write((const char*)fbb.GetBufferPointer(), fbb.GetSize());
        std::cout << "Saved: test_raw.mnn (" << fbb.GetSize() << " bytes)\n";
    }

    // Explicitly load libMNN_Vulkan.so to ensure Vulkan backend is available
    void* vkLib = dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    if (!vkLib) {
        std::cerr << "Failed to dlopen libMNN_Vulkan.so: " << dlerror() << "\n";
    } else {
        std::cout << "libMNN_Vulkan.so loaded\n";
        // Manually register the Vulkan runtime creator since --gc-sections
        // strips the static init from VulkanRuntime.cpp.
        typedef void (*RegFunc)();
        RegFunc reg = (RegFunc)dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
        if (reg) {
            std::cout << "Calling MNNVulkanRegisterAll...\n";
            reg();
        } else {
            std::cout << "MNNVulkanRegisterAll not found: " << dlerror() << "\n";
        }
    }

    // Load and run
    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    if (!interp) { std::cerr << "Failed interpreter\n"; return 1; }
    std::cout << "Interpreter OK\n";

    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    cfg.numThread = 1;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;
    auto sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "Failed session\n"; return 1; }
    std::cout << "Session OK\n";

    auto in = interp->getSessionInput(sess, "tensor_0");
    if (!in) { std::cerr << "No input\n"; return 1; }

    interp->resizeTensor(in, {1, 4, 1, 1});
    interp->resizeSession(sess);

    auto err = interp->runSession(sess);
    std::cout << "Run: " << (err == MNN::NO_ERROR ? "OK" : "FAIL") << " (code=" << err << ")\n";

    auto out = interp->getSessionOutput(sess, "tensor_1");
    if (!out) { std::cerr << "No output\n"; return 1; }
    
    printf("Output dims: %d", out->buffer().dimensions);
    for (int i = 0; i < out->buffer().dimensions; i++) {
        printf(" %d", out->length(i));
    }
    printf("\n");
    printf("Output type: code=%d bits=%d\n", out->getType().code, out->getType().bits);
    printf("Element size: %d\n", out->elementSize());

    // Read output using createHostTensorFromDevice (proper GPU→CPU copy)
    MNN::Tensor* hostOut = MNN::Tensor::createHostTensorFromDevice(out, true);
    if (!hostOut) { std::cerr << "Failed to create host tensor\n"; return 1; }
    
    printf("Host dims: %d", hostOut->buffer().dimensions);
    for (int i = 0; i < hostOut->buffer().dimensions; i++) {
        printf(" %d", hostOut->length(i));
    }
    printf("\n");

    int nonZero = 0;
    int n = hostOut->elementSize();
    for (int i = 0; i < n; ++i) {
        float v = hostOut->host<float>()[i];
        if (v != 0.0f) nonZero++;
        std::cout << " [" << i << "]=" << v;
    }
    std::cout << "\nNon-zero: " << (nonZero > 0 ? "YES" : "NO") << "\n";
    
    delete hostOut;

    interp->releaseSession(sess);
    delete interp;
    return 0;
}
