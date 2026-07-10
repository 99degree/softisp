// test_express_extra.cpp — Test VulkanFuse via MNN Express API
//
// Build:
//   g++ -std=c++11 -I~/MNN/include -o test_express_extra test_express_extra.cpp \
//       -L~/MNN/build_vk/OFF -lMNN \
//       -L~/MNN/build_vk/express/OFF -lMNN_Express \
//       -L~/MNN/build_vk/source/backend/vulkan/OFF -lMNN_Vulkan \
//       -lpthread -ldl

#include <MNN/expr/ExprCreator.hpp>
#include <MNN/expr/Executor.hpp>
#include <MNN/expr/Expr.hpp>
#include <MNN/Interpreter.hpp>
#include <fstream>
#include <iostream>
#include <vector>

using namespace MNN::Express;

int main(int argc, const char* argv[]) {
    // Read SPIR-V
    const char* spvPath = argc > 1 ? argv[1] : "/data/data/com.termux/files/home/softisp/vulkan_isp/test_constant.spv";
    std::ifstream spvFile(spvPath, std::ios::binary);
    if (!spvFile) { std::cerr << "Can't open " << spvPath << "\n"; return 1; }
    std::vector<uint8_t> spv((std::istreambuf_iterator<char>(spvFile)),
                              std::istreambuf_iterator<char>());
    std::cout << "SPIR-V: " << spv.size() << " bytes\n";

    // Set Vulkan backend
    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_VULKAN;
    config.numThread = 1;
    auto executor = Executor::newExecutor(MNN_FORWARD_VULKAN, config, 1);
    ExecutorScope scope(executor);

    // Create input variable (not used by shader)
    auto input = _Input({1, 4, 1, 1}, NCHW, halide_type_of<float>());
    // Fill with dummy data
    std::fill(input->writeMap<float>(), input->writeMap<float>() + 4, 0.0f);

    // Create ExtraT manually with our SPIR-V
    std::shared_ptr<MNN::ExtraT> extra(new MNN::ExtraT);
    extra->type = "TestConst";

    // Add attributes
    // Spirv attribute
    {
        std::unique_ptr<MNN::AttributeT> attr(new MNN::AttributeT);
        attr->key = "spirv";
        attr->tensor.reset(new MNN::BlobT);
        attr->tensor->dataType = MNN::DataType_DT_INT8;
        attr->tensor->int8s.resize(spv.size());
        memcpy(attr->tensor->int8s.data(), spv.data(), spv.size());
        extra->attr.push_back(std::move(attr));
    }
    // output_shape
    {
        std::unique_ptr<MNN::AttributeT> attr(new MNN::AttributeT);
        attr->key = "output_shape";
        attr->tensor.reset(new MNN::BlobT);
        attr->tensor->dataType = MNN::DataType_DT_INT32;
        attr->tensor->int32s = {1, 4, 1, 1};
        extra->attr.push_back(std::move(attr));
    }
    // global_size [1,1,1]
    {
        std::unique_ptr<MNN::AttributeT> attr(new MNN::AttributeT);
        attr->key = "global_size";
        attr->tensor.reset(new MNN::BlobT);
        attr->tensor->dataType = MNN::DataType_DT_INT32;
        attr->tensor->int32s = {1, 1, 1};
        extra->attr.push_back(std::move(attr));
    }
    // input binding: tensor position 0 at descriptor binding 1
    {
        std::unique_ptr<MNN::AttributeT> attr(new MNN::AttributeT);
        attr->key = "input";
        attr->i = 0;  // position 0 in op's input list
        attr->list.reset(new MNN::ListValueT);
        attr->list->i = {0, 1};  // is_output=0, binding=1
        extra->attr.push_back(std::move(attr));
    }
    // output binding: tensor position 0 at descriptor binding 2
    {
        std::unique_ptr<MNN::AttributeT> attr(new MNN::AttributeT);
        attr->key = "input";
        attr->i = 0;  // position 0 in op's output list
        attr->list.reset(new MNN::ListValueT);
        attr->list->i = {1, 2};  // is_output=1, binding=2
        extra->attr.push_back(std::move(attr));
    }

    // Create Extra op
    auto x = _Extra(extra.get(), {input});
    std::cout << "Extra op created: " << x->info()->size() << " outputs\n";

    // Print output info
    auto info = x->getInfo();
    if (info) {
        std::cout << "Output dims:";
        for (auto d : info->dim) std::cout << " " << d;
        std::cout << "\n";
    }

    // Read output
    auto outPtr = x->readMap<float>();
    if (outPtr) {
        int nonZero = 0;
        for (int i = 0; i < info ? info->size : 4; ++i) {
            float v = outPtr[i];
            if (v != 0.0f) nonZero++;
            std::cout << " [" << i << "]=" << v;
        }
        std::cout << "\nNon-zero: " << (nonZero > 0 ? "YES" : "NO") << "\n";
    } else {
        std::cout << "readMap returned null\n";
    }

    return 0;
}
