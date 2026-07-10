// create_fused_mnn.cpp — Build MNN model with fused ISP Extra op using MNN C++ Express API
// Compile: cd ~/MNN/build_vk && g++ -std=c++17 -O2 -I../include -I.. -I../source -I../schema/current -I../3rd_party/flatbuffers/include -L./OFF ../softisp/vulkan_isp/create_fused_mnn.cpp -lMNN -lpthread -ldl -o create_fused_mnn
// Run: LD_LIBRARY_PATH=./OFF ./create_fused_mnn

#include <MNN/Interpreter.hpp>
#include <MNN/expr/ExprCreator.hpp>
#include <MNN/expr/Expr.hpp>
#include <MNN/expr/Module.hpp>
#include <MNN/MNNDefine.h>
#include <MNN/AutoTime.hpp>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <memory>
#include <string>

using namespace MNN;
using namespace MNN::Express;

static std::vector<uint8_t> load_spirv(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    size_t size = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> data(size);
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

int main(int argc, char* argv[]) {
    const char* spirv_path = argc > 1 ? argv[1] : "/data/data/com.termux/files/home/softisp/vulkan_isp/fused_isp.spv";
    const char* output_path = argc > 2 ? argv[2] : "/data/data/com.termux/files/home/softisp/vulkan_isp/fused_isp.mnn";

    auto spirv = load_spirv(spirv_path);
    if (spirv.empty()) {
        std::cerr << "Failed to load SPIR-V from " << spirv_path << std::endl;
        return 1;
    }
    std::cout << "Loaded SPIR-V: " << spirv.size() << " bytes" << std::endl;

    // Build model using MNN Express API
    // We need to create an Extra op with SPIR-V in attributes
    // Express API doesn't directly support Extra ops with custom attributes
    // So we build the flatbuffers Net directly using MNN's internal API

    // Approach: Use the flatbuffers builder through MNN's Net class
    // But there's no public API for this. Let's use MNN's serialize API.

    // Alternative: Create ONNX -> MNNConvert
    // But MNNConvert might not preserve Extra ops

    std::cout << "MNN Express API doesn't expose Extra op creation with SPIR-V attributes." << std::endl;
    std::cout << "Need to use flatbuffers builder directly or ONNX -> MNNConvert pipeline." << std::endl;

    // For now, write the SPIR-V and attributes to a binary file
    // that can be loaded by a custom model loader
    std::ofstream out("/data/data/com.termux/files/home/softisp/vulkan_isp/fused_isp_data.bin", std::ios::binary);
    if (out) {
        // Write header: magic + version + sizes
        uint32_t magic = 0x49535046; // "FSPI"
        uint32_t version = 1;
        uint32_t spirv_size = spirv.size();
        uint32_t attr_count = 7;
        out.write(reinterpret_cast<char*>(&magic), 4);
        out.write(reinterpret_cast<char*>(&version), 4);
        out.write(reinterpret_cast<char*>(&spirv_size), 4);
        out.write(reinterpret_cast<char*>(&attr_count), 4);
        out.write(reinterpret_cast<const char*>(spirv.data()), spirv.size());

        // Write attribute data (simplified)
        // This is just for reference - real model needs flatbuffers
        std::cout << "Wrote SPIR-V to binary for reference" << std::endl;
    }

    std::cout << "\n=== Next Steps ===" << std::endl;
    std::cout << "1. Use Python flatbuffers (with all includes) to build MNN model" << std::endl;
    std::cout << "2. Or use MNN's C++ internal API (not public)" << std::endl;
    std::cout << "3. Or create ONNX with custom op -> MNNConvert" << std::endl;

    return 0;
}