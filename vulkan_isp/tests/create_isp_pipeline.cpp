//
// create_isp_pipeline.cpp - Build ISP pipeline model with chained VulkanFuse shaders
//
// Builds an MNN flatbuffer model with 6 interconnected Extra ops, one per ISP stage.
// Run: LD_LIBRARY_PATH=... ./create_isp_pipeline [output.mnn]
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "schema/current/MNN_generated.h"

using namespace MNN;

// SHADER_ORDER must match the SPIR-V files
enum ShaderStage {
    SHADER_UNPACK_BLC = 0,
    SHADER_DEMOSAIC_CCM,
    SHADER_FCS,
    SHADER_EE,
    SHADER_LDCI,
    SHADER_DISPLAY,
    SHADER_COUNT
};

struct StageConfig {
    const char* name;
    const char* filename;
    int inputCount;
    int outputCount;
    int inputDimensions[4];   // [N,C,H,W]
    int outputDimensions[4];  // [N,C,H,W]
    // Descriptor binding: for each I/O tensor, the descriptor set binding
    int inputBindings[4];
    int outputBindings[4];    // max 4 outputs
    bool needAutoTuning;
};

static StageConfig stages[SHADER_COUNT] = {
    { "unpack_blc",  "vulkan_isp/shader1_unpack_blc.spv",  1, 1, {1, 1, 2160, 3840}, {1, 4, 1080, 1920}, {1}, {2}, true },
    { "demosaic_ccm","vulkan_isp/shader2_demosaic_ccm.spv", 1, 1, {1, 4, 1080, 1920}, {1, 3, 1080, 1920}, {1}, {2}, true },
    { "fcs",         "vulkan_isp/shader3_fcs.spv",          1, 1, {1, 3, 1080, 1920}, {1, 3, 1080, 1920}, {1}, {2}, true },
    { "ee",          "vulkan_isp/shader4_ee.spv",           1, 1, {1, 3, 1080, 1920}, {1, 3, 1080, 1920}, {1}, {2}, true },
    { "ldci",        "vulkan_isp/shader5_ldci.spv",         1, 1, {1, 3, 1080, 1920}, {1, 3, 1080, 1920}, {1}, {2}, true },
    { "display",     "vulkan_isp/shader6_display.spv",     1, 1, {1, 3, 1080, 1920}, {1, 3,  540,  960}, {1}, {2}, true },
};

// Read binary file
static std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.good()) { fprintf(stderr, "Failed to open: %s\n", path); return {}; }
    size_t sz = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

int main(int argc, char* argv[]) {
    const char* outPath = argc > 1 ? argv[1] : "/data/data/com.termux/files/home/softisp/vulkan_isp/isp_pipeline.mnn";
    
    // Read SPIR-V for each stage
    std::vector<uint8_t> spirv[SHADER_COUNT];
    for (int s = 0; s < SHADER_COUNT; s++) {
        char path[256];
        // Try relative to MNN build dir or absolute
        snprintf(path, sizeof(path), "/data/data/com.termux/files/home/softisp/%s", stages[s].filename);
        spirv[s] = readFile(path);
        if (spirv[s].empty()) {
            fprintf(stderr, "Cannot read SPIR-V for stage %d (%s)\n", s, stages[s].name);
        } else {
            printf("Stage %d (%s): %zu bytes SPIR-V\n", s, stages[s].name, spirv[s].size());
        }
    }
    
    // Build model using MNN's flatbuffer API directly.
    // We'll use the Interpreter approach since it's simpler.
    
    // --- Alternative: Build model with Interpreter programmatically ---
    // MNN::Interpreter doesn't have a clean builder API for this.
    // Let's use the flatbuffers schema directly.
    
    flatbuffers::FlatBufferBuilder fbb(256 * 1024);
    
    // Create tensor name strings
    std::vector<flatbuffers::Offset<flatbuffers::String>> tensorNamesFB;
    int totalTensors = 1 + SHADER_COUNT; // input + 1 output per stage
    for (int i = 0; i < totalTensors; i++) {
        char name[32];
        snprintf(name, sizeof(name), "tensor_%d", i);
        tensorNamesFB.push_back(fbb.CreateString(name));
    }
    
    // Create string for "spirv", "input", "const", "global_size", "output_shape"
    auto key_spirv       = fbb.CreateString("spirv");
    auto key_input       = fbb.CreateString("input");
    auto key_const       = fbb.CreateString("const");
    auto key_global_size = fbb.CreateString("global_size");
    auto key_output_shape = fbb.CreateString("output_shape");
    auto key_group_size  = fbb.CreateString("group_size");
    auto typeName        = fbb.CreateString("IspPipeline");
    
    // Input op (tensor 0)
    {
        // Don't create Input op - just let the first tensor be a free input
    }
    
    // We need to have NO Input op (it's implicit) and build Extra ops with:
    // output_shape attributes for proper sizing
    
    std::vector<flatbuffers::Offset<MNN::Op>> ops;
    
    // For each stage, create an Extra op
    int tensorIdx = 1;  // output tensors start at 1 (input is 0)
    
    for (int s = 0; s < SHADER_COUNT; s++) {
        if (spirv[s].empty()) continue;
        
        int inTensorIdx  = s == 0 ? 0 : tensorIdx - 1;  // chain: prev output → current input
        int outTensorIdx = tensorIdx;
        tensorIdx++;
        
        auto& cfg = stages[s];
        auto& spv = spirv[s];
        
        // Build attributes
        std::vector<flatbuffers::Offset<MNN::Attribute>> attrs;
        
        // spirv attribute
        auto spirvData = fbb.CreateVector(spv.data(), spv.size());
        auto spirvBlob = MNN::CreateBlob(fbb, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                          MNN::Blob::TableType_Blob, 0, 0, 0, 0, 0.0f, 0.0f, 
                                          spirvData, 0, 0, 0, 0);
        // Actually let me use the simpler CreateBlob function
        // Let's just use the flatbuffers-native approach
        
        // SPIR-V blob: DataType_DT_INT8, int8s data
        auto spvVec = fbb.CreateVector(spv.data(), spv.size());
        auto blob = MNN::CreateBlob(fbb, 
            MNN::DataType_DT_INT8,    // dataType
            MNN::MNN_DATA_FORMAT_NCHW,// dataFormat
            0,                        // float32s
            0,                        // int32s
            0,                        // uint8s (don't set)
            spvVec,                   // int8s
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        );
        
        attrs.push_back(MNN::CreateAttribute(fbb, key_spirv, 0, 0, 0, blob, 0, 0, 0));
        
        // Global size attribute
        std::vector<int32_t> gs = {
            cfg.outputDimensions[3],  // W
            cfg.outputDimensions[2],  // H
            1
        };
        auto gsVec = fbb.CreateVector(gs);
        auto gsBlob = MNN::CreateBlob(fbb, MNN::DataType_DT_INT32, MNN::MNN_DATA_FORMAT_NCHW,
            0, gsVec, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        attrs.push_back(MNN::CreateAttribute(fbb, key_global_size, 0, 0, 0, gsBlob, 0, 0, 0));
        
        // output_shape attribute - tells ShapeExtra what the output tensor size should be
        std::vector<int32_t> outShape = {
            cfg.outputDimensions[0],
            cfg.outputDimensions[1],
            cfg.outputDimensions[2],
            cfg.outputDimensions[3]
        };
        auto outShapeVec = fbb.CreateVector(outShape);
        auto outShapeBlob = MNN::CreateBlob(fbb, MNN::DataType_DT_INT32, MNN::MNN_DATA_FORMAT_NCHW,
            0, outShapeVec, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        attrs.push_back(MNN::CreateAttribute(fbb, key_output_shape, 0, 0, 0, outShapeBlob, 0, 0, 0));
        
        // Input binding attribute: [io_type=0 (input), binding=1]
        // - attr->i() = tensor position (0 for first input)
        // - list = [io_type=tensor_idx, binding]
        {
            auto listData = fbb.CreateVector(std::vector<int32_t>{inTensorIdx, cfg.inputBindings[0]});
            auto listVal = MNN::CreateListValue(fbb, 0, 0, 0, listData, 0);
            attrs.push_back(MNN::CreateAttribute(fbb, key_input, 0, 0, 0, 0, listVal, 0));
        }
        
        // Output binding attribute: [io_type=1 (output), binding=2]
        {
            auto listData = fbb.CreateVector(std::vector<int32_t>{1, cfg.outputBindings[0]});
            auto listVal = MNN::CreateListValue(fbb, 0, 0, 0, listData, 0);
            attrs.push_back(MNN::CreateAttribute(fbb, key_input, 1, 0, 0, 0, listVal, 0));
        }
        
        // const uniform attribute (dummy values for now)
        // binding=3 (or higher)
        int uniformBinding = 3;
        if (s == 0) {
            // UnpackBLC uniform: input_width, input_height, output_w, output_h, sensor_max,
            // blc_r, blc_gr, blc_gb, blc_b, wb_r, wb_gr, wb_gb, wb_b
            std::vector<float> uniform = {
                3840.0f, 2160.0f,                               // input dimensions (full sensor)
                1920.0f, 1080.0f,                               // output dimensions (after unpack)
                1023.0f,                                        // sensor max (10-bit)
                0.0f, 0.0f, 0.0f, 0.0f,                         // BLC offsets (R, Gr, Gb, B)
                1.0f, 1.0f, 1.0f, 1.0f                          // WB gains (R, Gr, Gb, B)
            };
            auto uniVec = fbb.CreateVector(uniform);
            auto uniBlob = MNN::CreateBlob(fbb, MNN::DataType_DT_FLOAT, MNN::MNN_DATA_FORMAT_NCHW,
                uniVec, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            attrs.push_back(MNN::CreateAttribute(fbb, key_const, uniformBinding, 0, 0, uniBlob, 0, 1));
            // b=true = uniform buffer
        } else if (s == 1) {
            // DemosaicCCM uniform
            std::vector<float> uniform = {
                1920.0f, 1080.0f,                               // input dimensions
                1920.0f, 1080.0f,                               // output dimensions
                0.0f,                                            // padding/flag
                1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f  // CCM 3x3 identity
            };
            auto uniVec = fbb.CreateVector(uniform);
            auto uniBlob = MNN::CreateBlob(fbb, MNN::DataType_DT_FLOAT, MNN::MNN_DATA_FORMAT_NCHW,
                uniVec, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            attrs.push_back(MNN::CreateAttribute(fbb, key_const, uniformBinding, 0, 0, uniBlob, 0, 1));
        } else {
            // Other stages: just need width/height
            std::vector<float> uniform = {
                (float)cfg.outputDimensions[3],  // W
                (float)cfg.outputDimensions[2],  // H
            };
            auto uniVec = fbb.CreateVector(uniform);
            auto uniBlob = MNN::CreateBlob(fbb, MNN::DataType_DT_FLOAT, MNN::MNN_DATA_FORMAT_NCHW,
                uniVec, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            attrs.push_back(MNN::CreateAttribute(fbb, key_const, uniformBinding, 0, 0, uniBlob, 0, 1));
        }
        
        // Create Extra op
        auto attrVec = fbb.CreateVector(attrs);
        auto extra    = MNN::CreateExtra(fbb, typeName, attrVec);
        
        // No InputIndexes vector = from previous tensor
        auto inIdxVec = fbb.CreateVector(std::vector<int32_t>{inTensorIdx});
        auto outIdxVec = fbb.CreateVector(std::vector<int32_t>{outTensorIdx});
        
        ops.push_back(MNN::CreateOp(fbb, 
            MNN::OpType_Extra,        // type
            MNN::OpParameter_Extra,   // mainType
            extra.o,                  // main
            0,                        // defaultDim
            fbb.CreateString(cfg.name),// name
            inIdxVec,                 // inputIndexes
            outIdxVec,                // outputIndexes
            0                         // device
        ));
    }
    
    // Create tensor describes (needed for MNN to know tensor shapes)
    std::vector<flatbuffers::Offset<MNN::TensorDescribe>> tensorDescs;
    // Tensor 0: input [1,1,2160,3840] int16
    {
        std::vector<int32_t> dims = {1, 1, 2160, 3840};
        auto dimVec = fbb.CreateVector(dims);
        auto blob = MNN::CreateBlob(fbb, MNN::DataType_DT_INT16, MNN::MNN_DATA_FORMAT_NCHW,
            0, dimVec, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        tensorDescs.push_back(MNN::CreateTensorDescribe(fbb, blob, 0, fbb.CreateString("tensor_0")));
    }
    // Tensor 1-6: output of each stage
    for (int s = 0; s < SHADER_COUNT; s++) {
        auto& cfg = stages[s];
        std::vector<int32_t> dims = {
            cfg.outputDimensions[0],
            cfg.outputDimensions[1],
            cfg.outputDimensions[2],
            cfg.outputDimensions[3]
        };
        auto dimVec = fbb.CreateVector(dims);
        auto blob = MNN::CreateBlob(fbb, MNN::DataType_DT_FLOAT, MNN::MNN_DATA_FORMAT_NCHW,
            0, dimVec, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        char name[32];
        snprintf(name, sizeof(name), "tensor_%d", s + 1);
        tensorDescs.push_back(MNN::CreateTensorDescribe(fbb, blob, s + 1, fbb.CreateString(name)));
    }
    
    // Build the Net
    auto opVec     = fbb.CreateVector(ops);
    auto tnVec     = fbb.CreateVector(tensorNamesFB);
    auto tdVec     = fbb.CreateVector(tensorDescs);
    auto bizCode   = fbb.CreateString("ISPPipeline");
    
    auto net = MNN::CreateNet(fbb, 0, 0, opVec, tnVec, tdVec, 0, 0, 0, 0, bizCode);
    fbb.Finish(net);
    
    // Save
    auto buf = fbb.GetBufferPointer();
    auto len = fbb.GetSize();
    
    FILE* f = fopen(outPath, "wb");
    if (!f) { fprintf(stderr, "Failed to write: %s\n", outPath); return 1; }
    fwrite(buf, 1, len, f);
    fclose(f);
    printf("Written: %s (%zu bytes)\n", outPath, len);
    
    // Verify by creating Interpreter
    auto interp = Interpreter::createFromBuffer(buf, len);
    if (interp) {
        printf("Interpreter created OK\n");
        printf("  Session input: %s\n", interp->getSessionInput(nullptr, "tensor_0") ? "tensor_0" : "none");
        printf("  Session output: %s\n", interp->getSessionOutput(nullptr, "tensor_0") ? "none" : "?");
        
        // Try to create a session to verify shapes
        ScheduleConfig cfg;
        cfg.type = MNN_FORWARD_CPU;
        auto sess = interp->createSession(cfg);
        if (sess) {
            printf("Session created OK (CPU)\n");
            interp->releaseSession(sess);
        }
        
        delete interp;
    } else {
        fprintf(stderr, "Failed to create Interpreter!\n");
        return 1;
    }
    
    return 0;
}
