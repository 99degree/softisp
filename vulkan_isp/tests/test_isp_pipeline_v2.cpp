//
// test_isp_pipeline.cpp - Multi-stage ISP pipeline test
// Tests all 6 shader stages chained together via VulkanFuse Extra ops.
//
// Build: g++ -std=c++11 -I/path/to/MNN/include -I/path/to/MNN -I/path/to/MNN/schema/current
//        -I/path/to/MNN/3rd_party/flatbuffers/include -o test_isp_pipeline test_isp_pipeline.cpp
//        -L/path/to/MNN/build_vk/OFF -L/path/to/MNN/build_vk/express/OFF -L/path/to/MNN/build_vk/source/backend/vulkan/OFF
//        -lMNN -lMNN_Express -lMNN_Vulkan -lpthread -ldl
//
// Run: LD_LIBRARY_PATH=... ./test_isp_pipeline
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/MNNForwardType.h>
#include "schema/current/MNN_generated.h"

using namespace MNN;

// Read entire file into vector
static std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.good()) { fprintf(stderr, "FAIL: cannot read %s\n", path); return {}; }
    size_t sz = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read((char*)buf.data(), sz);
    return buf;
}

// Convert float array to byte vector for SPIR-V int8s storage
static std::vector<int8_t> floatsToInt8s(const std::vector<float>& floats) {
    std::vector<int8_t> result(floats.size() * sizeof(float));
    memcpy(result.data(), floats.data(), result.size());
    return result;
}

// Shader stage configuration
struct Stage {
    const char* name;
    std::string spvPathStr;
    int inDim[4];       // input tensor dimensions
    int outDim[4];      // output tensor dimensions
    int inputBinding;   // descriptor binding for input
    int outputBinding;  // descriptor binding for output
    int inTensorIdx;    // input tensor index (set by builder)
    bool uniformIsUBO;
    int outTensorIdx;   // output tensor index (set by builder)
    int uniformBinding; // descriptor binding for uniforms
    bool uniformIsUBO;    // true=UBO, false=SSBO
    // Uniform data (varies per stage)
    std::vector<float> uniformData;
};

int main(int argc, const char* argv[]) {
    const char* outputPath = argc > 1 ? argv[1] : "isp_pipeline.mnn";
    
    std::string baseDir = "/data/data/com.termux/files/home/softisp/vulkan_isp/";
    
    // ============================================
    // Define stages with SPIR-V and uniform data
    // ============================================
    std::vector<Stage> stages;
    
    // Stage 0: UnpackBLC
    {
        Stage s;
        s.name = "unpack_blc";
        s.spvPathStr = baseDir + "shader1_unpack_blc.spv";
        int inDim[]  = {1, 1, 2160, 3840};
        int outDim[] = {1, 4, 1080, 1920};
        memcpy(s.inDim, inDim, sizeof(inDim));
        memcpy(s.outDim, outDim, sizeof(outDim));
        s.inputBinding = 1;
        s.outputBinding = 2;
        s.uniformBinding = 3;
        s.uniformIsUBO = false;
        s.uniformIsUBO = false;
        // Uniform: input_w, input_h, output_w, output_h, sensor_max,
        //          blc_r, blc_gr, blc_gb, blc_b, wb_r, wb_gr, wb_gb, wb_b
        s.uniformData = {3840, 2160, 1920, 1080, 1023.0f,
                         0, 0, 0, 0,    // BLC
                         1, 1, 1, 1};   // WB (identity)
        stages.push_back(std::move(s));
    }
    
    // Stage 1: DemosaicCCM
    {
        Stage s;
        s.name = "demosaic_ccm";
        s.spvPathStr = baseDir + "shader2_demosaic_ccm.spv";
        int inDim[]  = {1, 4, 1080, 1920};
        int outDim[] = {1, 3, 1080, 1920};
        memcpy(s.inDim, inDim, sizeof(inDim));
        memcpy(s.outDim, outDim, sizeof(outDim));
        s.inputBinding = 1;
        s.outputBinding = 2;
        s.uniformBinding = 3;
        s.uniformIsUBO = true;
        s.uniformIsUBO = true;
        // Uniform: input_w, input_h, output_w, output_h, pad,
        //          ccm 3x3 (identity)
        s.uniformData = {1920, 1080, 1920, 1080, 0.0f,
                         1,0,0, 0,1,0, 0,0,1};
        stages.push_back(std::move(s));
    }
    
    // Stage 2: FCS
    {
        Stage s;
        s.name = "fcs";
        s.spvPathStr = baseDir + "shader3_fcs.spv";
        int inDim[]  = {1, 3, 1080, 1920};
        int outDim[] = {1, 3, 1080, 1920};
        memcpy(s.inDim, inDim, sizeof(inDim));
        memcpy(s.outDim, outDim, sizeof(outDim));
        s.inputBinding = 1;
        s.outputBinding = 2;
        s.uniformBinding = 3;
        s.uniformIsUBO = true;
        s.uniformIsUBO = true;
        // Uniform: W, H
        s.uniformData = {1920, 1080};
        stages.push_back(std::move(s));
    }
    
    // Stage 3: EE
    {
        Stage s;
        s.name = "ee";
        s.spvPathStr = baseDir + "shader4_ee.spv";
        int inDim[]  = {1, 3, 1080, 1920};
        int outDim[] = {1, 3, 1080, 1920};
        memcpy(s.inDim, inDim, sizeof(inDim));
        memcpy(s.outDim, outDim, sizeof(outDim));
        s.inputBinding = 1;
        s.outputBinding = 2;
        s.uniformBinding = 3;
        s.uniformIsUBO = true;
        s.uniformIsUBO = true;
        // Uniform: W, H, strength
        s.uniformData = {1920, 1080, 1.0f};
        stages.push_back(std::move(s));
    }
    
    // Stage 4: LDCI
    {
        Stage s;
        s.name = "ldci";
        s.spvPathStr = baseDir + "shader5_ldci.spv";
        int inDim[]  = {1, 3, 1080, 1920};
        int outDim[] = {1, 3, 1080, 1920};
        memcpy(s.inDim, inDim, sizeof(inDim));
        memcpy(s.outDim, outDim, sizeof(outDim));
        s.inputBinding = 1;
        s.outputBinding = 2;
        s.uniformBinding = 3;
        s.uniformIsUBO = true;
        s.uniformIsUBO = true;
        // Uniform: W, H
        s.uniformData = {1920, 1080};
        stages.push_back(std::move(s));
    }
    
    // Stage 5: Display
    {
        Stage s;
        s.name = "display";
        s.spvPathStr = baseDir + "shader6_display.spv";
        int inDim[]  = {1, 3, 1080, 1920};
        int outDim[] = {1, 3, 540, 960};
        memcpy(s.inDim, inDim, sizeof(inDim));
        memcpy(s.outDim, outDim, sizeof(outDim));
        s.inputBinding = 1;
        s.outputBinding = 2;
        s.uniformBinding = 3;
        s.uniformIsUBO = true;
        s.uniformIsUBO = true;
        // Uniform: input_W, input_H, output_W, output_H
        s.uniformData = {1920, 1080, 960, 540};
        stages.push_back(std::move(s));
    }
    
    // ============================================
    // Assign tensor indices
    // ============================================
    int totalTensors = 1; // tensor 0 = input
    for (auto& s : stages) {
        s.inTensorIdx = totalTensors - 1;  // input is previous stage's output
        s.outTensorIdx = totalTensors;
        totalTensors++;
    }
    
    // ============================================
    // Read SPIR-V for all stages
    // ============================================
    std::vector<std::vector<uint8_t>> spirvData(stages.size());
    for (int i = 0; i < (int)stages.size(); i++) {
        spirvData[i] = readFile(stages[i].spvPathStr.c_str());
        if (spirvData[i].empty()) {
            // Already failed, just continue
        }
        printf("  Stage %d (%s): %zu bytes SPIR-V\n", i, stages[i].name, spirvData[i].size());
    }
    
    // ============================================
    // Build MNN model using flatbuffers T objects
    // ============================================
    flatbuffers::FlatBufferBuilder fbb(1024 * 1024);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "ISPPipeline";
    
    // Tensor names
    for (int i = 0; i < totalTensors; i++) {
        char name[32];
        snprintf(name, sizeof(name), "tensor_%d", i);
        net->tensorName.push_back(name);
    }
    
    // Create Extra ops for each stage
    for (int i = 0; i < (int)stages.size(); i++) {
        auto& s = stages[i];
        auto& spv = spirvData[i];
        
        std::unique_ptr<MNN::OpT> op(new MNN::OpT);
        op->type = MNN::OpType_Extra;
        op->main.type = MNN::OpParameter_Extra;
        op->main.value = new MNN::ExtraT();
        auto extra = static_cast<MNN::ExtraT*>(op->main.value);
        extra->type = "IspShader";
        op->inputIndexes.push_back(s.inTensorIdx);
        op->outputIndexes.push_back(s.outTensorIdx);
        op->name = s.name;
        
        // 1. spirv attribute
        {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = "spirv";
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT8;
            a->tensor->int8s.resize(spv.size());
            memcpy(a->tensor->int8s.data(), spv.data(), spv.size());
            extra->attr.push_back(std::move(a));
        }
        
        // 2. output_shape attribute (for ShapeExtra)
        {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = "output_shape";
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {s.outDim[0], s.outDim[1], s.outDim[2], s.outDim[3]};
            extra->attr.push_back(std::move(a));
        }
        
        // 3. global_size attribute
        {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = "global_size";
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {s.outDim[3], s.outDim[2], 1};
            extra->attr.push_back(std::move(a));
        }
        
        // 4. input binding: descriptor binding 1
        {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = "input";
            a->i = 0;  // position 0 in op's input list
            a->list.reset(new MNN::ListValueT);
            // [io_type=0 (input), binding]
            a->list->i = {0, s.inputBinding};
            extra->attr.push_back(std::move(a));
        }
        
        // 5. output binding: descriptor binding 2
        {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = "input";
            a->i = 0;  // position 0 in op's output list
            a->list.reset(new MNN::ListValueT);
            a->list->i = {1, s.outputBinding};
            extra->attr.push_back(std::move(a));
        }
        
        // 6. const uniform buffer
        if (!s.uniformData.empty()) {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = "const";
            a->i = s.uniformBinding;  // descriptor binding
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_FLOAT;
            a->tensor->float32s = s.uniformData;
            a->b = s.uniformIsUBO;  // true=UBO, false=SSBO
            extra->attr.push_back(std::move(a));
        }
        
        net->oplists.push_back(std::move(op));
    }
    
    // Tensor describes (for MNN shape inference)
    {
        // Input tensor 0: INT16 bayer [1,1,2160,3840]
        std::unique_ptr<MNN::TensorDescribeT> td(new MNN::TensorDescribeT);
        td->index = 0;
        td->name = "tensor_0";
        td->blob.reset(new MNN::BlobT);
        td->blob->dataType = MNN::DataType_DT_INT16;
        td->blob->dataFormat = MNN::MNN_DATA_FORMAT_NCHW;
        td->blob->int32s = {1, 1, 2160, 3840};
        net->extraTensorDescribe.push_back(std::move(td));
        
        // Output tensors
        for (int i = 0; i < (int)stages.size(); i++) {
            auto& s = stages[i];
            std::unique_ptr<MNN::TensorDescribeT> td(new MNN::TensorDescribeT);
            td->index = s.outTensorIdx;
            char name[32];
            snprintf(name, sizeof(name), "tensor_%d", s.outTensorIdx);
            td->name = name;
            td->blob.reset(new MNN::BlobT);
            td->blob->dataType = MNN::DataType_DT_FLOAT;
            td->blob->dataFormat = MNN::MNN_DATA_FORMAT_NCHW;
            td->blob->int32s = {s.outDim[0], s.outDim[1], s.outDim[2], s.outDim[3]};
            net->extraTensorDescribe.push_back(std::move(td));
        }
    }
    
    // Pack and save
    auto netOffset = MNN::Net::Pack(fbb, net.get());
    fbb.Finish(netOffset);
    
    {
        std::ofstream out(outputPath, std::ios::binary);
        out.write((const char*)fbb.GetBufferPointer(), fbb.GetSize());
        printf("\nModel saved: %s (%zu bytes)\n", outputPath, fbb.GetSize());
    }
    
    // ============================================
    // Load and initialize Vulkan backend
    // ============================================
    void* vkLib = dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    if (!vkLib) {
        fprintf(stderr, "FAIL: dlopen libMNN_Vulkan.so: %s\n", dlerror());
        return 1;
    }
    printf("libMNN_Vulkan.so loaded\n");
    
    typedef void (*RegFunc)();
    RegFunc reg = (RegFunc)dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) {
        printf("Calling MNNVulkanRegisterAll...\n");
        reg();
    } else {
        printf("MNNVulkanRegisterAll not found: %s\n", dlerror());
    }
    
    // ============================================
    // Create Interpreter and run
    // ============================================
    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    if (!interp) { fprintf(stderr, "FAIL: create Interpreter\n"); return 1; }
    printf("Interpreter OK\n");
    
    ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    cfg.numThread = 1;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;
    
    auto sess = interp->createSession(cfg);
    if (!sess) { fprintf(stderr, "FAIL: create Session\n"); return 1; }
    printf("Session OK\n");
    
    // Get input tensor and fill with test data
    auto input = interp->getSessionInput(sess, "tensor_0");
    if (!input) { fprintf(stderr, "FAIL: no input tensor\n"); return 1; }
    
    // Explicitly set input shape and resize
    std::vector<int> inputDims = {1, 1, 2160, 3840};
    interp->resizeTensor(input, inputDims);
    interp->resizeSession(sess);
    
    printf("Input dims: ");
    for (int i = 0; i < input->buffer().dimensions; i++)
        printf("%d ", input->length(i));
    printf("\n");
    printf("Input element size: %d\n", input->elementSize());
    
    // Fill input with test Bayer pattern
    // Create host tensor with allocated memory for INT16 input
    int inputElements = inputDims[0] * inputDims[1] * inputDims[2] * inputDims[3];
    auto hostPtr = (int16_t*)malloc(inputElements * sizeof(int16_t));
    if (!hostPtr) { fprintf(stderr, "FAIL: malloc\n"); return 1; }
    auto hostInput = MNN::Tensor::create(inputDims, input->getType(), hostPtr, MNN::Tensor::CAFFE);
    if (!hostInput) { fprintf(stderr, "FAIL: create host tensor\n"); free(hostPtr); return 1; }
    
    int16_t* inputData = hostInput->host<int16_t>();
    printf("Filling %d input elements with gradient pattern...\n", hostInput->elementSize());
    for (int i = 0; i < inputElements && i < 100; i++) {
        inputData[i] = (int16_t)((i % 1024));  // ramp 0-1023
    }
    for (int i = 100; i < inputElements; i++) {
        inputData[i] = (int16_t)((i * 997) % 1024);  // pseudo-random
    }
    
    input->copyFromHostTensor(hostInput);
    printf("Input copied to device\n");
    
    // Run
    auto code = interp->runSession(sess);
    printf("Run: %s (code=%d)\n", code == NO_ERROR ? "OK" : "FAIL", (int)code);
    
    // Check all output tensors
    for (int i = 0; i < (int)stages.size(); i++) {
        auto& s = stages[i];
        char name[32];
        snprintf(name, sizeof(name), "tensor_%d", s.outTensorIdx);
        
        auto out = interp->getSessionOutput(sess, name);
        if (!out) {
            printf("  Output '%s': FAIL (no tensor)\n", name);
            continue;
        }
        
        printf("  Output '%s' dims:", name);
        for (int j = 0; j < out->buffer().dimensions; j++)
            printf(" %d", out->length(j));
        printf(" type=code=%d,bits=%d\n", out->getType().code, out->getType().bits);
        
        // Read back
        auto hostOut = MNN::Tensor::createHostTensorFromDevice(out, true);
        if (hostOut && hostOut->buffer().host) {
            int nonZero = 0;
            int n = hostOut->elementSize();
            n = n > 16 ? 16 : n;  // first 16 elements
            for (int j = 0; j < n; j++) {
                float v = hostOut->host<float>()[j];
                if (v != 0.0f) nonZero++;
            }
            printf("    First 16 elements non-zero: %d/%d\n", nonZero, n);
            if (nonZero == 0) {
                printf("    ALL ZEROS!\n");
            } else {
                printf("    Values: ");
                for (int j = 0; j < 4 && j < n; j++)
                    printf("%.3f ", hostOut->host<float>()[j]);
                printf("\n");
            }
        }
        delete hostOut;
    }
    
    // Cleanup
    if (hostInput && hostInput->buffer().host) {
        free(hostInput->buffer().host);
    }
    delete hostInput;
    interp->releaseSession(sess);
    delete interp;
    printf("\nDone.\n");
    
    return 0;
}
