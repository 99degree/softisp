// test_minimal.cpp — Minimal VulkanFuse test with direct flatbuffer model creation
// Creates an MNN model with a single Extra op containing a trivial SPIR-V shader.
//
// Build: g++ -std=c++11 -I~/MNN/include -I~/MNN -o test_minimal test_minimal.cpp \
//        -L~/MNN/build_vk/OFF -lMNN -L~/MNN/build_vk/express/OFF -lMNN_Express \
//        -L~/MNN/build_vk/source/backend/vulkan/OFF -lMNN_Vulkan -Wl,-rpath,. \
//        -lpthread -ldl

#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/MNNForwardType.h>
#include <MNN/Interpreter.hpp>
#include <flatbuffers/flatbuffers.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cstring>

// Include generated MNN flatbuffer headers
#include "MNN_generated.h"

int main(int argc, const char* argv[]) {
    // Read SPIR-V from file
    const char* spvPath = argc > 1 ? argv[1] : "/data/data/com.termux/files/home/softisp/vulkan_isp/test_constant.spv";
    std::ifstream spvFile(spvPath, std::ios::binary);
    if (!spvFile) { std::cerr << "Can't open " << spvPath << "\n"; return 1; }
    std::vector<uint8_t> spv((std::istreambuf_iterator<char>(spvFile)),
                              std::istreambuf_iterator<char>());
    std::cout << "SPIR-V: " << spv.size() << " bytes\n";

    // Build MNN model with flatbuffers
    flatbuffers::FlatBufferBuilder fbb;

    // Create strings
    auto name_spirv = fbb.CreateString("spirv");
    auto name_output_shape = fbb.CreateString("output_shape");
    auto name_global_size = fbb.CreateString("global_size");
    auto name_input = fbb.CreateString("input");
    auto name_op = fbb.CreateString("TestConst");
    auto name_t0 = fbb.CreateString("tensor_0");
    auto name_t1 = fbb.CreateString("tensor_1");
    auto name_net = fbb.CreateString("TestNet");

    // SPIR-V blob (int8s) - cast to int8_t
    auto spv_i8 = std::vector<int8_t>(spv.begin(), spv.end());
    auto spirv_vec = fbb.CreateVector(spv_i8);
    MNN::BlobBuilder spv_blob(fbb);
    spv_blob.add_int8s(spirv_vec);
    spv_blob.add_dataType(MNN::DataType_DT_INT8);
    auto spirv_blob = spv_blob.Finish();

    // Output shape [1, 4, 1, 1]
    MNN::BlobBuilder shape_blob(fbb);
    std::vector<int32_t> dims_vec_data = {1, 4, 1, 1};
    auto dims_vec = fbb.CreateVector(dims_vec_data);
    shape_blob.add_int32s(dims_vec);
    auto shape_blob_fin = shape_blob.Finish();

    // Input attribute: tensor 0, list=[0,1] (is_output=0, binding=1)
    auto list_i = fbb.CreateVector(std::vector<int32_t>{0, 1});
    MNN::ListValueBuilder list_b(fbb);
    list_b.add_i(list_i);
    auto list_obj = list_b.Finish();

    // Output attribute: tensor 0, list=[1,2] (is_output=1, binding=2)
    auto list_i2 = fbb.CreateVector(std::vector<int32_t>{1, 2});
    MNN::ListValueBuilder list_b2(fbb);
    list_b2.add_i(list_i2);
    auto list_obj2 = list_b2.Finish();

    // Build attributes
    std::vector<flatbuffers::Offset<MNN::Attribute>> attrs;
    
    // spirv
    MNN::AttributeBuilder attr_spv(fbb);
    attr_spv.add_key(name_spirv);
    attr_spv.add_tensor(spirv_blob);
    attrs.push_back(attr_spv.Finish());
    
    // output_shape
    MNN::AttributeBuilder attr_os(fbb);
    attr_os.add_key(name_output_shape);
    attr_os.add_tensor(shape_blob_fin);
    attrs.push_back(attr_os.Finish());
    
    // global_size [1, 1, 1]
    MNN::BlobBuilder gs_blob(fbb);
    auto gs_vec = fbb.CreateVector(std::vector<int32_t>{1, 1, 1});
    gs_blob.add_int32s(gs_vec);
    auto gs_blob_fin = gs_blob.Finish();
    MNN::AttributeBuilder attr_gs(fbb);
    attr_gs.add_key(name_global_size);
    attr_gs.add_tensor(gs_blob_fin);
    attrs.push_back(attr_gs.Finish());
    
    // input attr (for input tensor 0, binding 1)
    MNN::AttributeBuilder attr_in(fbb);
    attr_in.add_key(name_input);
    attr_in.add_i(0);  // position in op's input list
    attr_in.add_list(list_obj);
    attrs.push_back(attr_in.Finish());
    
    // input attr (for output tensor 0, binding 2)
    MNN::AttributeBuilder attr_out(fbb);
    attr_out.add_key(name_input);
    attr_out.add_i(0);  // position in op's output list
    attr_out.add_list(list_obj2);
    attrs.push_back(attr_out.Finish());
    
    auto attrs_vec = fbb.CreateVector(attrs);
    
    // Extra op
    MNN::ExtraBuilder extra_b(fbb);
    extra_b.add_type(name_op);
    extra_b.add_attr(attrs_vec);
    auto extra = extra_b.Finish();
    
    // Op: Extra with input 0, output 1
    auto input_idx = fbb.CreateVector(std::vector<int32_t>{0});
    auto output_idx = fbb.CreateVector(std::vector<int32_t>{1});
    
    MNN::OpBuilder op_b(fbb);
    op_b.add_type(MNN::OpType_Extra);
    op_b.add_main_type(MNN::OpParameter_Extra);
    op_b.add_main(extra.Union());
    op_b.add_name(name_op);
    op_b.add_inputIndexes(input_idx);
    op_b.add_outputIndexes(output_idx);
    auto op = op_b.Finish();
    
    auto ops_vec = fbb.CreateVector(std::vector<flatbuffers::Offset<MNN::Op>>{op});
    auto tensor_names_vec = fbb.CreateVector(std::vector<flatbuffers::Offset<flatbuffers::String>>{name_t0, name_t1});
    
    // Net
    MNN::NetBuilder net_b(fbb);
    net_b.add_oplists(ops_vec);
    net_b.add_tensorName(tensor_names_vec);
    net_b.add_bizCode(name_net);
    auto net = net_b.Finish();
    
    fbb.Finish(net);
    
    // Save
    {
        std::ofstream out("test_minimal.mnn", std::ios::binary);
        out.write((const char*)fbb.GetBufferPointer(), fbb.GetSize());
        std::cout << "Model saved: test_minimal.mnn (" << fbb.GetSize() << " bytes)\n";
    }
    
    // Now load and run
    auto interp = MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    if (!interp) { std::cerr << "Failed to create interpreter\n"; return 1; }
    std::cout << "Interpreter created\n";
    
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    cfg.numThread = 1;
    
    auto sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "Failed to create session\n"; return 1; }
    std::cout << "Session created\n";
    
    // Get input tensor
    auto in = interp->getSessionInput(sess, "tensor_0");
    if (!in) { std::cerr << "No input\n"; return 1; }
    
    // Resize input to [1, 4, 1, 1] (the shader writes 4 floats)
    interp->resizeTensor(in, {1, 4, 1, 1});
    interp->resizeSession(sess);
    
    // Allocate input (we don't care about input for constant shader)
    
    // Run
    auto err = interp->runSession(sess);
    std::cout << "Run: " << (err == MNN::NO_ERROR ? "OK" : "FAIL") << " (code=" << err << ")\n";
    
    // Get output
    auto out = interp->getSessionOutput(sess, "tensor_1");
    if (!out) { std::cerr << "No output\n"; return 1; }
    
    // Read output
    auto host = new MNN::Tensor(out, MNN::Tensor::CAFFE);
    out->copyToHostTensor(host);
    
    int nonZero = 0;
    for (int i = 0; i < host->elementSize(); ++i) {
        float v = host->host<float>()[i];
        if (v != 0.0f) nonZero++;
        std::cout << " [" << i << "]=" << v;
    }
    std::cout << "\nNon-zero: " << (nonZero > 0 ? "YES" : "NO") << "\n";
    
    delete host;
    interp->releaseSession(sess);
    delete interp;
    
    return 0;
}
