// Check actual output buffer layout: planar CHW vs interleaved NHWC
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <functional>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "isp_opset.h"
static std::vector<uint8_t> rf(const char*p){
    std::ifstream f(p,std::ios::binary|std::ios::ate);
    if(!f.good())return{};size_t s=f.tellg();f.seekg(0);
    std::vector<uint8_t> b(s);f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};
int main(){
    const int W=8,H=8; // small for clarity
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto spv=rf((base+"shader6_display_simple.spv").c_str());

    // Build: display only (passthrough with gamma=1.0 so output = input)
    flatbuffers::FlatBufferBuilder fbb(1<<20);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode="layout_diag";net->tensorName={"t0","t1"};
    auto op=std::unique_ptr<MNN::OpT>(new MNN::OpT);
    op->type=MNN::OpType_Extra;op->main.type=MNN::OpParameter_Extra;
    op->main.value=new MNN::ExtraT();
    auto* e=static_cast<MNN::ExtraT*>(op->main.value);
    e->type="isp.display";op->inputIndexes={0};op->outputIndexes={1};

    // Use gamma=1.0 (linear) so values pass through unchanged
    isp::OpDesc desc = isp::Display(W, H, 1.0f);
    auto ad=[&](const char*k,std::function<void(MNN::AttributeT*)>fn){
        std::unique_ptr<MNN::AttributeT>at(new MNN::AttributeT);at->key=k;fn(at.get());e->attr.push_back(std::move(at));
    };
    ad("spirv",[&](MNN::AttributeT*a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT8;a->tensor->int8s=ti8(spv);});
    ad("output_shape",[&](MNN::AttributeT*a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT32;a->tensor->int32s=desc.output_shape;});
    ad("global_size",[&](MNN::AttributeT*a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT32;a->tensor->int32s=desc.global_size;});
    ad("group_size",[&](MNN::AttributeT*a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT32;a->tensor->int32s={1,1,1};});
    ad("optimized_dispatch",[&](MNN::AttributeT*a){a->b=true;});
    ad("input",[&](MNN::AttributeT*a){a->i=0;a->list.reset(new MNN::ListValueT);a->list->i={0,1};});
    ad("input",[&](MNN::AttributeT*a){a->i=0;a->list.reset(new MNN::ListValueT);a->list->i={1,2};});
    ad("const",[&](MNN::AttributeT*a){a->i=0;a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_FLOAT;a->tensor->float32s=desc.uniforms;a->b=false;});
    net->oplists.push_back(std::move(op));
    auto off=MNN::Net::Pack(fbb,net.get());fbb.Finish(off);

    dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
    auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
    auto ip=MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(),fbb.GetSize());
    MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
    auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
    auto ss=ip->createSession(c);
    auto in=ip->getSessionInput(ss,"t0");
    ip->resizeTensor(in,{1,3,H,W});ip->resizeSession(ss);

    // Feed distinct values per channel per pixel
    // R at (x,y) = x*0.01 + y*0.001 (distinct per pixel)
    // G = R + 0.3, B = R + 0.6
    std::vector<float> id(W*H*3);
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        float val = x*0.01f + y*0.001f;
        id[y*W+x + 0*W*H] = val;           // R channel (CHW)
        id[y*W+x + 1*W*H] = val + 0.3f;    // G channel
        id[y*W+x + 2*W*H] = val + 0.6f;    // B channel
    }
    auto hi=MNN::Tensor::create({1,3,H,W},halide_type_of<float>(),id.data(),MNN::Tensor::CAFFE);
    in->copyFromHostTensor(hi);
    ip->runSession(ss);

    auto ot=ip->getSessionOutput(ss,"t1");

    // Read back as NCHW host tensor
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);

    printf("Shape: [");
    for(int d=0;d<ot->shape().size();d++)printf("%d%c",ot->shape()[d],d+1<ot->shape().size()?',' : ']');
    printf("\n");

    // Print pixel (0,0) three ways to determine layout
    printf("\nPixel (0,0):\n");
    printf("  CHW indices:  ch0[%d]=%.4f  ch1[%d]=%.4f  ch2[%d]=%.4f\n",
           0*W*H+0, od[0*W*H+0], 1*W*H+0, od[1*W*H+0], 2*W*H+0, od[2*W*H+0]);
    printf("  NHWC index:   [0]=%.4f [1]=%.4f [2]=%.4f\n", od[0], od[1], od[2]);
    
    // Print pixel (1,0):
    printf("\nPixel (1,0):\n");
    printf("  CHW:  ch0[%d]=%.4f  ch1[%d]=%.4f  ch2[%d]=%.4f\n",
           0*W*H+1, od[0*W*H+1], 1*W*H+1, od[1*W*H+1], 2*W*H+1, od[2*W*H+1]);
    printf("  NHWC: [%d]=%.4f [%d]=%.4f [%d]=%.4f\n",
           3, od[3], 4, od[4], 5, od[5]);

    // Print first 12 raw values
    printf("\nRaw[0..11]: ");
    for(int i=0;i<12;i++) printf("%.4f ",od[i]);
    printf("\n");

    // Analysis
    printf("\nExpected input pixel (0,0) = (%.3f, %.3f, %.3f)\n", 
           0.0f, 0.3f, 0.6f);
    printf("Expected input pixel (1,0) = (%.3f, %.3f, %.3f)\n",
           0.01f, 0.31f, 0.61f);

    printf("\nLayout determination:\n");
    bool is_chw = (fabsf(od[0*W*H+0]-0.0f)<0.001f &&  // R(0,0)
                   fabsf(od[1*W*H+0]-0.3f)<0.001f &&  // G(0,0)
                   fabsf(od[2*W*H+0]-0.6f)<0.001f);   // B(0,0)
    bool is_nhwc = (fabsf(od[0]-0.0f)<0.001f &&  // R(0,0)
                    fabsf(od[1]-0.3f)<0.001f &&  // G(0,0)
                    fabsf(od[2]-0.6f)<0.001f);   // B(0,0)
    printf("  CHW (planar):  %s\n", is_chw ? "YES" : "NO");
    printf("  NHWC (interleaved): %s\n", is_nhwc ? "YES" : "NO");

    delete[] od; delete bc; delete ip;
    return 0;
}
