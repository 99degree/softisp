#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <chrono>
#include <functional>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "isp_opset.h"
int main() {
    const int FW=1920, FH=1080;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto rd=[&](const char* p){
        std::ifstream f(p,std::ios::binary|std::ios::ate);
        size_t s=f.tellg(); f.seekg(0);
        std::vector<uint8_t> r(s); f.read((char*)r.data(),s);
        std::vector<int8_t> v(s); memcpy(v.data(),r.data(),s); return v;
    };
    auto spv_demo=rd((base+"isp_opsets/demosaic_noscale.spv").c_str());

    // 1-op model: demosaic_noscale
    flatbuffers::FlatBufferBuilder fbb(1<<20);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode="diag"; net->tensorName={"t0","t1"};
    auto op=std::unique_ptr<MNN::OpT>(new MNN::OpT);
    op->type=MNN::OpType_Extra; op->main.type=MNN::OpParameter_Extra;
    op->main.value=new MNN::ExtraT();
    auto* e=static_cast<MNN::ExtraT*>(op->main.value);
    e->type="isp.demosaic_noscale"; op->inputIndexes={0}; op->outputIndexes={1};
    auto desc=isp::DemosaicNoscale(FW,FH);
    auto ad=[&](const char* k, std::function<void(MNN::AttributeT*)> fn){
        std::unique_ptr<MNN::AttributeT> at(new MNN::AttributeT); at->key=k; fn(at.get()); e->attr.push_back(std::move(at));
    };
    ad("spirv",[&](MNN::AttributeT* a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT8;a->tensor->int8s=spv_demo;});
    ad("output_shape",[&](MNN::AttributeT* a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT32;a->tensor->int32s=desc.output_shape;});
    ad("global_size",[&](MNN::AttributeT* a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT32;a->tensor->int32s=desc.global_size;});
    ad("group_size",[&](MNN::AttributeT* a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT32;a->tensor->int32s={1,1,1};});
    ad("optimized_dispatch",[&](MNN::AttributeT* a){a->b=true;});
    ad("input",[&](MNN::AttributeT* a){a->i=0;a->list.reset(new MNN::ListValueT);a->list->i={0,1};});
    ad("input",[&](MNN::AttributeT* a){a->i=0;a->list.reset(new MNN::ListValueT);a->list->i={1,2};});
    ad("const",[&](MNN::AttributeT* a){a->i=0;a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_FLOAT;a->tensor->float32s=desc.uniforms;a->b=false;});
    net->oplists.push_back(std::move(op));
    auto off=MNN::Net::Pack(fbb,net.get()); fbb.Finish(off);

    dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
    auto reg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll"); if(reg)reg();
    auto interp=MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(),fbb.GetSize());
    MNN::ScheduleConfig cfg; cfg.type=MNN_FORWARD_VULKAN; cfg.numThread=1;
    auto bc=new MNN::BackendConfig; bc->precision=MNN::BackendConfig::Precision_High; cfg.backendConfig=bc;
    auto sess=interp->createSession(cfg);
    auto inp=interp->getSessionInput(sess,"t0");
    int numElems = FW * FH * 4;
    interp->resizeTensor(inp,{1,4,FH,FW});
    interp->resizeSession(sess);
    
    // Fill RGGB with FHD-sized synthetic data (same values as unpack would produce)
    std::vector<float> inData(numElems, 0.0f);
    for(int y=0;y<FH;y++) for(int x=0;x<FW;x++) {
        int idx=y*FW+x;
        // RGGB planes: each position has all 4 Bayer colors from 2×2 block
        inData[idx+0*FW*FH] = 100.0f/1023.0f; // R at (2y,2x)
        inData[idx+1*FW*FH] = 200.0f/1023.0f; // Gr at (2y,2x+1)
        inData[idx+2*FW*FH] = 300.0f/1023.0f; // Gb at (2y+1,2x)
        inData[idx+3*FW*FH] = 400.0f/1023.0f; // B at (2y+1,2x+1)
    }
    halide_type_t float_type = halide_type_of<float>();
    auto hi=MNN::Tensor::create({1,4,FH,FW}, float_type, inData.data(), MNN::Tensor::CAFFE);
    inp->copyFromHostTensor(hi);
    interp->runSession(sess);

    auto out=interp->getSessionOutput(sess,"t1");
    printf("Output: ");
    for(int d=0;d<out->shape().size();d++) printf("%d%s",out->shape()[d],d+1<out->shape().size()?",":"");
    printf(" = %d elems\n",(int)out->elementSize());

    float* od=new float[out->elementSize()];
    auto ho=MNN::Tensor::create(out->shape(),out->getType(),od,MNN::Tensor::CAFFE);
    out->copyToHostTensor(ho);

    int cx=FW/2, cy=FH/2;
    printf("Center (r=%d,c=%d): RGB = (%.6f, %.6f, %.6f)\n",cy,cx,
           od[cy*FW*3+cx*3+0], od[cy*FW*3+cx*3+1], od[cy*FW*3+cx*3+2]);
    printf("Expected: R=0.0977, G=0.2444, B=0.3910\n");
    // Check min/max
    float mn=1e9,mx=-1e9; int nz=0,tot=out->elementSize();
    for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    printf("Range: [%.6f, %.6f]  %d/%d valid\n",mn,mx,nz,tot);

    delete[] od; delete bc; delete interp;
    return 0;
}
