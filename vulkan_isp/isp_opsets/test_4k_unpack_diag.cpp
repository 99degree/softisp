#include <functional>
// Quick unpack-only diag at 4K
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

int main() {
    const int BW = 3840, BH = 2160;
    std::string base = "/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto spv = [&](const char* p){ 
        std::ifstream f(p, std::ios::binary|std::ios::ate);
        size_t sz = f.tellg(); f.seekg(0);
        std::vector<uint8_t> r(sz); f.read((char*)r.data(), sz);
        std::vector<int8_t> v(sz); memcpy(v.data(), r.data(), sz);
        return v;
    }((base + "shader1_unpack_blc.spv").c_str());

    flatbuffers::FlatBufferBuilder fbb(1<<20);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode = "diag"; net->tensorName = {"t0","t1"};
    auto op = std::unique_ptr<MNN::OpT>(new MNN::OpT);
    op->type = MNN::OpType_Extra; op->main.type = MNN::OpParameter_Extra;
    op->main.value = new MNN::ExtraT();
    auto* e = static_cast<MNN::ExtraT*>(op->main.value);
    e->type = "isp.unpack_blc"; op->inputIndexes={0}; op->outputIndexes={1};
    auto a = [&](const char* k, std::function<void(MNN::AttributeT*)> fn){ std::unique_ptr<MNN::AttributeT> at(new MNN::AttributeT); at->key=k; fn(at.get()); e->attr.push_back(std::move(at)); };
    auto desc = isp::UnpackBlc(BW, BH);
    a("spirv", [&](MNN::AttributeT* at){ at->tensor.reset(new MNN::BlobT); at->tensor->dataType=MNN::DataType_DT_INT8; at->tensor->int8s=spv; });
    a("output_shape",[&](MNN::AttributeT* at){ at->tensor.reset(new MNN::BlobT); at->tensor->dataType=MNN::DataType_DT_INT32; at->tensor->int32s=desc.output_shape; });
    a("global_size",[&](MNN::AttributeT* at){ at->tensor.reset(new MNN::BlobT); at->tensor->dataType=MNN::DataType_DT_INT32; at->tensor->int32s=desc.global_size; });
    a("group_size",[&](MNN::AttributeT* at){ at->tensor.reset(new MNN::BlobT); at->tensor->dataType=MNN::DataType_DT_INT32; at->tensor->int32s={1,1,1}; });
    a("optimized_dispatch",[&](MNN::AttributeT* at){ at->b=true; });
    a("input",[&](MNN::AttributeT* at){ at->i=0; at->list.reset(new MNN::ListValueT); at->list->i={0,1}; });
    a("input",[&](MNN::AttributeT* at){ at->i=0; at->list.reset(new MNN::ListValueT); at->list->i={1,2}; });
    a("const",[&](MNN::AttributeT* at){ at->i=0; at->tensor.reset(new MNN::BlobT); at->tensor->dataType=MNN::DataType_DT_FLOAT; at->tensor->float32s=desc.uniforms; at->b=false; });
    net->oplists.push_back(std::move(op));
    auto off = MNN::Net::Pack(fbb, net.get()); fbb.Finish(off);

    dlopen("libMNN_Vulkan.so", RTLD_NOW|RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll"); if(reg)reg();
    auto interp=MNN::Interpreter::createFromBuffer(fbb.GetBufferPointer(), fbb.GetSize());
    MNN::ScheduleConfig cfg; cfg.type=MNN_FORWARD_VULKAN; cfg.numThread=1;
    auto bc=new MNN::BackendConfig; bc->precision=MNN::BackendConfig::Precision_High; cfg.backendConfig=bc;
    auto sess=interp->createSession(cfg);
    auto inp=interp->getSessionInput(sess,"t0");
    interp->resizeTensor(inp,{1,1,BH,BW}); interp->resizeSession(sess);
    std::vector<int32_t> d(BW*BH);
    for(int y=0;y<BH;y++) for(int x=0;x<BW;x++)
        d[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    auto hi=MNN::Tensor::create({1,1,BH,BW},inp->getType(),d.data(),MNN::Tensor::CAFFE);
    inp->copyFromHostTensor(hi);
    interp->runSession(sess);

    auto out=interp->getSessionOutput(sess,"t1");
    printf("Unpack output: [");
    for(int dd=0;dd<out->shape().size();dd++) printf("%d%s",out->shape()[dd],dd+1<out->shape().size()?",":"");
    printf("] = %d elems\n",(int)out->elementSize());

    float* od=new float[out->elementSize()];
    auto ho=MNN::Tensor::create(out->shape(),out->getType(),od,MNN::Tensor::CAFFE);
    out->copyToHostTensor(ho);

    // Sample: center pixel, corner, etc
    int Hhalf=BH/2, Whalf=BW/2;
    int c=Whalf/2, r=Hhalf/2; // center of RGGB planes
    printf("Center pixel (r=%d,c=%d) in RGGB output:\n", r, c);
    for(int ch=0;ch<4;ch++) {
        int idx = r*Whalf + c + ch*Whalf*Hhalf;
        printf("  ch%d[%d] = %f (expected %s)\n", ch, idx, od[idx],
               ch==0 ? "100/1023=0.098" : ch==1 ? "200/1023=0.196" : ch==2 ? "300/1023=0.293" : "400/1023=0.391");
    }
    // Also sample corner
    printf("Corner (0,0):\n");
    for(int ch=0;ch<4;ch++) printf("  ch%d[%d] = %f\n", ch, 0+ch*Whalf*Hhalf, od[0+ch*Whalf*Hhalf]);

    delete[] od; delete bc; delete interp;
    return 0;
}
