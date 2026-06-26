// Read back the RGGB output from unpack to verify channel ordering
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
    const int BW=3840,BH=2160,FW=1920,FH=1080;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto su=rf((base+"shader1_unpack_blc.spv").c_str());

    // Single stage: unpack only
    flatbuffers::FlatBufferBuilder fbb(1<<20);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode="unpack";net->tensorName={"t0","t1"};
    auto op=std::unique_ptr<MNN::OpT>(new MNN::OpT);
    op->type=MNN::OpType_Extra;op->main.type=MNN::OpParameter_Extra;
    op->main.value=new MNN::ExtraT();
    auto* e=static_cast<MNN::ExtraT*>(op->main.value);
    e->type="isp.unpack_blc";op->inputIndexes={0};op->outputIndexes={1};
    auto desc=isp::UnpackBlc(BW,BH);
    auto ad=[&](const char*k,std::function<void(MNN::AttributeT*)>fn){
        std::unique_ptr<MNN::AttributeT>at(new MNN::AttributeT);at->key=k;fn(at.get());e->attr.push_back(std::move(at));
    };
    ad("spirv",[&](MNN::AttributeT*a){a->tensor.reset(new MNN::BlobT);a->tensor->dataType=MNN::DataType_DT_INT8;a->tensor->int8s=ti8(su);});
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
    auto sess=ip->createSession(c);
    auto in=ip->getSessionInput(sess,"t0");
    ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);
    std::vector<int32_t> d(BW*BH);
    for(int y=0;y<BH;y++)for(int x=0;x<BW;x++)
        d[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),d.data(),MNN::Tensor::CAFFE);
    in->copyFromHostTensor(hi);
    ip->runSession(sess);

    auto ot=ip->getSessionOutput(sess,"t1");
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);

    int ow=ot->shape()[3],oh=ot->shape()[2];
    int plane=ow*oh;
    int cx=ow/2,cy=oh/2;
    
    printf("Output shape: [");
    for(int d=0;d<ot->shape().size();d++)printf("%d%c",ot->shape()[d],d+1<ot->shape().size()?',' : ']');
    printf("\n");

    // Check what's at position (cx, cy) in each channel
    printf("Center(%d,%d) in each channel:\n",cx,cy);
    for(int ch=0;ch<4;ch++){
        float val = od[ch*plane + cy*ow + cx];
        printf("  ch%d[%d] = %.6f (expected: %s)\n",ch,
               ch*plane+cy*ow+cx, val,
               ch==0?"100/1023=0.0978":ch==1?"200/1023=0.1955":ch==2?"300/1023=0.2933":"400/1023=0.3910");
    }

    // Check adjacent pixels to determine NHWC vs CHW
    printf("\nFirst 12 raw values (layout check):\n");
    for(int i=0;i<12;i++){
        printf("  od[%d]=%.6f",i,od[i]);
        // Determine expected: 
        // If CHW: all ch0 (R) = 100/1023 = 0.0978
        // If NHWC: R(0,0), Gr(0,0), Gb(0,0), B(0,0), R(0,1), G(0,1), ...
        if(i%4==0) printf(" <- R");
        else if(i%4==1) printf(" <- Gr");
        else if(i%4==2) printf(" <- Gb");
        else printf(" <- B");
        printf("\n");
    }

    delete[] od;delete bc;delete ip;
    return 0;
}
