// Isolate the EE shader at FHD to see what it produces
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
static std::vector<uint8_t> rf(const char*p){std::ifstream f(p,std::ios::binary|std::ios::ate);if(!f.good())return{};size_t s=f.tellg();f.seekg(0);std::vector<uint8_t> b(s);f.read((char*)b.data(),s);return b;}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};
int main(){
    const int W=1920,H=1080;
    std::string b="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto spv=rf((b+"shader4_ee.spv").c_str());
    
    // Build single-stage EE op
    flatbuffers::FlatBufferBuilder fbb(1<<20);
    std::unique_ptr<MNN::NetT> net(new MNN::NetT);
    net->bizCode="ee_diag";net->tensorName={"t0","t1"};
    auto op=std::unique_ptr<MNN::OpT>(new MNN::OpT);
    op->type=MNN::OpType_Extra;op->main.type=MNN::OpParameter_Extra;
    op->main.value=new MNN::ExtraT();
    auto* e=static_cast<MNN::ExtraT*>(op->main.value);
    e->type="isp.ee";op->inputIndexes={0};op->outputIndexes={1};
    auto desc=isp::Ee(W,H,0.5f,0.01f);
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
    
    // Feed uniform RGB (same as demosaic would produce)
    ip->resizeTensor(in,{1,3,H,W});ip->resizeSession(ss);
    std::vector<float> id(W*H*3);
    float R=100.0f/1023.0f, G=((200+300)/2)/1023.0f, B=400.0f/1023.0f;
    for(int i=0;i<W*H;i++){id[i+0*W*H]=R;id[i+1*W*H]=G;id[i+2*W*H]=B;}
    auto hi=MNN::Tensor::create({1,3,H,W},halide_type_of<float>(),id.data(),MNN::Tensor::CAFFE);
    in->copyFromHostTensor(hi);
    ip->runSession(ss);
    
    auto ot=ip->getSessionOutput(ss,"t1");
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);
    
    int tot=ot->elementSize();float mn=1e9,mx=-1e9;int nz=0;
    for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    printf("EE diag: [%d/%d valid] [%.6f,%.6f]\n",nz,tot,mn,mx);
    int cx=W/2,cy=H/2;
    printf("Input: R=%.6f G=%.6f B=%.6f\n",R,G,B);
    printf("Center(%d,%d): (%.6f,%.6f,%.6f)\n",cx,cy,
           od[cy*W*3+cx*3+0],od[cy*W*3+cx*3+1],od[cy*W*3+cx*3+2]);
    // Raw first few values
    printf("Raw[0..11]: ");for(int i=0;i<12;i++)printf("%.6f ",od[i]);printf("\n");
    
    delete[] od;delete bc;delete ip;
    return 0;
}
