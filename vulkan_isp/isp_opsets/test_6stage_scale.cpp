// Find the break point by testing different sizes
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

struct StageSPV {
    const char* path;
    std::function<isp::OpDesc(int,int)> desc;
} stages[6] = {
    {"shader1_unpack_blc.spv", isp::UnpackBlc},
    {"shader2_demosaic_ccm.spv", isp::DemosaicCcm},
    {"shader3_fcs.spv", [](int w,int h){return isp::Fcs(w,h);}},
    {"shader4_ee.spv", [](int w,int h){return isp::Ee(w,h);}},
    {"shader5_ldci.spv", [](int w,int h){return isp::Ldci(w,h);}},
    {"shader6_display_simple.spv", [](int w,int h){return isp::Display(w,h);}},
};

int test_size(int W, int H) {
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    
    // Load SPIR-V
    std::vector<std::vector<int8_t>> spv(6);
    for(int i=0;i<6;i++){
        auto raw=rf((base+stages[i].path).c_str());
        if(raw.empty()) return -1;
        spv[i].resize(raw.size()); memcpy(spv[i].data(),raw.data(),raw.size());
    }
    
    isp::IspPipelineBuilder pipe;
    for(int i=0;i<6;i++) pipe.addStage(stages[i].desc(W,H),spv[i]);
    
    size_t ms;auto md=pipe.build(&ms);
    
    dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
    auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
    auto ip=MNN::Interpreter::createFromBuffer(md,ms);
    MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
    auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
    auto sess=ip->createSession(c);
    auto in=ip->getSessionInput(sess,"tensor_0");
    ip->resizeTensor(in,{1,1,H,W});ip->resizeSession(sess);
    
    std::vector<int32_t> d(W*H);
    for(int y=0;y<H;y++)for(int x=0;x<W;x++)
        d[y*W+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    auto hi=MNN::Tensor::create({1,1,H,W},in->getType(),d.data(),MNN::Tensor::CAFFE);
    in->copyFromHostTensor(hi);
    ip->runSession(sess);
    
    auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);
    
    int tot=ot->elementSize();
    int ow=ot->shape()[3],oh=ot->shape()[2];
    float mn=1e9,mx=-1e9;int nz=0;
    for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    
    int cx=ow/2,cy=oh/2;
    float r=od[0*ow*oh+cy*ow+cx];
    float g=od[1*ow*oh+cy*ow+cx];
    float b=od[2*ow*oh+cy*ow+cx];
    
    bool ok = (nz==tot && mn>0.3f && mx>0.6f);
    printf("%4dx%-4d | %5d elems | [%.4f,%.4f] | center=(%.4f,%.4f,%.4f) | %s\n",
           W,H,tot,mn,mx,r,g,b,ok?"PASS":"FAIL");
    
    delete[] od;delete bc;delete ip;
    return ok?0:1;
}

int main(){
    printf(" Size    | Elems   | Range         | Center RGB          | Status\n");
    printf("---------+---------+---------------+---------------------+-------\n");
    test_size(8,8);
    test_size(16,16);
    test_size(32,32);
    test_size(64,64);
    test_size(128,128);
    test_size(256,256);
    test_size(512,512);
    test_size(640,480);
    test_size(1024,768);
    test_size(1920,1080);
    test_size(3840,2160);
    return 0;
}
