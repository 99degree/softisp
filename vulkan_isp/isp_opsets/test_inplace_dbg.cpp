// Debug: find the crash in inplace mode
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
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
    auto sd=rf((base+"isp_opsets/demosaic_noscale.spv").c_str());
    auto ss=rf((base+"shader6_display_simple.spv").c_str());

    printf("1. Building model with display inplace (tensor_2→tensor_2)...\n");
    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(BW,BH),ti8(su),false);      // t0→t1
    pipe.addStage(isp::DemosaicNoscale(FW,FH),ti8(sd),false); // t1→t2
    pipe.addStage(isp::Display(FW,FH),ti8(ss), true);         // t2→t2 (inplace)
    size_t ms;auto md=pipe.build(&ms);
    printf("   tensors=%d, model=%zuB, output=%s\n",pipe.tensorCount(),ms,pipe.outputTensorName().c_str());

    printf("2. Loading model...\n");
    dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
    auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
    auto ip=MNN::Interpreter::createFromBuffer(md,ms);
    
    printf("3. Creating session...\n");
    MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
    auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
    auto sess=ip->createSession(c);
    
    printf("4. Resizing input...\n");
    auto in=ip->getSessionInput(sess,"tensor_0");
    ip->resizeTensor(in,{1,1,BH,BW});
    
    printf("5. Resizing session...\n");
    ip->resizeSession(sess);
    
    printf("6. Setting input data...\n");
    std::vector<int32_t> d(BW*BH);
    for(int y=0;y<BH;y++)for(int x=0;x<BW;x++)
        d[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),d.data(),MNN::Tensor::CAFFE);
    in->copyFromHostTensor(hi);
    
    printf("7. Running session...\n");
    ip->runSession(sess);
    
    printf("8. Reading output '%s'...\n",pipe.outputTensorName().c_str());
    auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
    if(!ot){printf("   NULL output!\n");return 1;}
    printf("   shape=[%d,%d,%d,%d] elems=%d\n",
           ot->shape()[0],ot->shape()[1],ot->shape()[2],ot->shape()[3],ot->elementSize());
    
    printf("9. Copy to host...\n");
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);
    
    printf("10. Checking center pixel...\n");
    int plane=FW*FH,cx=FW/2,cy=FH/2;
    float r=od[0*plane+cy*FW+cx],g=od[1*plane+cy*FW+cx],b=od[2*plane+cy*FW+cx];
    printf("    Center: RGB=(%.4f,%.4f,%.4f)\n",r,g,b);
    printf("    OK!\n");
    delete[] od;delete bc;delete ip;
    return 0;
}
