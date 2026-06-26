// Test: unpack(4K) -> demosaic_noscale(FHD) -> readback RGGB vs RGB
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
    auto sd=rf((base+"isp_opsets/demosaic_noscale.spv").c_str());

    // Build: unpack(4K) -> demosaic_noscale(FHD)
    isp::IspPipelineBuilder pipe(FW,FH,2);
    pipe.addStage(0,isp::UnpackBlc(BW,BH),ti8(su));
    pipe.addStage(1,isp::DemosaicNoscale(FW,FH),ti8(sd));
    size_t ms;auto md=pipe.build(&ms);
    printf("Model: %zu bytes\n",ms);

    dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
    auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
    auto ip=MNN::Interpreter::createFromBuffer(md,ms);
    MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
    auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
    auto sess=ip->createSession(c);
    auto in=ip->getSessionInput(sess,"tensor_0");
    ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);

    std::vector<int32_t> d(BW*BH);
    for(int y=0;y<BH;y++)for(int x=0;x<BW;x++)
        d[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),d.data(),MNN::Tensor::CAFFE);
    in->copyFromHostTensor(hi);
    ip->runSession(sess);

    // Read tensor_2 (demosaic output, 3ch RGB at FHD)
    auto ot=ip->getSessionOutput(sess,"tensor_2");
    if(!ot){printf("tensor_2 NOT FOUND\n");return 1;}
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);

    int ow=ot->shape()[3],oh=ot->shape()[2];
    int plane=ow*oh;
    int cx=ow/2,cy=oh/2;
    
    printf("Output shape: [");
    for(int d=0;d<ot->shape().size();d++)printf("%d%c",ot->shape()[d],d+1<ot->shape().size()?',' : ']');
    printf("\n");

    float r=od[0*plane+cy*ow+cx];
    float g=od[1*plane+cy*ow+cx];
    float b=od[2*plane+cy*ow+cx];
    
    int nz=0,tot=ot->elementSize();
    float mn=1e9,mx=-1e9;
    for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    
    printf("Center(%d,%d): (%.6f,%.6f,%.6f)\n",cx,cy,r,g,b);
    printf("Range: [%.6f,%.6f] %d/%d\n",mn,mx,nz,tot);
    printf("Expected: R=%.4f G=%.4f B=%.4f\n",100.0f/1023.0f,250.0f/1023.0f,400.0f/1023.0f);
    bool ok = (fabsf(r-100.0f/1023.0f)<0.001f && fabsf(g-250.0f/1023.0f)<0.001f && fabsf(b-400.0f/1023.0f)<0.001f);
    printf("Status: %s\n",ok?"PASS":"FAIL");

    delete[] od;delete bc;delete ip;
    return 0;
}
