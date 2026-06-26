// Read 4K→FHD at multiple positions to find the (1,1,1) pixel
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
    auto sf=rf((base+"shader3_fcs.spv").c_str());
    auto se=rf((base+"shader4_ee.spv").c_str());
    auto sl=rf((base+"shader5_ldci.spv").c_str());
    auto ss=rf((base+"shader6_display_simple.spv").c_str());

    // 6-stage: unpack+demosaic_noscale+fcs+ee+ldci+display
    isp::IspPipelineBuilder pipe(FW,FH,6);
    pipe.addStage(0,isp::UnpackBlc(BW,BH),ti8(su));
    pipe.addStage(1,isp::DemosaicNoscale(FW,FH),ti8(sd));
    pipe.addStage(2,isp::Fcs(FW,FH),ti8(sf));
    pipe.addStage(3,isp::Ee(FW,FH),ti8(se));
    pipe.addStage(4,isp::Ldci(FW,FH),ti8(sl));
    pipe.addStage(5,isp::Display(FW,FH),ti8(ss));
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
    auto ot=ip->getSessionOutput(sess,"tensor_6");
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);

    int plane=FW*FH;
    printf("Output: [1,3,%d,%d] %d elems\n",FH,FW,(int)ot->elementSize());
    
    // Sample many positions
    for(int cy=0;cy<FH;cy+=FH/10){
        for(int cx=0;cx<FW;cx+=FW/10){
            float r=od[0*plane+cy*FW+cx];
            float g=od[1*plane+cy*FW+cx];
            float b=od[2*plane+cy*FW+cx];
            printf("  (%4d,%4d): (%.4f,%.4f,%.4f)%s\n",cx,cy,r,g,b,
                   (r>0.99f||g>0.99f||b>0.99f)?" ⚡1.0!":"");
        }
    }
    
    // Stats
    int n1=0,tot=ot->elementSize();
    float mn=1e9,mx=-1e9;
    for(int i=0;i<tot;i++){if(od[i]>=0.9999f)n1++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    printf("Values >= 1.0: %d/%d pixels (%.1f%%)\n",n1/3,tot/3,100.0f*n1*3/tot);

    delete[] od;delete bc;delete ip;
    return 0;
}
