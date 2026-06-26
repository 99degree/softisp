// Test 6-stage at small 8x8 size to check if EE/LDCI identity holds in chains
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
    const int W=8,H=8; // small size for verification
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";

    // Load SPIR-V for all 6 stages
    auto su=rf((base+"shader1_unpack_blc.spv").c_str());
    auto sd=rf((base+"shader2_demosaic_ccm.spv").c_str());  // regular demosaic (with upscale)
    auto sf=rf((base+"shader3_fcs.spv").c_str());
    auto se=rf((base+"shader4_ee.spv").c_str());
    auto sl=rf((base+"shader5_ldci.spv").c_str());
    auto spv_disp=rf((base+"shader6_display_simple.spv").c_str());

    // Build 6-stage pipeline (standard full-res, no noscale needed at 8x8)
    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(W,H),ti8(su));         // Bayer->RGGB 4ch
    pipe.addStage(isp::DemosaicCcm(W,H),ti8(sd));        // RGGB->RGB (upscale)
    pipe.addStage(isp::Fcs(W,H),ti8(sf));
    pipe.addStage(isp::Ee(W,H),ti8(se));
    pipe.addStage(isp::Ldci(W,H),ti8(sl));
    pipe.addStage(isp::Display(W,H),ti8(spv_disp));

    size_t ms;auto md=pipe.build(&ms);
    printf("Model: %zu bytes\n",ms);

    dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
    auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
    auto ip=MNN::Interpreter::createFromBuffer(md,ms);
    MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
    auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
    auto sess=ip->createSession(c);
    auto in=ip->getSessionInput(sess,"tensor_0");
    ip->resizeTensor(in,{1,1,H,W});ip->resizeSession(sess);

    // Synthetic Bayer
    int32_t d[64];
    for(int y=0;y<H;y++) for(int x=0;x<W;x++)
        d[y*W+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    auto hi=MNN::Tensor::create({1,1,H,W},in->getType(),d,MNN::Tensor::CAFFE);
    in->copyFromHostTensor(hi);
    ip->runSession(sess);

    auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);

    // Correct CHW indexing
    int sh=ot->shape()[1]; // channels
    int ow=ot->shape()[3],oh=ot->shape()[2];
    printf("Output shape: [");
    for(int d=0;d<ot->shape().size();d++)printf("%d%c",ot->shape()[d],d+1<ot->shape().size()?',' : ']');
    printf("\n");
    
    int nz=0,tot=ot->elementSize();
    float mn=1e9,mx=-1e9;
    for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    
    int cx=ow/2,cy=oh/2;
    printf("Center(%d,%d): RGB = (%.6f, %.6f, %.6f)\n",cx,cy,
           od[0*ow*oh + cy*ow + cx],
           od[1*ow*oh + cy*ow + cx],
           od[2*ow*oh + cy*ow + cx]);
    printf("Range: [%.6f,%.6f] %d/%d valid\n",mn,mx,nz,tot);
    printf("Expected: R=pow(100/1023,1/2.2)=%.4f G=pow(0.244,1/2.2)=%.4f B=pow(400/1023,1/2.2)=%.4f\n",
           0.3454,0.5315,0.6584);
    bool ok = (mn>0.3f && mx>0.6f && nz==tot);
    printf("Status: %s\n",ok?"PASS - all 6 stages correct!":"FAIL");

    delete[] od;delete bc;delete ip;
    return 0;
}
