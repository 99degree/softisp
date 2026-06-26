// test_4k_to_fhd.cpp — 4K Bayer → FHD RGB via 6 ISP opsets
// Verifies full pipeline at 4K resolution with correct CHW output indexing.
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

    printf("4K Bayer -> FHD RGB pipeline (%d MP, %d stages)\n", BW*BH/1000000, 6);

    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(BW,BH),ti8(su));
    pipe.addStage(isp::DemosaicNoscale(FW,FH),ti8(sd));
    pipe.addStage(isp::Fcs(FW,FH),ti8(sf));
    pipe.addStage(isp::Ee(FW,FH),ti8(se));
    pipe.addStage(isp::Ldci(FW,FH),ti8(sl));
    pipe.addStage(isp::Display(FW,FH),ti8(ss));
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

    // Warmup + benchmark
    ip->runSession(sess);
    auto rs=std::chrono::high_resolution_clock::now();
    for(int i=0;i<10;i++) ip->runSession(sess);
    auto re=std::chrono::high_resolution_clock::now();
    double ms_f=std::chrono::duration<double,std::milli>(re-rs).count()/10.0;

    auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);

    // Stats using correct CHW indexing
    int plane=FW*FH;
    int nz=0,tot=ot->elementSize();
    float mn=1e9,mx=-1e9;
    for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    int cx=FW/2,cy=FH/2;
    float r=od[0*plane+cy*FW+cx];
    float g=od[1*plane+cy*FW+cx];
    float b=od[2*plane+cy*FW+cx];
    
    printf("Results: %.2f ms/frame (%.1f FPS)\n",ms_f,1000.0/ms_f);
    printf("Output:  %d/%d valid (%.1f%%) [%.4f,%.4f]\n",nz,tot,100.0*nz/tot,mn,mx);
    printf("Center:  (%d,%d) RGB = (%.4f, %.4f, %.4f)\n",cx,cy,r,g,b);
    printf("Status:  %s\n",(nz==tot&&mn>0.3f)?"PASS ✅":"FAIL ❌");
    
    delete[] od;delete bc;delete ip;
    return 0;
}
