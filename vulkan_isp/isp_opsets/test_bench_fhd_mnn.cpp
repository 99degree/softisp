// Measure MNN pipeline at FHD (1920x1080) for direct comparison with mesa_test
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
static std::vector<uint8_t> rf(const char*p){
    std::ifstream f(p,std::ios::binary|std::ios::ate);
    if(!f.good())return{};size_t s=f.tellg();f.seekg(0);
    std::vector<uint8_t> b(s);f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};

int main(){
    const int W=1920,H=1080;  // FHD
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto su=rf((base+"shader1_unpack_blc.spv").c_str());
    auto sd=rf((base+"shader2_demosaic_ccm.spv").c_str());
    auto sf=rf((base+"shader3_fcs.spv").c_str());
    auto se=rf((base+"shader4_ee.spv").c_str());
    auto sl=rf((base+"shader5_ldci.spv").c_str());
    auto ss=rf((base+"shader6_display_simple.spv").c_str());

    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(W,H),ti8(su));       // reads Bayer at W×H (input is w/h)
    pipe.addStage(isp::DemosaicCcm(W,H),ti8(sd));      // RGGB→RGB at W×H
    pipe.addStage(isp::Fcs(W,H),ti8(sf));
    pipe.addStage(isp::Ee(W,H),ti8(se));
    pipe.addStage(isp::Ldci(W,H),ti8(sl));
    pipe.addStage(isp::Display(W,H),ti8(ss));
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
    
    // Warmup
    ip->runSession(sess);
    
    // Benchmark
    const int ITERS=30;
    double tmin=1e9;
    for(int i=0;i<ITERS;i++){
        in->copyFromHostTensor(hi);
        auto t0=std::chrono::high_resolution_clock::now();
        ip->runSession(sess);
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count();
        if(dt<tmin) tmin=dt;
    }
    
    printf("MNN VulkanFuse 6-stage ISP @ FHD (%dx%d): %.2f ms (%.1f FPS)\n",
           W, H, tmin, 1000.0/tmin);
    printf("Mesa direct Vulkan (7-stage, similar): 23.95 ms (41.8 FPS)\n");
    printf("Ratio: %.1f× slower\n", tmin/23.95);
    printf("Note: MNN uses 6 stages, mesa uses 7; UnpackBlc vs BLC/WB differ\n");
    
    delete bc;delete ip;
    return 0;
}
