// Measure zero-copy vs copy readback
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
    const int W=1920,H=1080;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto su=rf((base+"shader1_unpack_blc.spv").c_str());
    auto sd=rf((base+"shader2_demosaic_ccm.spv").c_str());
    auto sf=rf((base+"shader3_fcs.spv").c_str());
    auto se=rf((base+"shader4_ee.spv").c_str());
    auto sl=rf((base+"shader5_ldci.spv").c_str());
    auto ss=rf((base+"shader6_display_simple.spv").c_str());

    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(W,H),ti8(su));
    pipe.addStage(isp::DemosaicCcm(W,H),ti8(sd));
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
    ip->runSession(sess);

    auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
    int tot=ot->elementSize();
    
    // Benchmark readback
    const int ITERS=30;
    double tmin=1e9;
    for(int i=0;i<ITERS;i++){
        float* od=new float[tot];
        auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
        auto t0=std::chrono::high_resolution_clock::now();
        ot->copyToHostTensor(ho);
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count();
        if(dt<tmin) tmin=dt;
        delete[] od;
    }
    printf("Readback (GPU→host): %.3f ms for %.1f MB (FHD RGB)\n", tmin, tot*4.0/1024/1024);
    printf("Previous (copy path): ~10.0 ms (estimate)\n");
    
    // Also measure upload
    tmin=1e9;
    for(int i=0;i<ITERS;i++){
        auto t0=std::chrono::high_resolution_clock::now();
        in->copyFromHostTensor(hi);
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count();
        if(dt<tmin) tmin=dt;
    }
    printf("Upload (host→GPU): %.3f ms for %.1f MB (FHD Bayer)\n", tmin, W*H*4.0/1024/1024);

    delete bc;delete ip;
    return 0;
}
