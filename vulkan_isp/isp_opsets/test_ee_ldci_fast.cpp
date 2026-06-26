// Compare standard vs fast (shared-memory) EE and LDCI shaders
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
    std::ifstream f(p,std::ios::binary|std::ios::ate);if(!f.good())return{};
    size_t s=f.tellg();f.seekg(0);std::vector<uint8_t>b(s);f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};
int main(){
    int W=1920,H=1080;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    
    // Test 4 configurations
    struct Cfg{
        const char*name;
        std::vector<int8_t> ee,ldci;
    };
    Cfg cfgs[]={
        {"std_ee+std_ldci",
         ti8(rf((base+"shader4_ee.spv").c_str())), ti8(rf((base+"shader5_ldci.spv").c_str()))},
        {"fast_ee+std_ldci",
         ti8(rf((base+"shader4_ee_fast.spv").c_str())), ti8(rf((base+"shader5_ldci.spv").c_str()))},
        {"std_ee+fast_ldci",
         ti8(rf((base+"shader4_ee.spv").c_str())), ti8(rf((base+"shader5_ldci_fast.spv").c_str()))},
        {"fast_ee+fast_ldci",
         ti8(rf((base+"shader4_ee_fast.spv").c_str())), ti8(rf((base+"shader5_ldci_fast.spv").c_str()))},
    };
    
    // Pre-generate random-ish float input
    std::vector<float> d(W*H*3);
    for(int i=0;i<W*H*3;i++) d[i]=(rand()%1000)/1000.0f;
    auto hi=MNN::Tensor::create({1,3,H,W},halide_type_of<float>(),d.data(),MNN::Tensor::CAFFE);
    
    for(auto& cfg:cfgs){
        if(cfg.ee.empty()||cfg.ldci.empty()){printf("%-20s: SKIP (missing)\n",cfg.name);continue;}
        isp::IspPipelineBuilder pipe;
        pipe.addStage(isp::Ee(W,H),cfg.ee);
        pipe.addStage(isp::Ldci(W,H),cfg.ldci);
        size_t ms;auto md=pipe.build(&ms);
        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,3,H,W});ip->resizeSession(sess);
        in->copyFromHostTensor(hi);
        
        // Warmup + timing
        ip->runSession(sess);
        const int N=15;
        auto t0=std::chrono::high_resolution_clock::now();
        for(int i=0;i<N;i++){in->copyFromHostTensor(hi);ip->runSession(sess);}
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count()/N;
        
        auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        float* od=new float[ot->elementSize()];
        auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
        ot->copyToHostTensor(ho);
        int tot=ot->elementSize();
        float mn=1e9,mx=-1e9;int nz=0;
        for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
        bool ok=(nz==tot);
        printf("%-20s: %6.2fms (%-5.1f FPS)  %s  range=[%.4f,%.4f]\n",
               cfg.name,dt,1000.0/dt,ok?"PASS":"FAIL",mn,mx);
        delete[] od;delete bc;delete ip;
    }
    return 0;
}
