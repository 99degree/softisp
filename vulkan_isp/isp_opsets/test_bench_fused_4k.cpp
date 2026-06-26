// Benchmark 4K→FHD pipeline with chain fusion optimization
// Compare: standard (6 stages) vs fused (4 stages: unpack+demosaic+fcs_display+ee_ldci)
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
    std::ifstream f(p,std::ios::binary|std::ios::ate);if(!f.good()){fprintf(stderr,"FAIL: %s\n",p);exit(1);}
    size_t s=f.tellg();f.seekg(0);std::vector<uint8_t>b(s);f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};

int main(){
    int BW=3840,BH=2160,FW=1920,FH=1080;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    int ITERS=15;
    
    struct Cfg{
        const char* name;
        void (*build)(isp::IspPipelineBuilder&);
        std::vector<int8_t> spv;
    };
    
    // Load all shaders
    auto su    =ti8(rf((base+"shader1_unpack_blc.spv").c_str()));
    auto sdn   =ti8(rf((base+"isp_opsets/demosaic_noscale.spv").c_str()));
    auto sf    =ti8(rf((base+"shader3_fcs.spv").c_str()));
    auto se    =ti8(rf((base+"shader4_ee.spv").c_str()));
    auto sl    =ti8(rf((base+"shader5_ldci.spv").c_str()));
    auto ss    =ti8(rf((base+"shader6_display_simple.spv").c_str()));
    auto sel   =ti8(rf((base+"shader_ee_ldci_fused.spv").c_str()));
    auto sfd   =ti8(rf((base+"shader_fcs_display_fused.spv").c_str()));
    
    // Pre-create input data
    std::vector<int32_t> bayer(BW*BH);
    for(int y=0;y<BH;y++) for(int x=0;x<BW;x++)
        bayer[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    
    printf("══════════════════════════════════════════════════════\n");
    printf("  4K→FHD ISP Pipeline — Chain Fusion Benchmark\n");
    printf("══════════════════════════════════════════════════════\n\n");
    
    // Config 1: Standard 6-stage
    printf("┌─ Config 1: Standard 6-stage ───────────────────────┐\n");
    {
        isp::IspPipelineBuilder pipe;
        pipe.addStage(isp::UnpackBlc(BW,BH),su);
        pipe.addStage(isp::DemosaicNoscale(FW,FH),sdn);
        pipe.addStage(isp::Fcs(FW,FH),sf);
        pipe.addStage(isp::Ee(FW,FH),se);
        pipe.addStage(isp::Ldci(FW,FH),sl);
        pipe.addStage(isp::Display(FW,FH),ss);
        size_t ms;auto md=pipe.build(&ms);  // Uses fusion if applicable
        
        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);
        
        // Count ops
        
        auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),bayer.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi); ip->runSession(sess);  // warmup
        
        double best=1e9;
        for(int i=0;i<ITERS;i++){
            in->copyFromHostTensor(hi);
            auto t0=std::chrono::high_resolution_clock::now();
            ip->runSession(sess);
            auto t1=std::chrono::high_resolution_clock::now();
            double dt=std::chrono::duration<double,std::milli>(t1-t0).count();
            if(dt<best) best=dt;
        }
        
        auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        float* od=new float[ot->elementSize()];
        auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
        ot->copyToHostTensor(ho);
        
        int tot=ot->elementSize();float mn=1e9,mx=-1e9;int nz=0;
        for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
        int cx=FW/2,cy=FH/2;
        float r=od[0*FW*FH+cy*FW+cx],g=od[1*FW*FH+cy*FW+cx],b=od[2*FW*FH+cy*FW+cx];
        bool ok=(nz==tot&&mn>0.3f);
        printf("│ Time: %6.2fms  (%5.1f FPS)\n", best, 1000.0/best);
        printf("│ RGB=(%.4f,%.4f,%.4f) %s\n",r,g,b,ok?"PASS":"FAIL");
        delete[] od;delete bc;delete ip;
    }
    
    // Config 2: Fused (FCS+Display, EE+LDCI) — needs fused SPIR-V
    printf("├─ Config 2: Fused 4-stage ──────────────────────────┤\n");
    {
        isp::IspPipelineBuilder pipe;
        pipe.addStage(isp::UnpackBlc(BW,BH),su);
        pipe.addStage(isp::DemosaicNoscale(FW,FH),sdn);
        pipe.addStage(isp::FcsDisplayFused(FW,FH,1.0,2.2),sfd);
        pipe.addStage(isp::EeLdciFused(FW,FH,0.5,0.01,0.5,1.0),sel);
        size_t ms;auto md=pipe.build(&ms);  // fusion already applied
        
        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);
        
        
        auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),bayer.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi); ip->runSession(sess);
        
        double best=1e9;
        for(int i=0;i<ITERS;i++){
            in->copyFromHostTensor(hi);
            auto t0=std::chrono::high_resolution_clock::now();
            ip->runSession(sess);
            auto t1=std::chrono::high_resolution_clock::now();
            double dt=std::chrono::duration<double,std::milli>(t1-t0).count();
            if(dt<best) best=dt;
        }
        
        auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        float* od=new float[ot->elementSize()];
        auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
        ot->copyToHostTensor(ho);
        
        int tot=ot->elementSize();float mn=1e9,mx=-1e9;int nz=0;
        for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
        int cx=FW/2,cy=FH/2;
        float r=od[0*FW*FH+cy*FW+cx],g=od[1*FW*FH+cy*FW+cx],b=od[2*FW*FH+cy*FW+cx];
        bool ok=(nz==tot&&mn>0.3f);
        printf("│ Time: %6.2fms  (%5.1f FPS)\n", best, 1000.0/best);
        printf("│ RGB=(%.4f,%.4f,%.4f) %s\n",r,g,b,ok?"PASS":"FAIL");
        delete[] od;delete bc;delete ip;
    }
    
    printf("└────────────────────────────────────────────────────┘\n");
    printf("\n  Target: 33ms (30 FPS)\n");
    return 0;
}
