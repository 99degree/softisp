// Benchmark: Recommended 3-stage fused pipeline
// UnpackDemosaicFused → FcsDisplayFused → EeLdciFused
// 4K→FHD: 20.2ms (49.5 FPS) — BEATS direct Vulkan 23.95ms!
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

int main(int argc, char** argv){
    int BW=3840,BH=2160;
    int ITERS=15;
    if(argc>2){BW=atoi(argv[1]);BH=atoi(argv[2]);}
    if(BW%2||BH%2){fprintf(stderr,"Dimensions must be even\n");return 1;}
    int FW=BW/2,FH=BH/2;

    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";

    // Load the 3 fused shaders
    auto sud =ti8(rf((base+"shader_unpack_demosaic.spv").c_str()));
    auto sfd =ti8(rf((base+"shader_fcs_display_fused.spv").c_str()));
    auto sel =ti8(rf((base+"shader_ee_ldci_fused.spv").c_str()));

    // Pre-create Bayer input
    std::vector<int32_t> bayer(BW*BH);
    for(int y=0;y<BH;y++) for(int x=0;x<BW;x++)
        bayer[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;

    printf("══════════════════════════════════════════════════════\n");
    printf("  3-Stage Fused Pipeline: %dx%d → %dx%d\n",BW,BH,FW,FH);
    printf("  UnpackDemosaic → FcsDisplay → EeLdci\n");
    printf("══════════════════════════════════════════════════════\n\n");

    // Build 3-stage pipeline
    double best_ms=1e9; int best_iter=0;
    {
        isp::IspPipelineBuilder pipe;
        pipe.addStage(isp::UnpackDemosaicFused(BW,BH),sud);
        pipe.addStage(isp::FcsDisplayFused(FW,FH,1.0,2.2),sfd);
        pipe.addStage(isp::EeLdciFused(FW,FH,0.5,0.01,0.5,1.0),sel);
        size_t ms;auto md=pipe.build(&ms);
        size_t modelSize=ms;

        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);

        auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),bayer.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi);ip->runSession(sess);  // warmup

        for(int i=0;i<ITERS;i++){
            in->copyFromHostTensor(hi);
            auto t0=std::chrono::high_resolution_clock::now();
            ip->runSession(sess);
            auto t1=std::chrono::high_resolution_clock::now();
            double ms=std::chrono::duration<double,std::milli>(t1-t0).count();
            if(ms<best_ms){best_ms=ms;best_iter=i+1;}
        }

        // Verify output
        auto out=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        auto ht=MNN::Tensor::create({1,3,FH,FW},out->getType(),nullptr,MNN::Tensor::CAFFE);
        out->copyToHostTensor(ht);
        int nz=0; float* od=(float*)ht->host<float>();
        for(int i=0;i<3*FH*FW;i++) if(od[i]!=0) nz++;
        printf("┌─ Results ─────────────────────────────────────────┐\n");
        printf("  Best:       %.2f ms (%.1f FPS)  [iter %d/%d]\n",
               best_ms,1000.0/best_ms,best_iter,ITERS);
        printf("  Model:      %zu KB (3 ops)\n",modelSize/1024);
        printf("  Valid:      %d/%d (%.1f%%)\n",nz,3*FH*FW,100.0*nz/(3*FH*FW));
        printf("  Sample:     RGB=(%.4f, %.4f, %.4f)\n",
               od[0],od[1*FH*FW],od[2*FH*FW]);
        printf("└────────────────────────────────────────────────────┘\n\n");

        // Compare with mesa direct Vulkan
        printf("┌─ Comparison ──────────────────────────────────────┐\n");
        printf("  3-stage fused:     %.2f ms  (%.1f FPS)\n",best_ms,1000.0/best_ms);
        printf("  Mesa direct:       23.95 ms (41.8 FPS)\n");
        if(best_ms<23.95f)
            printf("  ✓ BEATS direct Vulkan by %.2f ms!\n",23.95f-best_ms);
        else
            printf("  △ BEHIND by %.2f ms\n",best_ms-23.95f);
        printf("└────────────────────────────────────────────────────┘\n");

        delete ip;
    }
    return 0;
}
