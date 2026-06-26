// Benchmark: Fully Fused 6-in-1 ISP shader
// Combines unpack→demosaic→fcs→ee→ldci→display in one dispatch.
// Compares against 4-stage fused pipeline.
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

    // Load shaders
    auto su    =ti8(rf((base+"shader1_unpack_blc.spv").c_str()));
    auto sdn   =ti8(rf((base+"isp_opsets/demosaic_noscale.spv").c_str()));
    auto sel   =ti8(rf((base+"shader_ee_ldci_fused.spv").c_str()));
    auto sfd   =ti8(rf((base+"shader_fcs_display_fused.spv").c_str()));
    auto sf6   =ti8(rf((base+"shader_fused_6in1.spv").c_str()));
    auto sud   =ti8(rf((base+"shader_unpack_demosaic.spv").c_str()));

    // Pre-create Bayer input data (INT32, 4K)
    std::vector<int32_t> bayer(BW*BH);
    for(int y=0;y<BH;y++) for(int x=0;x<BW;x++)
        bayer[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;

    printf("══════════════════════════════════════════════════════\n");
    printf("  Fully Fused 6-in-1 ISP Shader Benchmark\n");
    printf("  4K→FHD: %dx%d → %dx%d\n",BW,BH,FW,FH);
    printf("══════════════════════════════════════════════════════\n\n");

    // ── Config 1: 4-stage fused pipeline (baseline) ──
    printf("┌─ Config 1: Fused 4-stage (baseline) ───────────────┐\n");
    double best4=1e9;
    {
        isp::IspPipelineBuilder pipe;
        pipe.addStage(isp::UnpackBlc(BW,BH),su);
        pipe.addStage(isp::DemosaicNoscale(FW,FH),sdn);
        pipe.addStage(isp::FcsDisplayFused(FW,FH,1.0,2.2),sfd);
        pipe.addStage(isp::EeLdciFused(FW,FH,0.5,0.01,0.5,1.0),sel);
        size_t ms;auto md=pipe.build(&ms);

        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);

        auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),bayer.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi);ip->runSession(sess);

        for(int i=0;i<ITERS;i++){
            in->copyFromHostTensor(hi);
            auto t0=std::chrono::high_resolution_clock::now();
            ip->runSession(sess);
            auto t1=std::chrono::high_resolution_clock::now();
            double ms=std::chrono::duration<double,std::milli>(t1-t0).count();
            if(ms<best4)best4=ms;
        }
        // Verify
        auto out=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        auto ht=MNN::Tensor::create({1,3,FH,FW},out->getType(),nullptr,MNN::Tensor::CAFFE);
        out->copyToHostTensor(ht);
        int nz4=0;
        for(int i=0;i<3*FH*FW;i++) if(((float*)ht->host<float>())[i]!=0) nz4++;
        printf("  Best:      %.2f ms (%.1f FPS)\n",best4,1000.0/best4);
        printf("  Non-zero:  %d/%d (%.1f%%)\n",nz4,3*FH*FW,100.0*nz4/(3*FH*FW));
        delete ip;
    }

    // ── Config 2: Fully fused 6-in-1 ──
    printf("┌─ Config 2: Fully Fused 6-in-1 ─────────────────────┐\n");
    double best6=1e9;
    {
        isp::IspPipelineBuilder pipe;
        pipe.addStage(isp::FullyFused6in1(BW,BH,1023.0, 1.0, 0.5,0.01,0.5,1.0,2.2),sf6);
        size_t ms;auto md=pipe.build(&ms);

        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);

        auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),bayer.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi);ip->runSession(sess);

        for(int i=0;i<ITERS;i++){
            in->copyFromHostTensor(hi);
            auto t0=std::chrono::high_resolution_clock::now();
            ip->runSession(sess);
            auto t1=std::chrono::high_resolution_clock::now();
            double ms=std::chrono::duration<double,std::milli>(t1-t0).count();
            if(ms<best6)best6=ms;
        }
        auto out=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        auto ht=MNN::Tensor::create({1,3,FH,FW},out->getType(),nullptr,MNN::Tensor::CAFFE);
        out->copyToHostTensor(ht);
        int nz6=0;
        for(int i=0;i<3*FH*FW;i++) if(((float*)ht->host<float>())[i]!=0) nz6++;
        printf("  Best:      %.2f ms (%.1f FPS)\n",best6,1000.0/best6);
        printf("  Non-zero:  %d/%d (%.1f%%)\n",nz6,3*FH*FW,100.0*nz6/(3*FH*FW));

        // Check first pixel value for correctness
        float* od=(float*)ht->host<float>();
        printf("  Sample RGB: (%.4f, %.4f, %.4f)\n",od[0],od[1*FH*FW],od[2*FH*FW]);
    }

    // ── Config 3: 3-stage (unpack_demosaic+fcs_display+ee_ldci) ──
    printf("┌─ Config 3: 3-stage (unpack+demosaic) fused ──────┐\n");
    double best3=1e9;
    {
        isp::IspPipelineBuilder pipe;
        // Fused unpack+demosaic: needs uniforms {w,h,bw,bh,smax,blc[4],wb[4],ccm[9],pad[4]}
        isp::OpDesc ud_desc = {
            "isp.unpack_demosaic",
            {1,3,FH,FW},
            {FW,FH,1},
            {16,16,1},
            {float(FW),float(FH),float(BW),float(BH),1023.0f,
             0,0,0,0, 1,1,1,1,
             1,0,0, 0,1,0, 0,0,1,
             0,0,0,0}
        };
        pipe.addStage(ud_desc,sud);
        pipe.addStage(isp::FcsDisplayFused(FW,FH,1.0,2.2),sfd);
        pipe.addStage(isp::EeLdciFused(FW,FH,0.5,0.01,0.5,1.0),sel);
        size_t ms;auto md=pipe.build(&ms);

        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);

        auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),bayer.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi);ip->runSession(sess);

        for(int i=0;i<ITERS;i++){
            in->copyFromHostTensor(hi);
            auto t0=std::chrono::high_resolution_clock::now();
            ip->runSession(sess);
            auto t1=std::chrono::high_resolution_clock::now();
            double ms=std::chrono::duration<double,std::milli>(t1-t0).count();
            if(ms<best3)best3=ms;
        }
        auto out=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        auto ht=MNN::Tensor::create({1,3,FH,FW},out->getType(),nullptr,MNN::Tensor::CAFFE);
        out->copyToHostTensor(ht);
        int nz3=0;
        for(int i=0;i<3*FH*FW;i++) if(((float*)ht->host<float>())[i]!=0) nz3++;
        printf("  Best:      %.2f ms (%.1f FPS)\n",best3,1000.0/best3);
        printf("  Non-zero:  %d/%d (%.1f%%)\n",nz3,3*FH*FW,100.0*nz3/(3*FH*FW));
    }

    // ── Comparison ──
    printf("\n══════════════════════════════════════════════════════\n");
    printf("  Comparison:\n");
    printf("    4-stage fused:     %.2f ms (%.1f FPS)\n",best4,1000.0/best4);
    printf("    3-stage (U+D+F+E): %.2f ms (%.1f FPS)\n",best3,1000.0/best3);
    printf("    6-in-1 fused:      %.2f ms (%.1f FPS)\n",best6,1000.0/best6);
    printf("    vs Mesa direct:    23.95 ms\n");
    if(best3<best4) printf("  ✓ 3-stage BEATS 4-stage by %.2fms\n",best4-best3);
    else printf("  △ 4-stage still ahead by %.2fms\n",best3-best4);
    printf("══════════════════════════════════════════════════════\n");
    return 0;
}
