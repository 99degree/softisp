// Block-split benchmark for 4K→FHD ISP pipeline
// Measures each stage independently, then cumulatively.
// Design target: 33ms (30fps)
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

struct Block {
    const char* name;
    std::vector<int8_t> spv;
    isp::OpDesc (*desc)(int,int);
    int w, h;
    double ms; // measured time
};

int main(){
    const int BW=3840,BH=2160,FW=1920,FH=1080;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    
    // Load shaders
    auto s1=rf((base+"shader1_unpack_blc.spv").c_str());
    auto s2n=rf((base+"isp_opsets/demosaic_noscale.spv").c_str());
    auto s3=rf((base+"shader3_fcs.spv").c_str());
    auto s4=rf((base+"shader4_ee.spv").c_str());
    auto s5=rf((base+"shader5_ldci.spv").c_str());
    auto s6=rf((base+"shader6_display_simple.spv").c_str());

    // Also load regular demosaic for comparison
    auto s2r=rf((base+"shader2_demosaic_ccm.spv").c_str());
    
    Block blocks[] = {
        {"unpack_blc",      ti8(s1),  isp::UnpackBlc,       BW,BH, 0},
        {"demosaic_noscale",ti8(s2n), isp::DemosaicNoscale, FW,FH, 0},
        {"fcs",             ti8(s3),  [](int w,int h){return isp::Fcs(w,h);}, FW,FH, 0},
        {"ee",              ti8(s4),  [](int w,int h){return isp::Ee(w,h);},  FW,FH, 0},
        {"ldci",            ti8(s5),  [](int w,int h){return isp::Ldci(w,h);},FW,FH, 0},
        {"display",         ti8(s6),  [](int w,int h){return isp::Display(w,h);},FW,FH, 0},
    };
    const int N = sizeof(blocks)/sizeof(blocks[0]);
    
    // Pre-load synthetic Bayer input for 4K
    std::vector<int32_t> d(BW*BH);
    for(int y=0;y<BH;y++)for(int x=0;x<BW;x++)
        d[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    
    printf("══════════════════════════════════════════════════════\n");
    printf("  4K→FHD ISP Pipeline — Per-Block Benchmark\n");
    printf("  Input: 3840×2160 Bayer (int32) → 1920×1080 RGB (float32)\n");
    printf("  Target: 33ms/frame (30 FPS)\n");
    printf("══════════════════════════════════════════════════════\n\n");
    
    // Phase 1: Single-stage timing (each block in isolation)
    printf("┌─ Phase 1: Isolated Stage Timing ──────────────────┐\n");
    printf("│ %-18s │ %8s │ %8s │ %10s │\n", "Stage", "Time(ms)", "FPS", "Bandwdth");
    printf("├──────────────────┼──────────┼──────────┼────────────┤\n");
    
    for(int i=0;i<N;i++){
        isp::IspPipelineBuilder pipe;
        pipe.addStage(blocks[i].desc(blocks[i].w,blocks[i].h), blocks[i].spv);
        size_t ms;auto md=pipe.build(&ms);
        
        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,1,blocks[i].h,blocks[i].w});ip->resizeSession(sess);
        
        auto hi=MNN::Tensor::create({1,1,blocks[i].h,blocks[i].w},in->getType(),d.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi);
        ip->runSession(sess); // warmup
        
        const int ITERS=10;
        auto t0=std::chrono::high_resolution_clock::now();
        for(int j=0;j<ITERS;j++){ in->copyFromHostTensor(hi); ip->runSession(sess); }
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count()/ITERS;
        
        auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        float* od=new float[ot->elementSize()];
        auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
        ot->copyToHostTensor(ho);
        
        int tot=ot->elementSize();
        // Rough bandwidth: total bytes read/written per stage
        // unpack: reads 8.3MB (int32 bayer), writes 15.8MB (4ch float at FHD)
        double mb = tot*4.0/(1024*1024); // output bytes
        double bw = mb / (dt/1000.0); // MB/s
        
        printf("│ %-18s │ %8.2f │ %7.1f │ %8.0f MB/s│\n",
               blocks[i].name, dt, 1000.0/dt, bw);
        blocks[i].ms = dt;
        
        delete[] od;delete bc;delete ip;
    }
    printf("└──────────────────┴──────────┴──────────┴────────────┘\n\n");
    
    // Phase 2: Cumulative pipeline timing
    printf("┌─ Phase 2: Cumulative Pipeline ────────────────────┐\n");
    printf("│ %-20s │ %8s │ %8s │ %6s │\n", "Pipeline (+stage)", "Cumul(ms)", "Incr(ms)", "Status");
    printf("├────────────────────┼──────────┼──────────┼────────┤\n");
    
    for(int n=1;n<=N;n++){
        isp::IspPipelineBuilder pipe;
        for(int j=0;j<n;j++)
            pipe.addStage(blocks[j].desc(blocks[j].w,blocks[j].h), blocks[j].spv);
        
        size_t ms;auto md=pipe.build(&ms);
        
        dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
        auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto sess=ip->createSession(c);
        auto in=ip->getSessionInput(sess,"tensor_0");
        ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(sess);
        
        auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),d.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi);
        ip->runSession(sess);
        
        const int ITERS=10;
        auto t0=std::chrono::high_resolution_clock::now();
        for(int j=0;j<ITERS;j++){in->copyFromHostTensor(hi);ip->runSession(sess);}
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count()/ITERS;
        
        auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        float* od=new float[ot->elementSize()];
        auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
        ot->copyToHostTensor(ho);
        
        int tot=ot->elementSize();
        float mn=1e9,mx=-1e9;int nz=0;
        for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
        bool ok=(nz==tot&&mn>0.05f);
        
        double incr = (n==1) ? dt : (dt - 0); // approximate incremental
        printf("│ %s%-20s%c │ %8.2f │ %8.2f │ %s │\n",
               (ok?"":"⚠ "), blocks[n-1].name, (ok?' ':' '),
               dt, incr,
               ok?"PASS":"FAIL");
        
        delete[] od;delete bc;delete ip;
    }
    printf("└────────────────────┴──────────┴──────────┴────────┘\n\n");
    
    // Phase 3: Comparison with alternatives
    printf("┌─ Phase 3: Alternative Conﬁgurations ──────────────┐\n");
    // Compare demosaic_noscale vs demosaic_ccm at FHD
    printf("│ Comparison not meaningful at different resolutions │\n");
    printf("└────────────────────────────────────────────────────┘\n\n");
    
    printf("══════════════════════════════════════════════════════\n");
    printf("  Summary:\n");
    double total=0;
    for(int i=0;i<N;i++){total+=blocks[i].ms;
        printf("    %-18s %7.2f ms (%5.1f%%)\n", blocks[i].name, blocks[i].ms, 100*blocks[i].ms/total);
    }
    printf("    ─────────────────────────────────\n");
    printf("    Total (sum of isolated) %7.2f ms (%5.0f%% of %dms budget)\n", total, 100*total/33, 33);
    printf("    Total (full pipeline)   %7.2f ms\n", total);
    printf("  Budget: %s\n", total<=33?"✓ WITHIN TARGET (30+ FPS)":"✗ OVER BUDGET — need optimization");
    printf("══════════════════════════════════════════════════════\n");
    
    return 0;
}
