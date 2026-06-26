// Trace actual dispatches: count barriers, end/begin of command buffers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
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
    
    printf("=== Pipeline structure ===\n");
    printf("Model: %zu bytes, %d tensors\n", ms, pipe.tensorCount());
    
    auto sess=ip->createSession(c);
    auto in=ip->getSessionInput(sess,"tensor_0");
    ip->resizeTensor(in,{1,1,H,W});
    
    // Measure resize time (op compilation)
    auto t0=std::chrono::high_resolution_clock::now();
    ip->resizeSession(sess);
    auto t1=std::chrono::high_resolution_clock::now();
    double resize_ms=std::chrono::duration<double,std::milli>(t1-t0).count();
    printf("Resize session: %.2f ms\n", resize_ms);
    
    // Measure single run
    std::vector<int32_t> d(W*H);
    for(int y=0;y<H;y++)for(int x=0;x<W;x++)
        d[y*W+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    auto hi=MNN::Tensor::create({1,1,H,W},in->getType(),d.data(),MNN::Tensor::CAFFE);
    
    // Warmup (first run includes shader compilation & auto-tuning)
    in->copyFromHostTensor(hi);
    auto t2=std::chrono::high_resolution_clock::now();
    ip->runSession(sess);
    auto t3=std::chrono::high_resolution_clock::now();
    double first_ms=std::chrono::duration<double,std::milli>(t3-t2).count();
    printf("First runSession: %.2f ms (includes auto-tuning)\n", first_ms);
    
    // Subsequent runs (should be fast)
    const int N=50;
    in->copyFromHostTensor(hi);
    auto t4=std::chrono::high_resolution_clock::now();
    for(int i=0;i<N;i++){
        in->copyFromHostTensor(hi);
        ip->runSession(sess);
    }
    auto t5=std::chrono::high_resolution_clock::now();
    double avg_ms=std::chrono::duration<double,std::milli>(t5-t4).count()/N;
    printf("Avg subsequent run: %.2f ms (%.1f FPS)\n", avg_ms, 1000.0/avg_ms);
    printf("\nMesa direct Vulkan reference (7-stage FHD): 23.95 ms\n");
    printf("MNN overhead: %.1f× slower\n", avg_ms/23.95);
    
    // Check if the slow-down is in GPU dispatch or in MNN overhead
    printf("\nPossible bottlenecks:\n");
    printf("  1. Each MNN op resubmits command buffer (not batched)\n");
    printf("  2. Extra pipeline barriers add overhead\n");
    printf("  3. MNN descriptor write is slow\n");
    printf("  4. MNN pipeline creation is slow\n");
    printf("  5. Tensor memory management is slow\n");
    
    delete bc;delete ip;
    return 0;
}
