// Debug: check if zero-copy path is actually being used
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
#include <MNN/HalideRuntime.h>
#include "isp_opset.h"
#include <vulkan/vulkan.h>

static std::vector<uint8_t> rf(const char*p){
    std::ifstream f(p,std::ios::binary|std::ios::ate);
    if(!f.good())return{};size_t s=f.tellg();f.seekg(0);
    std::vector<uint8_t> b(s);f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};

// Forward declarations for MNN internal types (we need VulkanBuffer)
// Actually, the tensor's deviceId() is a VulkanBuffer*
// Let's try to use the public API to check memory type

int main(){
    const int W=1920,H=1080;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto su=rf((base+"shader1_unpack_blc.spv").c_str());
    auto sd=rf((base+"shader2_demosaic_ccm.spv").c_str());
    auto ss=rf((base+"shader6_display_simple.spv").c_str());

    // Simple 2-stage: unpack + display
    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(W,H),ti8(su));
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
    
    // The output tensor deviceId should be a VulkanBuffer*
    void* devId = (void*)ot->deviceId();
    printf("Output tensor deviceId: %p\n", devId);
    printf("Output shape: [");
    for(int i=0;i<ot->shape().size();i++) printf("%d%c",ot->shape()[i],i+1<ot->shape().size()?',' : ']');
    printf("\n");
    printf("Output type: code=%d bits=%d\n", ot->getType().code, ot->getType().bits);
    printf("Host ptr: %p\n", ot->host<float>());

    // Try reading directly using the getTensorInfo API
    // We can try onGetTensorInfo if available through the session's backend
    // But it's not directly accessible. Let's just time the copyToHostTensor.
    
    // Benchmark readback timing
    const int ITERS=10;
    float* od = new float[ot->elementSize()];
    auto ho = MNN::Tensor::create(ot->shape(), ot->getType(), od, MNN::Tensor::CAFFE);
    
    double best=1e9;
    for(int i=0;i<ITERS;i++){
        auto t0=std::chrono::high_resolution_clock::now();
        ot->copyToHostTensor(ho);
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count();
        if(dt<best) best=dt;
    }
    
    int plane=W*H/4;  // Unpack outputs at W/2 * H/2
    float r=od[0*plane],g=od[1*plane],b=od[2*plane];
    printf("Readback: %.3f ms, center=(%.4f,%.4f,%.4f)\n", best, r, g, b);
    
    // Now test WITHOUT a pipeline run before readback (should be faster since GPU is idle)
    // Just copy from existing output
    best=1e9;
    for(int i=0;i<ITERS;i++){
        auto t0=std::chrono::high_resolution_clock::now();
        ot->copyToHostTensor(ho);
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count();
        if(dt<best) best=dt;
    }
    printf("Readback (GPU idle): %.3f ms = %.1f MB/s\n", best, 
           ot->elementSize()*4.0/1024/1024/(best/1000.0f));
    
    delete[] od;delete bc;delete ip;
    return 0;
}
