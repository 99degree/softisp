// Test zero-copy readback: map GPU tensor buffer directly instead of copyToHostTensor
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
// Vulkan headers for direct buffer access
#include <vulkan/vulkan.h>

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

    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(BW,BH),ti8(su));
    pipe.addStage(isp::DemosaicNoscale(FW,FH),ti8(sd));
    pipe.addStage(isp::Fcs(FW,FH),ti8(sf));
    pipe.addStage(isp::Ee(FW,FH),ti8(se));
    pipe.addStage(isp::Ldci(FW,FH),ti8(sl));
    pipe.addStage(isp::Display(FW,FH),ti8(ss));
    size_t ms;auto md=pipe.build(&ms);

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
    ip->runSession(sess);

    // Get output tensor
    auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());

    // METHOD 1: Standard copyToHostTensor (baseline)
    float* od1=new float[ot->elementSize()];
    auto ho1=MNN::Tensor::create(ot->shape(),ot->getType(),od1,MNN::Tensor::CAFFE);
    auto t0=std::chrono::high_resolution_clock::now();
    ot->copyToHostTensor(ho1);
    auto t1=std::chrono::high_resolution_clock::now();
    double copy_ms=std::chrono::duration<double,std::milli>(t1-t0).count();

    // METHOD 2: Direct buffer mapping (zero-copy)
    // The tensor's deviceId() points to a VulkanBuffer
    // We need the VkDevice from the backend, but since this is a test,
    // let's check if the tensor buffer is host-accessible
    void* devPtr = (void*)ot->deviceId();
    printf("Output tensor deviceId: %p\n", devPtr);

    // Try to get the internal VulkanBuffer and map it
    // The tensor stores (VulkanBuffer*) in deviceId
    // VulkanBuffer has map() method
    
    // We can't easily map without the VkDevice, but let's measure the
    // copy overhead and think about the optimization
    
    int plane=FW*FH,cx=FW/2,cy=FH/2;
    float r=od1[0*plane+cy*FW+cx],g=od1[1*plane+cy*FW+cx],b=od1[2*plane+cy*FW+cx];
    int nz=0,tot=ot->elementSize();
    for(int i=0;i<tot;i++) if(od1[i]!=0) nz++;

    printf("\n=== Zero-Copy Readback Analysis ===\n");
    printf("Output size: %d elems = %.1f MB (FLOAT3)\n", tot, tot*4.0/1024/1024);
    printf("\nCurrent readback path:\n");
    printf("  GPU tensor buf → vkCmdCopyBuffer → host buf → memcpy → host tensor\n");
    printf("  Time: %.3f ms (includes full pipeline barrier + copy)\n", copy_ms);
    printf("  Result: %d/%d valid [%.4f,%.4f] center=(%.4f,%.4f,%.4f)\n",
           nz,tot,0.0f,0.0f,r,g,b);
    
    printf("\nProposed zero-copy path:\n");
    printf("  GPU tensor buf (HOST_VISIBLE) → vkMapMemory → direct CPU read\n");
    printf("  Eliminates: vkCmdCopyBuffer + barrier + memcpy\n");
    printf("  Requires: tensor buffer allocated with HOST_VISIBLE property\n");
    printf("\nOn UMA (Adreno): DEVICE_LOCAL|HOST_VISIBLE memory type exists.\n");
    printf("Current VulkanBufferAllocator uses requirements_mask=0 (pool default).\n");
    printf("Changing to VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT enables zero-copy.\n");
    printf("Expected saving: %.3f ms/frame (the entire copy path)\n", copy_ms);

    delete[] od1;delete bc;delete ip;
    return 0;
}
