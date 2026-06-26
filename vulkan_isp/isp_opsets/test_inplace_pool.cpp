// test_inplace_pool.cpp — Verify in-place tensor pooling works
// Stages 2 (FCS) and 5 (Display) use inplace=true → reuse input tensor buffer.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <functional>
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

void test_pipeline(bool use_inplace, const char* label){
    const int BW=3840,BH=2160,FW=1920,FH=1080;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto su=rf((base+"shader1_unpack_blc.spv").c_str());
    auto sd=rf((base+"isp_opsets/demosaic_noscale.spv").c_str());
    auto sf=rf((base+"shader3_fcs.spv").c_str());
    auto se=rf((base+"shader4_ee.spv").c_str());
    auto sl=rf((base+"shader5_ldci.spv").c_str());
    auto ss=rf((base+"shader6_display_simple.spv").c_str());

    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(BW,BH),ti8(su),false);        // stage 0: t0→t1
    pipe.addStage(isp::DemosaicNoscale(FW,FH),ti8(sd),false);   // stage 1: t1→t2
    pipe.addStage(isp::Fcs(FW,FH),ti8(sf), use_inplace);        // stage 2: t2→t2 or t2→t3
    pipe.addStage(isp::Ee(FW,FH),ti8(se), false);              // stage 3: t2→t3 or t3→t4
    pipe.addStage(isp::Ldci(FW,FH),ti8(sl), false);             // stage 4: t3→t4 or t4→t5
    pipe.addStage(isp::Display(FW,FH),ti8(ss), use_inplace);    // stage 5: t4→t4 or t5→t6

    size_t ms;auto md=pipe.build(&ms);
    printf("%-30s | tensors=%d | model=%zuB, output=%s | ",
           label, pipe.tensorCount(), ms, pipe.outputTensorName().c_str());

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

    auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);

    int plane=FW*FH,cx=FW/2,cy=FH/2;
    int nz=0,tot=ot->elementSize();float mn=1e9,mx=-1e9;
    for(int i=0;i<tot;i++){if(od[i]!=0)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    float r=od[0*plane+cy*FW+cx],g=od[1*plane+cy*FW+cx],b=od[2*plane+cy*FW+cx];
    printf("%d/%d val [%.4f,%.4f] RGB=(%.4f,%.4f,%.4f) %s\n",
           nz,tot,mn,mx,r,g,b,(nz==tot&&mn>0.3f)?"PASS":"FAIL");
    delete[] od;delete bc;delete ip;
}

int main(){
    printf("Pipeline tensor pooling test (4K→FHD 6-stage)\n");
    printf("────────────────────────────────────────────────────────────────\n");
    test_pipeline(false, "no inplace (7 tensors)");
    test_pipeline(true,  "inplace FCS+Display (5 tensors)");
    printf("────────────────────────────────────────────────────────────────\n");
    printf("Memory: 5 tensors = %.0f MB vs 7 = %.0f MB = %.0f%% of original\n",
           5.0*33+4.0*25, 7.0*33+6.0*25, (5.0*33+4.0*25)/(7.0*33+6.0*25)*100);
    return 0;
}
