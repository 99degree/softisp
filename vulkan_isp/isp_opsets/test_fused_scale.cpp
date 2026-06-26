// Verify fused 4-stage pipeline across all resolutions
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
    std::ifstream f(p,std::ios::binary|std::ios::ate);if(!f.good())return{};
    size_t s=f.tellg();f.seekg(0);std::vector<uint8_t>b(s);f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};

int test_size(int W, int H) {
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto su =ti8(rf((base+"shader1_unpack_blc.spv").c_str()));
    auto sdn=ti8(rf((base+"isp_opsets/demosaic_noscale.spv").c_str()));
    auto sfd=ti8(rf((base+"shader_fcs_display_fused.spv").c_str()));
    auto sel=ti8(rf((base+"shader_ee_ldci_fused.spv").c_str()));
    if(su.empty()||sdn.empty()||sfd.empty()||sel.empty()){printf("FAIL: missing shader\n");return 1;}
    
    // Build fused 4-stage
    isp::IspPipelineBuilder pipe;
    pipe.addStage(isp::UnpackBlc(W,H),su);
    pipe.addStage(isp::DemosaicNoscale(W/2,H/2),sdn);
    pipe.addStage(isp::FcsDisplayFused(W/2,H/2,1.0,2.2),sfd);
    pipe.addStage(isp::EeLdciFused(W/2,H/2,0.5,0.01,0.5,1.0),sel);
    
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
    for(int y=0;y<H;y++) for(int x=0;x<W;x++)
        d[y*W+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
    auto hi=MNN::Tensor::create({1,1,H,W},in->getType(),d.data(),MNN::Tensor::CAFFE);
    in->copyFromHostTensor(hi);
    ip->runSession(sess);
    
    auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
    float* od=new float[ot->elementSize()];
    auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
    ot->copyToHostTensor(ho);
    
    int tot=ot->elementSize();
    int ow=ot->shape()[3],oh=ot->shape()[2];
    float mn=1e9,mx=-1e9;int nz=0;
    for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
    int cx=ow/2,cy=oh/2;
    float r=od[0*ow*oh+cy*ow+cx];
    float g=od[1*ow*oh+cy*ow+cx];
    float b=od[2*ow*oh+cy*ow+cx];
    bool ok=(nz==tot&&r>0.3f&&g>0.5f&&b>0.6f);
    printf("%4dx%-4d | %5d elems | [%.4f,%.4f] | RGB=(%.4f,%.4f,%.4f) | %s\n",
           ow,oh,tot,mn,mx,r,g,b,ok?"PASS":"FAIL");
    delete[] od;delete bc;delete ip;
    return ok?0:1;
}

int main(){
    printf(" Fused 4-stage (unpack+demosaic+fcs_display+ee_ldci)\n");
    printf(" Size     | Elems   | Range         | Center RGB          | Status\n");
    printf("----------+---------+---------------+---------------------+-------\n");
    test_size(8,8);     // 4×4 output
    test_size(16,16);   // 8×8 output
    test_size(32,32);
    test_size(64,64);
    test_size(128,128);
    test_size(256,256);
    test_size(512,512);
    test_size(640,480);
    test_size(1024,768);
    test_size(1920,1080);
    test_size(3840,2160);
    return 0;
}
