#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <chrono>
#include <functional>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "isp_opset.h"
static std::vector<uint8_t> rf(const char* p) {
    std::ifstream f(p, std::ios::binary|std::ios::ate);
    if(!f.good())return{}; size_t s=f.tellg();f.seekg(0);
    std::vector<uint8_t> b(s);f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};
int main(){
    const int FW=1920,FH=1080,BW=3840,BH=2160;
    std::string b="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    auto su=rf((b+"shader1_unpack_blc.spv").c_str());
    auto sd=rf((b+"isp_opsets/demosaic_noscale.spv").c_str());
    auto sf=rf((b+"shader3_fcs.spv").c_str());
    auto ss=rf((b+"shader6_display_simple.spv").c_str());
    
    // unpack + demosaic + display (3 stage, no FCS)
    isp::IspPipelineBuilder p3(FW,FH,3);
    p3.addStage(0,isp::UnpackBlc(BW,BH),ti8(su));
    p3.addStage(1,isp::DemosaicNoscale(FW,FH),ti8(sd));
    p3.addStage(2,isp::Display(FW,FH),ti8(ss));
    size_t m3;auto md3=p3.build(&m3);
    
    // unpack + demosaic + fcs + display (4 stage, with FCS)
    isp::IspPipelineBuilder p4(FW,FH,4);
    p4.addStage(0,isp::UnpackBlc(BW,BH),ti8(su));
    p4.addStage(1,isp::DemosaicNoscale(FW,FH),ti8(sd));
    p4.addStage(2,isp::Fcs(FW,FH),ti8(sf));
    p4.addStage(3,isp::Display(FW,FH),ti8(ss));
    size_t m4;auto md4=p4.build(&m4);
    
    printf("Models: 3-stage=%zu, 4-stage=%zu\n",m3,m4);
    
    dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
    auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();
    
    for(int variant=0;variant<2;variant++){
        auto md=variant?md4:md3;
        auto ms=variant?m4:m3;
        auto tn=variant?"tensor_4":"tensor_3";
        
        auto ip=MNN::Interpreter::createFromBuffer(md,ms);
        MNN::ScheduleConfig c;c.type=MNN_FORWARD_VULKAN;c.numThread=1;
        auto bc=new MNN::BackendConfig;bc->precision=MNN::BackendConfig::Precision_High;c.backendConfig=bc;
        auto ss=ip->createSession(c);
        auto in=ip->getSessionInput(ss,"tensor_0");
        ip->resizeTensor(in,{1,1,BH,BW});ip->resizeSession(ss);
        
        std::vector<int32_t> d(BW*BH);
        for(int y=0;y<BH;y++)for(int x=0;x<BW;x++)
            d[y*BW+x]=(y%2==0&&x%2==0)?100:(y%2==0&&x%2==1)?200:(y%2==1&&x%2==0)?300:400;
        auto hi=MNN::Tensor::create({1,1,BH,BW},in->getType(),d.data(),MNN::Tensor::CAFFE);
        in->copyFromHostTensor(hi);
        ip->runSession(ss);
        auto ot=ip->getSessionOutput(ss,tn);
        float* od=new float[ot->elementSize()];
        auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
        ot->copyToHostTensor(ho);
        
        int tot=ot->elementSize();
        float mn=1e9,mx=-1e9; int nz=0;
        for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
        
        // Sample in NHWC interleaved
        int cx=FW/2,cy=FH/2;
        printf("\n%s [%d/%d valid] [%.4f,%.4f]:\n",variant?"4-stage (with FCS)":"3-stage (no FCS)",nz,tot,mn,mx);
        printf("  Center(%d,%d): (%.4f,%.4f,%.4f)\n",cx,cy,
               od[cy*FW*3+cx*3+0],od[cy*FW*3+cx*3+1],od[cy*FW*3+cx*3+2]);
        // Check 4 corners
        for(int* p=(int[]){0,0,FW-1,0,0,FH-1,FW-1,FH-1};p<(int[]){0,0,FW-1,0,0,FH-1,FW-1,FH-1}+8;p+=2){
            int xx=p[0],yy=p[1];
            printf("  (%d,%d): (%.4f,%.4f,%.4f)\n",xx,yy,
                   od[yy*FW*3+xx*3+0],od[yy*FW*3+xx*3+1],od[yy*FW*3+xx*3+2]);
        }
        // Check a few raw values
        printf("  Raw[0..5]: ");
        for(int i=0;i<6;i++)printf("%.4f ",od[i]);
        printf("\n");
        // Check if any value == 1.0
        int n1=0;
        for(int i=0;i<tot;i++)if(od[i]>=0.9999)n1++;
        printf("  Values >= 1.0: %d/%d\n",n1,tot);
        
        delete[] od;delete bc;delete ip;
    }
    return 0;
}
