// Per-stage diagnostic: find which stage fails at 4K→FHD scale
// Tests each pipeline prefix: 1-stage, 2-stage, ..., 6-stage
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <functional>
#include <map>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "isp_opset.h"

static std::vector<uint8_t> rf(const char*p){
    std::ifstream f(p,std::ios::binary|std::ios::ate);
    if(!f.good()){fprintf(stderr,"FAIL: can't read %s\n",p);return{};}
    size_t s=f.tellg();f.seekg(0);
    std::vector<uint8_t> b(s);f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;};

int main(int argc, char** argv){
    const int BW=3840,BH=2160,FW=1920,FH=1080;
    // Parse stage count filter
    int max_stages = argc>1 ? atoi(argv[1]) : 6;
    if(max_stages<1||max_stages>6)max_stages=6;
    
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";
    
    // Load all shaders
    struct Stage { const char* name; std::vector<int8_t> spv; isp::OpDesc (*desc)(int,int); int w,h; };
    std::vector<Stage> stages;
    auto s1=rf((base+"shader1_unpack_blc.spv").c_str());
    auto s2n=rf((base+"isp_opsets/demosaic_noscale.spv").c_str());
    auto s3=rf((base+"shader3_fcs.spv").c_str());
    auto s4=rf((base+"shader4_ee.spv").c_str());
    auto s5=rf((base+"shader5_ldci.spv").c_str());
    auto s6=rf((base+"shader6_display_simple.spv").c_str());
    
    if(s1.empty()||s2n.empty()||s3.empty()||s4.empty()||s5.empty()||s6.empty()){
        fprintf(stderr,"FAIL: missing shader files\n"); return 1;
    }
    
    stages.push_back({"unpack",  ti8(s1),  isp::UnpackBlc,       BW,BH});
    stages.push_back({"demosaic",ti8(s2n), isp::DemosaicNoscale, FW,FH});
    stages.push_back({"fcs",     ti8(s3),  [](int w,int h){return isp::Fcs(w,h);}, FW,FH});
    stages.push_back({"ee",      ti8(s4),  [](int w,int h){return isp::Ee(w,h);},  FW,FH});
    stages.push_back({"ldci",    ti8(s5),  [](int w,int h){return isp::Ldci(w,h);},FW,FH});
    stages.push_back({"display", ti8(s6),  [](int w,int h){return isp::Display(w,h);},FW,FH});
    
    printf("=== 4K→FHD Per-Stage Diagnostic ===\n");
    printf("Input:  %dx%d Bayer (int32), Output: %dx%d RGB (float32)\n", BW,BH,FW,FH);
    printf("Testing up to %d stages\n\n", max_stages);
    
    for(int n=1;n<=max_stages;n++){
        printf("--- Stage %d: ", n);
        for(int j=0;j<n;j++) printf("%s%s",j>0?"+":"",stages[j].name);
        printf(" ---\n");
        
        isp::IspPipelineBuilder pipe;
        for(int j=0;j<n;j++)
            pipe.addStage(stages[j].desc(stages[j].w,stages[j].h), stages[j].spv);
        
        size_t ms;auto md=pipe.build(&ms);
        printf("  Model: %zu bytes\n", ms);
        
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
        
        // Warmup + benchmark
        ip->runSession(sess);
        auto t0=std::chrono::high_resolution_clock::now();
        int N=5;
        for(int i=0;i<N;i++){ in->copyFromHostTensor(hi); ip->runSession(sess); }
        auto t1=std::chrono::high_resolution_clock::now();
        double dt=std::chrono::duration<double,std::milli>(t1-t0).count()/N;
        
        auto ot=ip->getSessionOutput(sess,pipe.outputTensorName().c_str());
        float* od=new float[ot->elementSize()];
        auto ho=MNN::Tensor::create(ot->shape(),ot->getType(),od,MNN::Tensor::CAFFE);
        ot->copyToHostTensor(ho);
        
        int tot=ot->elementSize();
        int ow=ot->shape()[3],oh=ot->shape()[2],ch=ot->shape()[1];
        float mn=1e9,mx=-1e9; int nz=0;
        for(int i=0;i<tot;i++){if(fabsf(od[i])>1e-6)nz++;if(od[i]<mn)mn=od[i];if(od[i]>mx)mx=od[i];}
        
        int cx=ow/2,cy=oh/2;
        float r=od[0*ow*oh+cy*ow+cx];
        float g=ch>1?od[1*ow*oh+cy*ow+cx]:0;
        float b=ch>2?od[2*ow*oh+cy*ow+cx]:0;
        
        // Check if output looks reasonable
        // For unpack: 4-channel RGGB, values should be ~0.1-0.4
        // For RGB stages: values should be ~0.3-0.7
        bool ok = nz==tot && mn>0.05f;
        printf("  Time: %.2fms, Output: %d/%d nz, range=[%.4f,%.4f]\n", dt, nz, tot, mn, mx);
        printf("  Center(%d,%d): ch[0]=%.4f",cx,cy,r);
        if(ch>=2) printf(" ch[1]=%.4f",g);
        if(ch>=3) printf(" ch[2]=%.4f",b);
        printf(" (%d-ch)\n",ch);
        printf("  Status: %s\n", ok ? "PASS ✓" : "FAIL ✗");
        
        if(!ok && nz<tot){
            // Show histogram of zero vs non-zero by channel
            if(ch>1){
                for(int c=0;c<ch;c++){
                    int zc=0;
                    for(int y=0;y<oh;y++) for(int x=0;x<ow;x++)
                        if(fabsf(od[c*ow*oh+y*ow+x])<1e-6) zc++;
                    printf("    Channel %d: %d/%d zero\n",c,zc,ow*oh);
                }
            }
        }
        
        delete[] od;delete bc;delete ip;
        printf("\n");
    }
    return 0;
}
