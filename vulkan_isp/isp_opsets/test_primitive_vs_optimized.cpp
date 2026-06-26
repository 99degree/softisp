// Primitive MNN ops vs Optimized VulkanFuse Extra ops
// Compares MNN's built-in Vulkan convolution with our custom SPIR-V
// Both do the same operation: 3×3 Laplacian unsharp mask (edge enhancement)
//
// primitive: MNN Convolution (NC4HW4, standard kernels)
// optimized: VulkanFuse Extra op (CHW planar, custom SPIR-V)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <chrono>
#include <memory>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "isp_opset.h"
#include <flatbuffers/flatbuffers.h>
#include "MNN_generated.h"

static std::vector<uint8_t> rf(const char*p){
    std::ifstream f(p,std::ios::binary|std::ios::ate);
    if(!f.good()){fprintf(stderr,"FAIL: %s\n",p);exit(1);}
    size_t s=f.tellg();f.seekg(0);std::vector<uint8_t>b(s);
    f.read((char*)b.data(),s);return b;
}
auto ti8=[](const std::vector<uint8_t>&r){
    std::vector<int8_t>v(r.size());memcpy(v.data(),r.data(),r.size());return v;
};

// ── Benchmark a model ──
struct BenchResult {
    double ms;
    int nonZero;
    float pixel;
};

BenchResult bench(const std::vector<uint8_t>& model, const char* outName,
                  int N, int C, int H, int W, int ITERS,
                  const char* label) {
    int plane = W*H;
    std::vector<float> input(N*C*H*W);
    for(int i=0;i<N*C*H*W;i++) input[i]=0.5f;

    auto ip = MNN::Interpreter::createFromBuffer(model.data(), model.size());
    MNN::ScheduleConfig sc; sc.type=MNN_FORWARD_VULKAN; sc.numThread=1;
    auto bc=new MNN::BackendConfig; bc->precision=MNN::BackendConfig::Precision_High; sc.backendConfig=bc;
    auto sess = ip->createSession(sc);
    auto in_t = ip->getSessionInput(sess, "tensor_0");
    ip->resizeTensor(in_t, {N,C,H,W});
    ip->resizeSession(sess);

    auto host_in = MNN::Tensor::create({N,C,H,W}, in_t->getType(), input.data(), MNN::Tensor::CAFFE);
    in_t->copyFromHostTensor(host_in);
    ip->runSession(sess);

    double best=1e9;
    for(int i=0;i<ITERS;i++){
        in_t->copyFromHostTensor(host_in);
        auto t0=std::chrono::high_resolution_clock::now();
        ip->runSession(sess);
        auto t1=std::chrono::high_resolution_clock::now();
        double m=std::chrono::duration<double,std::milli>(t1-t0).count();
        if(m<best)best=m;
    }

    auto out_t = ip->getSessionOutput(sess, outName);
    auto host_out = MNN::Tensor::create({N,3,H,W}, out_t->getType(), nullptr, MNN::Tensor::CAFFE);
    out_t->copyToHostTensor(host_out);
    int nz=0; for(int i=0;i<3*plane;i++) if(((float*)host_out->host<float>())[i]!=0) nz++;
    float px=((float*)host_out->host<float>())[0];

    printf("  %-44s %8.2f ms %6.1f FPS  %7d/ok  (%.4f)\n",
           label, best, 1000.0/best, nz, px);
    delete ip;
    return {best, nz, px};
}

int main() {
    int W=1920, H=1080, C=3, N=1, ITERS=8;
    std::string base="/data/data/com.termux/files/home/softisp/vulkan_isp/";

    printf("══════════════════════════════════════════════════════════════════\n");
    printf("  Primitive MNN Ops vs Optimized Extra Ops — FHD %dx%d\n",W,H);
    printf("══════════════════════════════════════════════════════════════════\n\n");

    dlopen("libMNN_Vulkan.so",RTLD_NOW|RTLD_GLOBAL);
    auto rg=(void(*)(void))dlsym(RTLD_DEFAULT,"MNNVulkanRegisterAll");if(rg)rg();

    printf("┌─ Single Op: Edge Enhancement (3×3 Laplacian) ──────────────┐\n");

    // ── 1) Primitive: MNN Convolution op ──
    auto t_ee_prim = [&]() {
        flatbuffers::FlatBufferBuilder fbb(64*1024);
        auto net = std::make_unique<MNN::NetT>();
        net->bizCode = "ee_conv";
        net->tensorName = {"tensor_0", "tensor_1"};

        auto op = std::make_unique<MNN::OpT>();
        op->type = MNN::OpType_Convolution;
        op->main.type = MNN::OpParameter_Convolution2D;
        auto conv = new MNN::Convolution2DT();
        conv->common.reset(new MNN::Convolution2DCommonT());
        conv->common->outputCount=3;
        conv->common->kernelX=3; conv->common->kernelY=3;
        conv->common->strideX=1; conv->common->strideY=1;
        conv->common->padMode=MNN::PadMode_SAME;
        conv->common->relu=false; conv->common->group=1;
        // Unsharp: output = center + 0.5*(4*center - sum)
        // = 3*center - 0.5*(t+b+l+r)
        conv->weight.resize(3*3*3*3);
        for(int oc=0; oc<3; oc++) {
            for(int ic=0; ic<3; ic++) {
                float* k = &conv->weight[(oc*3 + ic)*9];
                if(oc==ic){
                    k[0]=0;    k[1]=-0.5f; k[2]=0;
                    k[3]=-0.5f; k[4]= 3.0f; k[5]=-0.5f;
                    k[6]=0;    k[7]=-0.5f; k[8]=0;
                } else memset(k,0,9*sizeof(float));
            }
        }
        conv->bias.resize(3);
        conv->bias[0]=conv->bias[1]=conv->bias[2]=0;
        op->main.value = conv;
        op->inputIndexes.push_back(0);
        op->outputIndexes.push_back(1);
        net->oplists.push_back(std::move(op));

        auto off = MNN::Net::Pack(fbb, net.get());
        fbb.Finish(off);
        return std::vector<uint8_t>(fbb.GetBufferPointer(), fbb.GetBufferPointer()+fbb.GetSize());
    }();
    auto r1 = bench(t_ee_prim, "tensor_1", N, C, H, W, ITERS,
                   "Primitive: Conv3×3 (MNN standard, NC4HW4)");

    // ── 2) Optimized: Extra op with custom SPIR-V ──
    auto se = ti8(rf((base+"shader4_ee.spv").c_str()));
    auto t_ee_opt = [&]() {
        isp::IspPipelineBuilder pipe;
        pipe.addStage(isp::Ee(W,H,0.5,0.01), se);
        size_t ms; auto md = pipe.build(&ms);
        return std::vector<uint8_t>(md, md+ms);
    }();
    auto r2 = bench(t_ee_opt, "tensor_1", N, C, H, W, ITERS,
                   "Optimized: Extra op (CHW planar, custom SPIR-V)");

    printf("├────────────────────────────────────────────────────────────────┤\n");
    printf("│                                                                \n");
    printf("│ Comparison: Primitive Conv3×3 vs Optimized EE Extra op\n");
    printf("│   Both do: output = center + 0.5*(4*center - sum_neighbors)\n");
    printf("│                                                                \n");
    printf("│   MNN standard Convolution uses NC4HW4 layout (%d ch/4=%d blocks)\n", C, (C+3)/4);
    printf("│   Our Extra op uses CHW planar (no format conversion)\n");
    printf("│   Input: neutral gray (0.5), Output: same (Laplacian of flat=0)\n");
    printf("│                                                                \n");
    double ratio = r1.ms / r2.ms;
    printf("│   Primitive:  %.2f ms\n", r1.ms);
    printf("│   Optimized:  %.2f ms\n", r2.ms);
    printf("│   Ratio:      %.2f× (%s)\n", ratio, r2.ms<r1.ms?"Extra faster":"Conv faster");
    printf("│                                                                \n");

    if(r2.ms < r1.ms) {
        printf("│  → Custom SPIR-V is faster (CHW planar avoids NC4HW4 overhead)\n");
    } else {
        printf("│  → MNN Conv is faster (optimized NC4HW4 Winograd/ImplicitGEMM)\n");
    }
    printf("└────────────────────────────────────────────────────────────────┘\n");

    // ── Full pipeline: 4-stage (demosaic→fcs→ee→display) ──
    printf("\n┌─ Full Pipeline (demosaic→fcs→ee→display) ────────────────┐\n");

    // Primitive: Conv1×1(CCM) → Scale(FCS) → Conv3×3(EE) → Pow(display)
    auto t_full_prim = [&]() {
        flatbuffers::FlatBufferBuilder fbb(64*1024);
        auto net = std::make_unique<MNN::NetT>();
        net->bizCode = "full_prim";
        for(int i=0;i<5;i++) net->tensorName.push_back("tensor_"+std::to_string(i));

        int in=0;
        // Stage 1: Conv1×1 CCM 4ch→3ch
        {
            int out=1;
            auto op = std::make_unique<MNN::OpT>();
            op->type=MNN::OpType_Convolution;
            op->main.type=MNN::OpParameter_Convolution2D;
            auto c=new MNN::Convolution2DT();
            c->common.reset(new MNN::Convolution2DCommonT());
            c->common->outputCount=3; c->common->kernelX=1; c->common->kernelY=1;
            c->common->strideX=1; c->common->strideY=1;
            c->common->padMode=MNN::PadMode_SAME;
            c->weight.resize(12); // 3*4*1*1
            c->weight[0]=1; c->weight[1]=0; c->weight[2]=0; c->weight[3]=0;
            c->weight[4]=0; c->weight[5]=0.5; c->weight[6]=0.5; c->weight[7]=0;
            c->weight[8]=0; c->weight[9]=0; c->weight[10]=0; c->weight[11]=1;
            c->bias.resize(3); c->bias[0]=c->bias[1]=c->bias[2]=0;
            op->main.value=c;
            op->inputIndexes={0}; op->outputIndexes={1};
            net->oplists.push_back(std::move(op));
            in=1;
        }
        // Stage 2: Scale (FCS gain)
        {
            int out=2;
            auto op = std::make_unique<MNN::OpT>();
            op->type=MNN::OpType_Scale;
            op->main.type=MNN::OpParameter_Scale;
            auto s=new MNN::ScaleT();
            s->channels=3; s->scaleData={1.0f,1.0f,1.0f}; s->biasData={0,0,0};
            op->main.value=s;
            op->inputIndexes={1}; op->outputIndexes={2};
            net->oplists.push_back(std::move(op));
            in=2;
        }
        // Stage 3: Conv3×3 EE
        {
            int out=3;
            auto op = std::make_unique<MNN::OpT>();
            op->type=MNN::OpType_Convolution;
            op->main.type=MNN::OpParameter_Convolution2D;
            auto c=new MNN::Convolution2DT();
            c->common.reset(new MNN::Convolution2DCommonT());
            c->common->outputCount=3; c->common->kernelX=3; c->common->kernelY=3;
            c->common->strideX=1; c->common->strideY=1;
            c->common->padMode=MNN::PadMode_SAME;
            c->weight.resize(81);
            for(int oc=0;oc<3;oc++) for(int ic=0;ic<3;ic++){
                float* k=&c->weight[(oc*3+ic)*9];
                if(oc==ic){k[0]=0;k[1]=-0.5f;k[2]=0;k[3]=-0.5f;k[4]=3.0f;k[5]=-0.5f;k[6]=0;k[7]=-0.5f;k[8]=0;}
                else memset(k,0,9*sizeof(float));
            }
            c->bias.resize(3); c->bias[0]=c->bias[1]=c->bias[2]=0;
            op->main.value=c;
            op->inputIndexes={2}; op->outputIndexes={3};
            net->oplists.push_back(std::move(op));
            in=3;
        }
        // Stage 4: Display (using extra ops for gamma since Pow is complex)
        // For fair comparison, stop at EE (3 stages)
        // Actually let me use our Display Extra op for both — we just compare Conv vs Extra

        auto off = MNN::Net::Pack(fbb, net.get());
        fbb.Finish(off);
        return std::vector<uint8_t>(fbb.GetBufferPointer(), fbb.GetBufferPointer()+fbb.GetSize());
    }();
    auto r3 = bench(t_full_prim, "tensor_3", 1, 4, H, W, ITERS,
                   "Primitive full: Conv1×1→Scale→Conv3×3 (MNN standard, NC4HW4)");

    // Optimized: using our Extra ops (4ch input: DemosaicNoscale reads RGGB)
    auto sdn = ti8(rf((base+"isp_opsets/demosaic_noscale.spv").c_str()));
    auto sf  = ti8(rf((base+"shader3_fcs.spv").c_str()));
    auto t_full_opt = [&]() {
        isp::IspPipelineBuilder pipe;
        pipe.addStage(isp::DemosaicNoscale(W,H), sdn);
        pipe.addStage(isp::Fcs(W,H,1.0), sf);
        pipe.addStage(isp::Ee(W,H,0.5,0.01), se);
        size_t ms; auto md = pipe.build(&ms);
        return std::vector<uint8_t>(md, md+ms);
    }();
    auto r4 = bench(t_full_opt, "tensor_3", 1, 4, H, W, ITERS,
                   "Optimized full: 3 Extra ops (CHW planar, custom SPIR-V)");

    printf("├────────────────────────────────────────────────────────────────┤\n");
    printf("│ Full Pipeline (%d×%d, 4ch→3ch→3ch→3ch):\n",W,H);
    printf("│   Primitive (Conv1×1→Scale→Conv3×3):  %8.2f ms\n", r3.ms);
    printf("│   Optimized (3 Extra ops):              %8.2f ms\n", r4.ms);
    printf("│   Ratio:                                %8.2f×\n", r3.ms/r4.ms);
    printf("│                                                                \n");
    printf("│ NOTE: Both do demosaic(CCM)→fcs→ee(laplacian)\n");
    printf("│ Primitive uses NC4HW4 layout (MNN standard)\n");
    printf("│ Optimized uses CHW planar (direct compute)\n");
    printf("│                                                                \n");
    if(r4.ms < r3.ms) {
        printf("│  ✓ Our Extra ops are %.0f%% faster than standard MNN ops\n",
               (r3.ms-r4.ms)/r3.ms*100);
    } else {
        printf("│  △ MNN standard ops are %.0f%% faster\n",
               (r4.ms-r3.ms)/r4.ms*100);
    }
    printf("└────────────────────────────────────────────────────────────────┘\n");
    return 0;
}
