#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
int main() {
    auto interp = MNN::Interpreter::createFromFile(
        "/data/data/com.termux/files/home/softisp/vulkan_isp/fused_isp_unpack.mnn");
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_CPU;
    auto sess = interp->createSession(cfg);
    auto in = interp->getSessionInput(sess, "input");
    auto out = interp->getSessionOutput(sess, "output");
    std::cout << "In: [" << in->batch() << "," << in->channel() << "," 
              << in->height() << "," << in->width() << "]\n";
    std::cout << "Out: [" << out->batch() << "," << out->channel() << "," 
              << out->height() << "," << out->width() << "]\n";
    auto host = new MNN::Tensor(in, MNN::Tensor::CAFFE);
    in->copyToHostTensor(host);
    int32_t* d = host->host<int32_t>();
    for (int i = 0; i < host->elementSize(); i++) d[i] = 2048;
    in->copyFromHostTensor(host);
    auto err = interp->runSession(sess);
    std::cout << "Run: " << (err == MNN::NO_ERROR ? "OK" : "FAIL") << std::endl;
    auto oh = new MNN::Tensor(out, MNN::Tensor::CAFFE);
    out->copyToHostTensor(oh);
    float* f = oh->host<float>();
    float mn=1e9, mx=-1e9, s=0;
    int n = std::min(oh->elementSize(), 100);
    for (int i = 0; i < n; i++) { mn=std::min(mn,f[i]); mx=std::max(mx,f[i]); s+=f[i]; }
    std::cout << "Output: min=" << mn << " max=" << mx << " avg=" << (s/n) << std::endl;
    delete host; delete oh; interp->releaseSession(sess); delete interp;
    return 0;
}
