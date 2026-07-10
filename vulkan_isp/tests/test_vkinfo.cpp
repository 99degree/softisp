#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
int main() {
    auto interp = MNN::Interpreter::createFromFile(
        "/data/data/com.termux/files/home/softisp/vulkan_isp/fused_isp_unpack.mnn");
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    auto sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "FAIL\n"; return 1; }
    
    auto in = interp->getSessionInput(sess, "input");
    auto host = new MNN::Tensor(in, MNN::Tensor::CAFFE);
    in->copyToHostTensor(host);
    int32_t* d = host->host<int32_t>();
    for (int i = 0; i < host->elementSize(); i++) d[i] = 2048;
    in->copyFromHostTensor(host);
    
    auto err = interp->runSession(sess);
    
    auto out = interp->getSessionOutput(sess, "output");
    auto oh = new MNN::Tensor(out, MNN::Tensor::CAFFE);
    out->copyToHostTensor(oh);
    float* f = oh->host<float>();
    float mn=1e9, mx=-1e9;
    bool non_zero = false;
    int n = oh->elementSize();
    for (int i = 0; i < n; i++) {
        if (i < 10) std::cout << " [" << i << "]=" << f[i];
        if (f[i] != 0.0f) non_zero = true;
    }
    std::cout << "\nAny non-zero: " << (non_zero ? "YES" : "NO") << std::endl;
    std::cout << "Element count: " << n << std::endl;
    std::cout << "Bytes: " << out->size() << std::endl;
    
    delete host; delete oh; interp->releaseSession(sess); delete interp;
    return 0;
}
