#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
#include <dlfcn.h>

int main() {
    // Explicitly load libMNN_Vulkan.so to ensure static constructors run
    void* handle = dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    if (!handle) {
        std::cerr << "dlopen failed: " << dlerror() << std::endl;
    } else {
        std::cout << "libMNN_Vulkan.so loaded via dlopen" << std::endl;
    }
    
    auto interp = MNN::Interpreter::createFromFile(
        "/data/data/com.termux/files/home/softisp/vulkan_isp/fused_isp_unpack.mnn");
    if (!interp) { std::cerr << "Failed\n"; return 1; }
    std::cout << "Model loaded" << std::endl;
    
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    auto sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "Session failed\n"; return 1; }
    std::cout << "Vulkan session OK" << std::endl;
    
    auto in = interp->getSessionInput(sess, "input");
    auto host = new MNN::Tensor(in, MNN::Tensor::CAFFE);
    in->copyToHostTensor(host);
    auto* d = host->host<int32_t>();
    for (int i = 0; i < host->elementSize(); i++) d[i] = 2048;
    in->copyFromHostTensor(host);
    
    auto err = interp->runSession(sess);
    std::cout << "Run: " << (err == MNN::NO_ERROR ? "OK" : "FAIL") << std::endl;
    
    auto out = interp->getSessionOutput(sess, "output");
    auto oh = new MNN::Tensor(out, MNN::Tensor::CAFFE);
    out->copyToHostTensor(oh);
    float* f = oh->host<float>();
    bool non_zero = false;
    for (int i = 0; i < std::min(10, oh->elementSize()); i++) {
        if (f[i] != 0) non_zero = true;
        std::cout << " [" << i << "]=" << f[i];
    }
    std::cout << "\nNon-zero: " << (non_zero ? "YES" : "NO") << std::endl;
    
    if (handle) dlclose(handle);
    delete host; delete oh; interp->releaseSession(sess); delete interp;
    return 0;
}
