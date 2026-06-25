#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "schema/current/MNN_generated.h"

int main() {
    // Load model from buffer
    FILE* f = fopen("isp_pipeline.mnn", "rb");
    if (!f) { printf("FAIL: open model\n"); return 1; }
    fseek(f, 0, SEEK_END);
    size_t sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* buf = new uint8_t[sz];
    fread(buf, 1, sz, f);
    fclose(f);
    
    // Load vulkan
    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    typedef void (*RegFunc)();
    RegFunc reg = (RegFunc)dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();
    
    auto interp = MNN::Interpreter::createFromBuffer(buf, sz);
    if (!interp) { printf("FAIL: interp\n"); return 1; }
    
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;
    
    auto sess = interp->createSession(cfg);
    if (!sess) { printf("FAIL: session\n"); return 1; }
    
    // Set input
    auto input = interp->getSessionInput(sess, "tensor_0");
    std::vector<int> inDims = {1, 1, 2160, 3840};
    interp->resizeTensor(input, inDims);
    interp->resizeSession(sess);
    
    // Check all output tensors
    for (int i = 0; i <= 6; i++) {
        char name[32];
        snprintf(name, sizeof(name), "tensor_%d", i);
        auto t = interp->getSessionOutput(sess, name);
        if (t) {
            printf("Tensor %d (%s): dims=[", i, name);
            for (int j = 0; j < t->buffer().dimensions; j++)
                printf("%s%d", j?",":"", t->length(j));
            printf("] host=%p deviceId=%llx\n",
                (void*)t->buffer().host, (unsigned long long)t->deviceId());
        } else {
            printf("Tensor %d (%s): NOT FOUND\n", i, name);
        }
    }
    
    interp->releaseSession(sess);
    delete interp;
    delete[] buf;
    return 0;
}
