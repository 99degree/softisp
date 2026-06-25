#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "schema/current/MNN_generated.h"

int main() {
    auto check = [](const char* modelPath, const char* name) {
        FILE* f = fopen(modelPath, "rb");
        if (!f) { printf("%s: FAIL open\n", name); return; }
        fseek(f, 0, SEEK_END);
        size_t sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        uint8_t* buf = new uint8_t[sz];
        fread(buf, 1, sz, f);
        fclose(f);
        
        auto interp = MNN::Interpreter::createFromBuffer(buf, sz);
        if (!interp) { printf("%s: FAIL interp\n", name); delete[] buf; return; }
        
        MNN::ScheduleConfig cfg;
        cfg.type = MNN_FORWARD_VULKAN;
        auto bc = new MNN::BackendConfig;
        bc->precision = MNN::BackendConfig::Precision_High;
        cfg.backendConfig = bc;
        
        auto sess = interp->createSession(cfg);
        if (!sess) { printf("%s: FAIL sess\n", name); return; }
        
        auto out = interp->getSessionOutput(sess, "tensor_1");
        if (out) {
            printf("%s: tensor_1 found, host=%p deviceId=%llx\n", name,
                (void*)out->buffer().host, (unsigned long long)out->deviceId());
        } else {
            printf("%s: tensor_1 NOT FOUND\n", name);
        }
        
        interp->releaseSession(sess);
        delete interp;
        delete[] buf;
    };
    
    // Need to init Vulkan first
    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    typedef void (*RegFunc)();
    RegFunc reg = (RegFunc)dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();
    
    check("isp_pipeline.mnn", "pipeline");
    check("test_raw.mnn", "single");
}

