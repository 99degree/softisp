#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include "schema/current/MNN_generated.h"

int main() {
    // Load model
    FILE* f = fopen("isp_pipeline.mnn", "rb");
    fseek(f, 0, SEEK_END);
    size_t sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* buf = new uint8_t[sz];
    fread(buf, 1, sz, f);
    fclose(f);
    
    // Init Vulkan
    dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    auto reg = (void(*)(void))dlsym(RTLD_DEFAULT, "MNNVulkanRegisterAll");
    if (reg) reg();
    
    auto interp = MNN::Interpreter::createFromBuffer(buf, sz);
    
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    auto bc = new MNN::BackendConfig;
    bc->precision = MNN::BackendConfig::Precision_High;
    cfg.backendConfig = bc;
    
    auto sess = interp->createSession(cfg);
    
    // Set input
    auto input = interp->getSessionInput(sess, "tensor_0");
    std::vector<int> inDims = {1, 1, 1080, 1920};  // smaller for speed
    interp->resizeTensor(input, inDims);
    interp->resizeSession(sess);
    
    // Allocate and fill input
    float* hostPtr = new float[inDims[0] * inDims[1] * inDims[2] * inDims[3]];
    for (int i = 0; i < 1080 * 1920; i++) {
        hostPtr[i] = (float)(i % 1024);  // ramp pattern
    }
    
    auto hostInput = MNN::Tensor::create(inDims, input->getType(), hostPtr, MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostInput);
    printf("Input copied, elementSize=%d\n", input->elementSize());
    
    // Run
    auto code = interp->runSession(sess);
    printf("Run: code=%d\n", (int)code);
    
    // Read output
    auto out = interp->getSessionOutput(sess, "tensor_6");
    printf("Output: dims=%d host=%p\n", out->buffer().dimensions, (void*)out->buffer().host);
    
    // Create host output with manual allocation
    float* outPtr = new float[out->elementSize()];
    memset(outPtr, 0xAB, out->elementSize() * sizeof(float));
    auto hostOut = MNN::Tensor::create(out->shape(), out->getType(), outPtr, MNN::Tensor::CAFFE);
    
    printf("Before copy: outPtr[0]=%f (0x%08x)\n", outPtr[0], *(int*)&outPtr[0]);
    
    bool ok = out->copyToHostTensor(hostOut);
    printf("copyToHostTensor returned: %d\n", ok);
    
    // Check output
    int nz = 0;
    int n = hostOut->elementSize();
    n = n > 100 ? 100 : n;
    for (int i = 0; i < n; i++) {
        if (hostOut->host<float>()[i] != 0.0f) nz++;
    }
    printf("First %d elements non-zero: %d\n", n, nz);
    printf("Values[0..3]: %f %f %f %f\n",
        hostOut->host<float>()[0], hostOut->host<float>()[1],
        hostOut->host<float>()[2], hostOut->host<float>()[3]);
    
    interp->releaseSession(sess);
    delete interp;
    delete[] buf;
    delete[] hostPtr;
    delete[] outPtr;
    return 0;
}
