#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/HalideRuntime.h>
#include <map>
#include <vector>
#include <stdio.h>
#include <chrono>
#include <half.hpp>

int main(int argc, char* argv[]) {
    if (argc < 2) return 1;
    MNN::Interpreter* net = MNN::Interpreter::createFromFile(argv[1]);
    if (!net) { fprintf(stderr, "FAIL: createFromFile\n"); return 1; }
    
    MNN::ScheduleConfig config;
    config.numThread = 4;
    config.type = (MNNForwardType)7;  // Vulkan
    
    MNN::Session* session = net->createSession(config);
    if (!session) { fprintf(stderr, "FAIL: createSession\n"); return 1; }
    
    // Check runtime
    auto runtime = session->getRuntime();
    if (runtime) {
        printf("Runtime type: %d (0=CPU, 3=OpenCL, 6=OpenGL, 7=Vulkan)\n", runtime->type());
    }
    
    MNN::Tensor* input = net->getSessionInput(session, NULL);
    if (!input) { fprintf(stderr, "FAIL: getSessionInput\n"); return 1; }
    
    printf("Input tensor: shape=");
    for (int i = 0; i < input->shape().size(); i++) {
        printf("%d ", input->shape()[i]);
    }
    printf("\n");
    
    // Fill input with test data
    int n = input->elementSize();
    void* hostPtr = input->host<void>();
    if (hostPtr && n > 0) {
        uint16_t* i16 = (uint16_t*)hostPtr;
        for (int i = 0; i < n; i++) i16[i] = (i % 1024);
        printf("Filled %d elements directly\n", n);
    } else {
        MNN::Tensor* hostTensor = MNN::Tensor::createHostTensorFromDevice(input, false);
        if (hostTensor) {
            int n2 = hostTensor->elementSize();
            void* p2 = hostTensor->host<void>();
            if (p2 && n2 > 0) {
                uint16_t* i16 = (uint16_t*)p2;
                for (int i = 0; i < n2; i++) i16[i] = (i % 1024);
                input->copyFromHostTensor(hostTensor);
                printf("Filled %d elements via host tensor\n", n2);
            }
            delete hostTensor;
        }
    }
    
    // Run session
    auto t0 = std::chrono::high_resolution_clock::now();
    MNN::ErrorCode err = net->runSession(session);
    auto t1 = std::chrono::high_resolution_clock::now();
    
    if (err != MNN::NO_ERROR) {
        fprintf(stderr, "runSession error: %d\n", err);
    } else {
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        printf("runSession took %.4f ms\n", ms);
    }
    
    // Check outputs
    const std::map<std::string, MNN::Tensor*>& outputs = net->getSessionOutputAll(session);
    for (const auto& kv : outputs) {
        printf("Output %s: shape=", kv.first.c_str());
        for (int j = 0; j < kv.second->shape().size(); j++) {
            printf("%d ", kv.second->shape()[j]);
        }
        printf("\n");
        
        // Copy to host and print first few values
        MNN::Tensor* hostTensor = MNN::Tensor::createHostTensorFromDevice(kv.second, false);
        if (hostTensor) {
            void* data = hostTensor->host<void>();
            if (data) {
                half_float::half* h = (half_float::half*)data;
                printf("  First 8 values: ");
                for (int i = 0; i < 8 && i < hostTensor->elementSize(); i++) {
                    printf("%.4f ", (float)h[i]);
                }
                printf("\n");
            }
            delete hostTensor;
        }
    }
    
    net->releaseSession(session);
    MNN::Interpreter::destroy(net);
    return 0;
}