#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/ErrorCode.hpp>
#include <iostream>
#include <vector>
#include <chrono>

int main(int argc, char* argv[]) {
    const char* model_path = argc > 1 ? argv[1] : 
        "/data/data/com.termux/files/home/softisp/vulkan_isp/fused_isp_unpack.mnn";
    
    MNN::Interpreter* interp = MNN::Interpreter::createFromFile(model_path);
    if (!interp) {
        std::cerr << "Failed to load model" << std::endl;
        return 1;
    }
    std::cout << "Model loaded" << std::endl;
    
    // Try CPU first to verify model works
    MNN::ScheduleConfig cpuConfig;
    cpuConfig.type = MNN_FORWARD_CPU;
    cpuConfig.numThread = 4;
    
    MNN::Session* cpuSession = interp->createSession(cpuConfig);
    if (cpuSession) {
        std::cout << "CPU session created" << std::endl;
    } else {
        std::cerr << "CPU session failed" << std::endl;
    }
    
    // Try Vulkan
    MNN::ScheduleConfig vkConfig;
    vkConfig.type = MNN_FORWARD_VULKAN;
    
    MNN::Session* vkSession = interp->createSession(vkConfig);
    if (vkSession) {
        std::cout << "Vulkan session created" << std::endl;
    } else {
        std::cerr << "Vulkan session failed" << std::endl;
        delete interp;
        return 1;
    }
    
    // Get I/O tensors
    MNN::Tensor* input = interp->getSessionInput(vkSession, "input");
    MNN::Tensor* output = interp->getSessionOutput(vkSession, "output");
    
    std::cout << "Input: [" << input->batch() << "," << input->channel()
              << "," << input->height() << "," << input->width() << "]"
              << " type=" << input->getType().code << "." << input->getType().bits
              << " size=" << input->size() << std::endl;
    
    std::cout << "Output: [" << output->batch() << "," << output->channel()
              << "," << output->height() << "," << output->width() << "]"
              << " type=" << output->getType().code << "." << output->getType().bits
              << " size=" << output->size() << std::endl;
    
    // Fill input test data  
    auto hostInput = new MNN::Tensor(input, MNN::Tensor::CAFFE);
    input->copyToHostTensor(hostInput);
    int numElements = hostInput->elementSize();
    std::cout << "Host input elements: " << numElements << std::endl;
    
    if (input->getType().code == 0 && input->getType().bits == 32) {
        // INT32 - fill with 2048
        int32_t* data = hostInput->host<int32_t>();
        for (int i = 0; i < numElements; i++) data[i] = 2048;
        std::cout << "Filled INT32 input with 2048" << std::endl;
    } else if (input->getType().code == 0) {
        int16_t* data = hostInput->host<int16_t>();
        for (int i = 0; i < numElements; i++) data[i] = 2048;
        std::cout << "Filled INT16 input with 2048" << std::endl;
    } else {
        std::cout << "Unknown input type: code=" << (int)input->getType().code 
                  << " bits=" << (int)input->getType().bits << std::endl;
    }
    input->copyFromHostTensor(hostInput);
    
    // Warmup
    auto error = interp->runSession(vkSession);
    std::cout << "Warmup: " << (error == MNN::NO_ERROR ? "OK" : "FAILED") << std::endl;
    
    // Check output
    auto hostOutput = new MNN::Tensor(output, MNN::Tensor::CAFFE);
    output->copyToHostTensor(hostOutput);
    float* fdata = hostOutput->host<float>();
    int n = std::min(hostOutput->elementSize(), 100);
    float mn = 1e9f, mx = -1e9f, s = 0;
    for (int i = 0; i < n; i++) {
        float v = fdata[i];
        if (v < -1e6 || v > 1e6) std::cout << "  outlier[" << i << "]=" << v << std::endl;
        mn = std::min(mn, v);
        mx = std::max(mx, v);
        s += v;
    }
    std::cout << "Output(" << n << "): min=" << mn << " max=" << mx << " avg=" << (s/n) << std::endl;
    
    // Benchmark
    const int N = 10;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; i++) {
        interp->runSession(vkSession);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "Avg: " << us / N << " us  (" << 1000000.0 * N / us << " fps)" << std::endl;
    
    delete hostInput;
    delete hostOutput;
    delete vkSession;
    delete cpuSession;
    delete interp;
    return 0;
}
