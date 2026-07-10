#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/expr/Executor.hpp>
#include <stdio.h>
#include <string.h>
#include "TensorUtils.hpp"  // for dimensionFormat

int main() {
    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile("test_standard_8x8.mnn"));
    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_VULKAN;
    config.numThread = 1;
    MNN::BackendConfig bc;
    bc.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &bc;
    auto session = interp->createSession(config);
    if (!session) { printf("FAIL: session\n"); return 1; }

    auto input = interp->getSessionInput(session, NULL);
    printf("Input shape: [%d,%d,%d,%d]\n",
           input->batch(), input->channel(),
           input->height(), input->width());
    printf("Input elementSize: %d\n", input->elementSize());
    printf("Input bytes: %zu\n", input->size());
    printf("Input dimensionFormat: %d (CAFFE/NCHW=%d, NC4HW4=%d)\n",
           (int)MNN::TensorUtils::getDescribe(input)->dimensionFormat,
           (int)MNN::MNN_DATA_FORMAT_NCHW,
           (int)MNN::MNN_DATA_FORMAT_NC4HW4);

    // Fill input and copy
    float inData[64];
    for (int y=0; y<8; y++)
        for (int x=0; x<8; x++)
            if (y%2==0) inData[y*8+x] = (x%2==0) ? 100.0f : 200.0f;
            else inData[y*8+x] = (x%2==0) ? 200.0f : 50.0f;
    auto hostIn = MNN::Tensor::create({1,1,8,8}, halide_type_of<float>(), inData, MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);
    
    // Check format after copy
    printf("After copy:\n");
    printf("  dimensionFormat: %d\n",
           (int)MNN::TensorUtils::getDescribe(input)->dimensionFormat);

    // Also check the internal buffer offset
    printf("  buffer offset: %d\n",
           (int)MNN::TensorUtils::getDescribeOrigin(input)->offset);
    
    return 0;
}
