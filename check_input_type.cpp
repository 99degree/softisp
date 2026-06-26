#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/expr/Executor.hpp>
#include <stdio.h>

int main() {
    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile("test_standard_8x8.mnn"));
    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_VULKAN;
    config.numThread = 1;
    MNN::BackendConfig backendConfig;
    backendConfig.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &backendConfig;
    auto session = interp->createSession(config);

    auto input = interp->getSessionInput(session, NULL);
    auto& buf = input->buffer();
    printf("Input tensor:\n");
    printf("  dims=%d [", (int)buf.dimensions);
    for (int i=0; i<buf.dimensions; i++)
        printf("%s%d", i>0?",":"", (int)buf.dim[i].extent);
    printf("]\n");
    printf("  type: code=%d bits=%d\n", buf.type.code, buf.type.bits);
    printf("  size: %zu bytes\n", input->size());
    printf("  host: %p\n", input->host<void>());
    
    // Check format via TensorUtils
    printf("  shape: [%d,%d,%d,%d]\n",
           input->batch(), input->channel(),
           input->height(), input->width());

    // Fill input
    float inData[64];
    for (int y=0; y<8; y++)
        for (int x=0; x<8; x++)
            if (y%2==0) inData[y*8+x] = (x%2==0) ? 100.0f : 200.0f;
            else inData[y*8+x] = (x%2==0) ? 200.0f : 50.0f;

    auto hostIn = MNN::Tensor::create({1,1,8,8}, halide_type_of<float>(), inData, MNN::Tensor::CAFFE);
    printf("copyFromHost: %d\n", input->copyFromHostTensor(hostIn));
    
    // Check buffer after copy
    printf("After copy:\n");
    printf("  device: %p\n", (void*)buf.device);
    printf("  size: %zu\n", input->size());
    
    interp->runSession(session);
    
    auto output = interp->getSessionOutput(session, NULL);
    auto hostOut = MNN::Tensor::create({1,3,4,4}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    output->copyToHostTensor(hostOut);
    
    float* d = hostOut->host<float>();
    printf("Output CHW:\n");
    for (int c=0; c<3; c++) {
        printf(" ch%d:", c);
        for (int y=0; y<4; y++) {
            printf("\n  ");
            for (int x=0; x<4; x++) printf(" %.4f", d[c*16+y*4+x]);
        }
        printf("\n");
    }
    return 0;
}
