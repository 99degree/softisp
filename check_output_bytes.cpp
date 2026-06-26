#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <stdio.h>
#include <string.h>

int main() {
    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile("test_standard_8x8.mnn"));
    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_VULKAN;
    MNN::BackendConfig bc;
    bc.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &bc;
    auto session = interp->createSession(config);

    // Fill input (simple: all 1.0 for debugging)
    auto input = interp->getSessionInput(session, NULL);
    std::vector<float> inData(64, 1.0f);
    auto hostIn = MNN::Tensor::create({1,1,8,8}, halide_type_of<float>(), inData.data(), MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);
    interp->runSession(session);

    auto output = interp->getSessionOutput(session, NULL);
    printf("Output shape: [%d,%d,%d,%d]\n",
           output->batch(), output->channel(), output->height(), output->width());
    printf("Output bytes: %zu\n", output->size());
    printf("Output elementSize: %d\n", output->elementSize());
    
    // Read back
    auto hostOut = MNN::Tensor::create({1,3,4,4}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    output->copyToHostTensor(hostOut);
    
    float* d = hostOut->host<float>();
    printf("All output floats (48 values):\n");
    for (int i = 0; i < 48; i++) printf(" %.6f", d[i]);
    printf("\n");
    
    // Check if the output is just reordered input  
    printf("As 12x4 grid (each row = one 4-wide pixel with 12 floats):\n");
    for (int y = 0; y < 4; y++) {
        printf(" row%d:", y);
        for (int x = 0; x < 4; x++) {
            int idx = y * 4 + x;
            printf(" [");
            for (int c = 0; c < 3; c++)
                printf("%.4f%s", d[c*16 + idx], c<2?",":"");
            printf("]");
        }
        printf("\n");
    }
    return 0;
}
