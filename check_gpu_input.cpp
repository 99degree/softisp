#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <stdio.h>
#include <string.h>

int main() {
    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile("test_standard_8x8.mnn"));
    if (!interp) { printf("FAIL: open\n"); return 1; }

    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_VULKAN;
    MNN::BackendConfig bc;
    bc.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &bc;
    auto session = interp->createSession(config);
    if (!session) { printf("FAIL: session\n"); return 1; }

    auto input = interp->getSessionInput(session, NULL);
    printf("Input dims: %d %d %d %d\n",
           input->batch(), input->channel(),
           input->height(), input->width());
    printf("Input bytes: %zu\n", input->size());
    printf("Input elementSize: %d\n", input->elementSize());

    // Fill Bayer
    int W = input->width(), H = input->height();
    std::vector<float> hostData(W * H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (y%2==0) hostData[y*W+x] = (x%2==0) ? 100.0f : 200.0f;
            else hostData[y*W+x] = (x%2==0) ? 200.0f : 50.0f;

    printf("Host data first 16: ");
    for (int i = 0; i < 16; i++) printf("%.0f ", hostData[i]);
    printf("\n");

    // Copy to device
    auto hostIn = MNN::Tensor::create({1,1,8,8}, halide_type_of<float>(), 
                                       hostData.data(), MNN::Tensor::CAFFE);
    printf("copyFromHost: %d\n", input->copyFromHostTensor(hostIn));

    // Read back the GPU buffer to see what's actually stored
    auto hostCheck = MNN::Tensor::createHostTensorFromDevice(input, true);
    if (hostCheck) {
        printf("Readback data first 32: ");
        float* d = hostCheck->host<float>();
        for (int i = 0; i < 32; i++) printf("%.0f ", d[i]);
        printf("\n");
        printf("Readback format: %s\n", 
               hostCheck->getDimensionType() == MNN::Tensor::CAFFE ? "CAFFE" :
               hostCheck->getDimensionType() == MNN::Tensor::TENSORFLOW ? "TFLOW" :
               "NC4HW4");
    }

    // Now run the model and check
    interp->runSession(session);
    
    auto output = interp->getSessionOutput(session, NULL);
    auto hostOut = MNN::Tensor::create({1,3,4,4}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    output->copyToHostTensor(hostOut);
    
    float* d = hostOut->host<float>();
    printf("Output (CHW):\n");
    for (int c = 0; c < 3; c++) {
        printf(" ch%d:", c);
        for (int y = 0; y < 4; y++) {
            printf("\n  ");
            for (int x = 0; x < 4; x++) printf(" %.4f", d[c*16+y*4+x]);
        }
        printf("\n");
    }
    return 0;
}
