#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <stdio.h>
#include <string.h>
#include <math.h>

int main() {
    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile("test_standard_8x8_new.mnn"));
    if (!interp) { printf("FAIL: open\n"); return 1; }

    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_VULKAN;
    MNN::BackendConfig bc;
    bc.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &bc;
    auto session = interp->createSession(config);
    if (!session) { printf("FAIL: session\n"); return 1; }

    // Fill Bayer pattern
    auto input = interp->getSessionInput(session, NULL);
    int W = input->width(), H = input->height();
    std::vector<float> hostData(W * H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (y%2==0) hostData[y*W+x] = (x%2==0) ? 100.0f : 200.0f;
            else hostData[y*W+x] = (x%2==0) ? 200.0f : 50.0f;
    
    auto hostIn = MNN::Tensor::create({1,1,8,8}, halide_type_of<float>(), 
                                       hostData.data(), MNN::Tensor::CAFFE);
    input->copyFromHostTensor(hostIn);
    interp->runSession(session);

    auto output = interp->getSessionOutput(session, NULL);
    auto hostOut = MNN::Tensor::create({1,3,4,4}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    output->copyToHostTensor(hostOut);
    
    float* d = hostOut->host<float>();
    printf("Output (CHW):\n");
    for (int c = 0; c < 3; c++) {
        printf(" ch%d:", c);
        float minv = 1e9, maxv = -1e9, mean = 0;
        for (int y = 0; y < 4; y++) {
            printf("\n  ");
            for (int x = 0; x < 4; x++) {
                float v = d[c*16+y*4+x];
                printf(" %.4f", v);
                minv = fmin(minv, v); maxv = fmax(maxv, v); mean += v;
            }
        }
        mean /= 16;
        printf("\n  min=%.4f max=%.4f mean=%.4f\n", minv, maxv, mean);
    }
    
    // Expected: R~0.345, G~0.466, B~0.259 (with gamma)
    printf("\nExpected (after gamma): R≈0.345, G≈0.466, B≈0.259\n");
    float r_mean=0, g_mean=0, b_mean=0;
    for (int i = 0; i < 16; i++) {
        r_mean += d[0*16+i];
        g_mean += d[1*16+i];
        b_mean += d[2*16+i];
    }
    r_mean /= 16; g_mean /= 16; b_mean /= 16;
    printf("Got: R=%.4f, G=%.4f, B=%.4f\n", r_mean, g_mean, b_mean);
    
    return 0;
}
