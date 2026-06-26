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

    // Test 1: Patterned Bayer - each 2x2 quad varies
    int W=8, H=8;
    std::vector<float> hostData(W*H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int qx = x/2, qy = y/2;  // which 2x2 quad
            int px = x%2, py = y%2;  // position within quad
            if (py==0 && px==0) hostData[y*W+x] = 100.0f + qx + qy*4; // R
            else if (py==0 && px==1) hostData[y*W+x] = 200.0f + qx + qy*4; // Gr
            else if (py==1 && px==0) hostData[y*W+x] = 200.0f + qx + qy*4; // Gb
            else hostData[y*W+x] = 50.0f + qx + qy*4; // B
        }
    
    printf("Input Bayer:\n");
    for (int y = 0; y < H; y++) {
        printf("  row%d:", y);
        for (int x = 0; x < W; x++)
            printf(" %5.0f", hostData[y*W+x]);
        printf("\n");
    }
    
    auto hostIn = MNN::Tensor::create({1,1,8,8}, halide_type_of<float>(), 
                                       hostData.data(), MNN::Tensor::CAFFE);
    auto input = interp->getSessionInput(session, NULL);
    input->copyFromHostTensor(hostIn);
    interp->runSession(session);

    auto output = interp->getSessionOutput(session, NULL);
    auto hostOut = MNN::Tensor::create({1,3,4,4}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    output->copyToHostTensor(hostOut);
    
    float* d = hostOut->host<float>();
    printf("\nOutput RGB (CHW):\n");
    for (int y = 0; y < 4; y++) {
        printf("  pixel row%d:", y);
        for (int x = 0; x < 4; x++) {
            int idx = y*4+x;
            printf(" [R=%.3f G=%.3f B=%.3f]", d[0*16+idx], d[1*16+idx], d[2*16+idx]);
        }
        printf("\n");
    }
    
    // Verify each quad independently
    printf("\nVerification (expected vs actual R for each quad):\n");
    for (int qy = 0; qy < 4; qy++) {
        for (int qx = 0; qx < 4; qx++) {
            float R_raw = 100.0f + qx + qy*4;
            float R_norm = R_raw / 1023.0f;
            // srgb_gamma
            float R_gamma = (R_norm <= 0.0031308f) ? 
                R_norm * 12.92f : 
                1.055f * powf(R_norm, 1.0f/2.4f) - 0.055f;
            
            float G_raw = 200.0f + qx + qy*4;
            float G_norm = G_raw / 1023.0f;
            float G_gamma = (G_norm <= 0.0031308f) ? 
                G_norm * 12.92f : 
                1.055f * powf(G_norm, 1.0f/2.4f) - 0.055f;
            
            float B_raw = 50.0f + qx + qy*4;
            float B_norm = B_raw / 1023.0f;
            float B_gamma = (B_norm <= 0.0031308f) ? 
                B_norm * 12.92f : 
                1.055f * powf(B_norm, 1.0f/2.4f) - 0.055f;
            
            float R_actual = d[0*16 + qy*4 + qx];
            float G_actual = d[1*16 + qy*4 + qx];
            float B_actual = d[2*16 + qy*4 + qx];
            
            printf("  Q(%d,%d): R=%.3f(exp=%.3f) G=%.3f(exp=%.3f) B=%.3f(exp=%.3f) %s\n",
                   qx, qy,
                   R_actual, R_gamma, G_actual, G_gamma, B_actual, B_gamma,
                   (fabs(R_actual-R_gamma)<0.01 && fabs(G_actual-G_gamma)<0.01 && fabs(B_actual-B_gamma)<0.01) ? "✓" : "✗");
        }
    }
    return 0;
}
