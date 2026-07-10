#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>

int main() {
    // Create interpreter from the ONNX-converted MNN model
    auto ip = MNN::Interpreter::createFromFile("test_standard_8x8.mnn");
    if (!ip) { printf("FAIL: Interpreter\n"); return 1; }

    MNN::ScheduleConfig sc;
    sc.type = MNN_FORWARD_VULKAN;
    sc.backupType = MNN_FORWARD_CPU;
    auto sess = ip->createSession(sc);
    if (!sess) { printf("FAIL: Session\n"); return 1; }

    // Get input tensor
    auto in = ip->getSessionInput(sess, NULL);
    if (!in) { printf("FAIL: Input tensor\n"); return 1; }

    // Resize for 8×8 Bayer input
    ip->resizeTensor(in, {1, 1, 8, 8});
    ip->resizeSession(sess);

    // Create a simple Bayer pattern (RG/GB)
    // 8×8 Bayer: R at (0,0), G at (0,1), G at (1,0), B at (1,1) pattern
    auto hostIn = MNN::Tensor::create({1, 1, 8, 8}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    float* hd = hostIn->host<float>();
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            // Bayer pattern: R at (0,0), Gr at (0,1), Gb at (1,0), B at (1,1)
            if (y % 2 == 0 && x % 2 == 0) hd[y*8+x] = 100.0f;   // R
            else if (y % 2 == 0 && x % 2 == 1) hd[y*8+x] = 200.0f; // Gr
            else if (y % 2 == 1 && x % 2 == 0) hd[y*8+x] = 200.0f; // Gb
            else hd[y*8+x] = 50.0f;  // B
        }
    }

    in->copyFromHostTensor(hostIn);
    ip->runSession(sess);

    // Get output
    auto out = ip->getSessionOutput(sess, NULL);
    if (!out) { printf("FAIL: Output\n"); return 1; }

    auto hostOut = MNN::Tensor::create({1, 3, 4, 4}, out->getType(), NULL, MNN::Tensor::CAFFE);
    if (!hostOut) { printf("FAIL: hostOut alloc\n"); return 1; }
    out->copyToHostTensor(hostOut);

    float* od = hostOut->host<float>();
    printf("Output (4×4×3 CHW planar):\n");
    for (int c = 0; c < 3; c++) {
        printf("  ch%d:\n", c);
        for (int y = 0; y < 4; y++) {
            printf("   ");
            for (int x = 0; x < 4; x++) {
                printf(" %6.3f", od[c*16 + y*4 + x]);
            }
            printf("\n");
        }
    }

    // Count non-zero pixels
    int nonZero = 0;
    for (int i = 0; i < 3*4*4; i++)
        if (fabs(od[i]) > 0.001f) nonZero++;
    printf("\nNon-zero: %d/48 %s\n", nonZero,
           nonZero > 0 ? "✓" : "✗");

    delete ip;
    return nonZero > 0 ? 0 : 1;
}
