#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>

int main() {
    auto ip = MNN::Interpreter::createFromFile("test_standard_8x8.mnn");
    if (!ip) { printf("FAIL: Interpreter\n"); return 1; }

    MNN::ScheduleConfig sc;
    sc.type = MNN_FORWARD_CPU;
    auto sess = ip->createSession(sc);
    if (!sess) { printf("FAIL: Session\n"); return 1; }

    auto in = ip->getSessionInput(sess, NULL);
    ip->resizeTensor(in, {1, 1, 8, 8});
    ip->resizeSession(sess);

    // Create Bayer pattern
    auto hostIn = MNN::Tensor::create({1, 1, 8, 8}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    float* hd = hostIn->host<float>();
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            if (y % 2 == 0 && x % 2 == 0) hd[y*8+x] = 100.0f;   // R
            else if (y % 2 == 0 && x % 2 == 1) hd[y*8+x] = 200.0f; // Gr
            else if (y % 2 == 1 && x % 2 == 0) hd[y*8+x] = 200.0f; // Gb
            else hd[y*8+x] = 50.0f;  // B
        }
    }

    in->copyFromHostTensor(hostIn);
    ip->runSession(sess);

    auto out = ip->getSessionOutput(sess, NULL);
    auto hostOut = MNN::Tensor::create({1, 3, 4, 4}, out->getType(), NULL, MNN::Tensor::CAFFE);
    out->copyToHostTensor(hostOut);

    float* od = hostOut->host<float>();
    printf("CPU Output:\n");
    for (int c = 0; c < 3; c++) {
        printf("  ch%d:", c);
        for (int y = 0; y < 4; y++) {
            printf("\n   ");
            for (int x = 0; x < 4; x++) {
                printf(" %6.3f", od[c*16 + y*4 + x]);
            }
        }
        printf("\n");
    }
    int nonZero = 0;
    for (int i = 0; i < 3*4*4; i++)
        if (fabs(od[i]) > 0.001f) nonZero++;
    printf("\nNon-zero: %d/48 %s\n", nonZero, nonZero > 0 ? "✓" : "✗");
    delete ip;
    return nonZero > 0 ? 0 : 1;
}
