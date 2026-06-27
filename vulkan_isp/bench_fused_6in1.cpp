// Benchmark fused_6in1 pipeline (1 dispatch) on Vulkan
#include <MNN/Interpreter.hpp>
#include <MNN/MNNForwardType.h>
#include <MNN/Tensor.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

int main(int argc, char** argv) {
    const char* model_path = argc > 1 ? argv[1] : getenv("HOME");
    char path_buf[512];
    if (argc <= 1) {
        snprintf(path_buf, sizeof(path_buf), "%s/tmp/test_f6.mnn", model_path);
        model_path = path_buf;
    }

    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile(model_path));
    if (!interp) { fprintf(stderr, "Failed to load model\n"); return 1; }

    MNN::ScheduleConfig sconfig;
    sconfig.type = MNN_FORWARD_VULKAN;
    sconfig.numThread = 4;
    MNN::BackendConfig bconfig;
    bconfig.precision = MNN::BackendConfig::Precision_High;
    sconfig.backendConfig = &bconfig;

    auto session = interp->createSession(sconfig);
    if (!session) { fprintf(stderr, "Failed to create session\n"); return 1; }

    auto input = interp->getSessionInput(session, nullptr);
    auto output = interp->getSessionOutput(session, nullptr);
    fprintf(stderr, "Input: %dx%dx%d  Output: %dx%dx%d\n",
            input->width(), input->height(), input->channel(),
            output->width(), output->height(), output->channel());

    // Fill input with Bayer pattern R=100, Gr=200, Gb=200, B=50
    auto host_in = new MNN::Tensor(input, MNN::Tensor::CAFFE);
    auto ptr = host_in->host<float>();
    for (int y = 0; y < input->height(); y++) {
        for (int x = 0; x < input->width(); x++) {
            int idx = y * input->width() + x;
            if (y % 2 == 0)
                ptr[idx] = (x % 2 == 0) ? 100.0f : 200.0f;
            else
                ptr[idx] = (x % 2 == 0) ? 200.0f : 50.0f;
        }
    }
    input->copyFromHostTensor(host_in);
    delete host_in;

    // Warmup
    for (int i = 0; i < 5; i++) interp->runSession(session);

    // Benchmark
    int N = 50;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; i++) interp->runSession(session);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / N;
    fprintf(stderr, "Avg: %.2f ms/frame  (%.1f FPS)\n", ms, 1000.0 / ms);

    // Read output
    auto host_out = new MNN::Tensor(output, MNN::Tensor::CAFFE);
    output->copyToHostTensor(host_out);
    int W = output->width();
    int H = output->height();
    int plane = W * H;
    float r = host_out->host<float>()[0];
    float g = host_out->host<float>()[plane];
    float b = host_out->host<float>()[2 * plane];
    fprintf(stderr, "R=%.4f G=%.4f B=%.4f\n", r, g, b);

    // Verify against expected values from e2e test
    bool ok = fabsf(r - 0.3454f) < 0.02f && fabsf(g - 0.4794f) < 0.02f && fabsf(b - 0.2449f) < 0.02f;
    fprintf(stderr, "%s\n", ok ? "PASS" : "FAIL (expected R~0.345 G~0.479 B~0.245)");

    delete host_out;
    return ok ? 0 : 1;
}
