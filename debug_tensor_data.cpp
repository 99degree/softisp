// Debug: dump raw tensor buffer data at each stage
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/expr/Executor.hpp>
#include <stdio.h>
#include <string.h>

static void dump_tensor(const char* label, MNN::Tensor* t) {
    auto& buf = t->buffer();
    printf("%s: dims=%d [", label, (int)buf.dimensions);
    for (int i = 0; i < buf.dimensions; i++) {
        if (i > 0) printf(",");
        printf("%d", (int)buf.dim[i].extent);
    }
    printf("] bytes=%zu host=%p device=%p type=%d\n",
           t->size(), t->host<void*>(), (void*)buf.device,
           (int)t->getType().code);

    // If host data available, dump first 16 floats
    if (t->host<float>()) {
        float* data = t->host<float>();
        int count = t->elementSize();
        if (count > 16) count = 16;
        printf("  first %d floats:", count);
        for (int i = 0; i < count; i++)
            printf(" %.4f", data[i]);
        printf("\n");
    }
}

int main(int argc, const char* argv[]) {
    const char* model_file = "test_standard_8x8.mnn";
    if (argc > 1) model_file = argv[1];

    // Create interpreter
    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile(model_file));
    if (!interp) {
        printf("FAIL: can't open %s\n", model_file);
        return 1;
    }

    // Configure Vulkan backend
    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_VULKAN;
    config.numThread = 1;

    MNN::BackendConfig backendConfig;
    backendConfig.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &backendConfig;

    // Create session
    auto session = interp->createSession(config);
    if (!session) {
        printf("FAIL: can't create session\n");
        return 1;
    }

    // Get input tensor
    auto input = interp->getSessionInput(session, NULL);
    dump_tensor("Input", input);

    // Create our 8x8 Bayer pattern
    int inW = input->width();
    int inH = input->height();
    printf("Input shape: %dx%dx%d\n", input->channel(), inH, inW);

    // Fill Bayer pattern: R=100, Gr=200, Gb=200, B=50
    std::vector<float> inputData(inW * inH);
    for (int y = 0; y < inH; y++) {
        for (int x = 0; x < inW; x++) {
            if (y % 2 == 0) {
                inputData[y * inW + x] = (x % 2 == 0) ? 100.0f : 200.0f;
            } else {
                inputData[y * inW + x] = (x % 2 == 0) ? 200.0f : 50.0f;
            }
        }
    }
    auto hostInput = MNN::Tensor::createHostTensorFromDevice(input, false);
    if (!hostInput) { printf("FAIL: createHostTensorFromDevice returned null\n"); return 1; }
    hostInput->setHostData((void*)inputData.data());
    MNN::Tensor::copyFromHostTensor(hostInput.get(), input);
    dump_tensor("Input after fill", input);

    // Run session
    interp->runSession(session);
    printf("Session run OK\n");

    // Get output tensor
    auto output = interp->getSessionOutput(session, NULL);
    if (!output) {
        printf("FAIL: no output\n");
        return 1;
    }
    dump_tensor("Output", output);

    // Copy to host
    auto hostOut = MNN::Tensor::createHostTensorFromDevice(output, false);
    if (!hostOut) { printf("FAIL: createHostTensorFromDevice returned null for output\n"); return 1; }
    if (!output->copyToHostTensor(hostOut.get())) {
        printf("FAIL: copyToHostTensor\n");
    }

    dump_tensor("HostOutput", hostOut.get());
    int count = hostOut->elementSize();
    printf("Total elements: %d\n", count);

    // Print as CHW planar
    int C = hostOut->channel();
    int H = hostOut->height();
    int W = hostOut->width();
    printf("Output shape: [1,%d,%d,%d] (CxHxW)\n", C, H, W);
    float* d = hostOut->host<float>();
    if (!d) { printf("FAIL: no host data\n"); return 1; }
    for (int c = 0; c < C; c++) {
        printf("ch%d:", c);
        for (int y = 0; y < H; y++) {
            printf("\n   ");
            for (int x = 0; x < std::min(W, 8); x++) {
                int idx = c * W * H + y * W + x;
                printf(" %7.4f", d[idx]);
            }
        }
        printf("\n");
    }

    return 0;
}
