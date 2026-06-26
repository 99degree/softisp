// Debug: trace tensor data through pipeline
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/expr/Executor.hpp>
#include <stdio.h>
#include <string.h>
#include <math.h>

static void dump_tensor(const char* label, MNN::Tensor* t) {
    auto& buf = t->buffer();
    printf("%s: dims=%d [", label, (int)buf.dimensions);
    for (int i = 0; i < buf.dimensions; i++) {
        if (i > 0) printf(",");
        printf("%d", (int)buf.dim[i].extent);
    }
    printf("] bytes=%zu\n", t->size());
    if (t->host<float>()) {
        float* data = t->host<float>();
        int count = t->elementSize();
        if (count > 24) count = 24;
        printf("  first %d floats:", count);
        for (int i = 0; i < count; i++)
            printf(" %.4f", data[i]);
        printf("\n");
    }
}

int main() {
    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile("test_standard_8x8.mnn"));
    if (!interp) { printf("FAIL: open\n"); return 1; }

    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_VULKAN;
    config.numThread = 1;
    MNN::BackendConfig backendConfig;
    backendConfig.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &backendConfig;

    // Enable callback to probe intermediate tensors
    config.callback = [](const std::vector<MNN::Tensor*>& inputs,
                         const std::vector<MNN::Tensor*>& outputs,
                         const MNN::OpInfo& info) -> bool {
        printf("=== Op: %s inputs=%zu outputs=%zu ===\n",
               info.name(), inputs.size(), outputs.size());
        for (int i = 0; i < (int)inputs.size(); i++) {
            char label[64]; snprintf(label, 64, "  in[%d]", i);
            dump_tensor(label, inputs[i]);
        }
        for (int i = 0; i < (int)outputs.size(); i++) {
            char label[64]; snprintf(label, 64, "  out[%d]", i);
            dump_tensor(label, outputs[i]);
        }
        return true;
    };

    auto session = interp->createSession(config);
    if (!session) { printf("FAIL: session\n"); return 1; }

    auto input = interp->getSessionInput(session, NULL);
    int inW = 8, inH = 8;
    
    // Fill 8x8 Bayer
    float inputData[64];
    for (int y = 0; y < 8; y++)
        for (int x = 0; x < 8; x++)
            if (y%2==0) inputData[y*8+x] = (x%2==0) ? 100.0f : 200.0f;
            else inputData[y*8+x] = (x%2==0) ? 200.0f : 50.0f;

    auto hostIn = MNN::Tensor::create({1,1,8,8}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    memcpy(hostIn->host<float>(), inputData, 64*4);
    input->copyFromHostTensor(hostIn);

    interp->runSession(session);

    auto output = interp->getSessionOutput(session, NULL);
    printf("\n=== Final output ===\n");
    dump_tensor("Output", output);

    auto hostOut = MNN::Tensor::create({1,3,4,4}, halide_type_of<float>(), NULL, MNN::Tensor::CAFFE);
    output->copyToHostTensor(hostOut);
    
    float* d = hostOut->host<float>();
    printf("CHW planar output:\n");
    for (int c = 0; c < 3; c++) {
        printf("  ch%d:", c);
        for (int y = 0; y < 4; y++) {
            printf("\n   ");
            for (int x = 0; x < 4; x++)
                printf(" %6.3f", d[c*16 + y*4 + x]);
        }
        printf("\n");
    }
    return 0;
}
