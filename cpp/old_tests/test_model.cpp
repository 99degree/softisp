#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <map>
#include <stdio.h>

int main(int argc, char* argv[]) {
    if (argc < 2) return 1;
    MNN::Interpreter* net = MNN::Interpreter::createFromFile(argv[1]);
    if (!net) { fprintf(stderr, "FAIL: createFromFile\n"); return 1; }
    
    MNN::ScheduleConfig config;
    config.numThread = 4;
    config.type = (MNNForwardType)7;  // Vulkan
    
    MNN::Session* session = net->createSession(config);
    if (!session) { fprintf(stderr, "FAIL: createSession\n"); return 1; }
    
    MNN::Tensor* input = net->getSessionInput(session, NULL);
    if (!input) { fprintf(stderr, "FAIL: getSessionInput\n"); return 1; }
    
    printf("Input tensor: shape=");
    for (int i = 0; i < input->shape().size(); i++) {
        printf("%d ", input->shape()[i]);
    }
    printf("\n");
    
    // Check output tensors
    const std::map<std::string, MNN::Tensor*>& outputs = net->getSessionOutputAll(session);
    for (const auto& kv : outputs) {
        printf("Output %s: shape=", kv.first.c_str());
        for (int j = 0; j < kv.second->shape().size(); j++) {
            printf("%d ", kv.second->shape()[j]);
        }
        printf("\n");
    }
    
    net->releaseSession(session);
    MNN::Interpreter::destroy(net);
    return 0;
}