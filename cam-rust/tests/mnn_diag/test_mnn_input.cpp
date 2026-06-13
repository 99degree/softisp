#include <MNN/Interpreter.hpp>
#include <MNN/MNNDefine.h>
#include <MNN/Tensor.hpp>
#include <cstdio>
#include <vector>

int main(int argc, const char* argv[]) {
    const char* model_path = argc > 1 ? argv[1] : "/sdcard/Download/reference.mnn";
    
    printf("Loading model: %s\n", model_path);
    
    auto interpreter = MNN::Interpreter::createFromFile(model_path);
    if (!interpreter) {
        printf("Failed to create interpreter!\n");
        return 1;
    }
    
    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_CPU;
    config.numThread = 4;
    
    auto session = interpreter->createSession(config);
    if (!session) {
        printf("Failed to create session!\n");
        return 1;
    }
    
    // Try getting various tensors
    const char* names[] = {
        "RawInputBlock/frame",
        "NormalizeBlock/sensor_max",
        "BlcBlock/blc",
        "DisplayBlock/frame",
        "unified_stats/frame",
        "calibration/frame",
        nullptr
    };
    
    for (int i = 0; names[i]; i++) {
        auto tensor = interpreter->getSessionInput(session, names[i]);
        printf("getInput(\"%s\") = %s\n", names[i], tensor ? "FOUND" : "null");
    }
    
    // Also try getSessionOutput
    for (int i = 0; names[i]; i++) {
        auto tensor = interpreter->getSessionOutput(session, names[i]);
        printf("getOutput(\"%s\") = %s\n", names[i], tensor ? "FOUND" : "null");
    }
    
    interpreter->releaseModel();
    delete interpreter;
    printf("Done.\n");
    return 0;
}
