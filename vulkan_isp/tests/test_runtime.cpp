#include <MNN/Interpreter.hpp>
#include <chrono>
int main() {
    auto net = MNN::Interpreter::createFromFile("isp_heavy.mnn");
    auto session = net->createSession();
    float wb_gains[] = {2.1f, 1.0f, 1.0f, 1.8f};
    auto* tensor = net->getSessionInput(session, "wb_gains_tensor");
    tensor->writeMap<float>()[0] = wb_gains[0]; // Update R gain
    auto start = std::chrono::high_resolution_clock::now();
    net->runSession(session);
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double>(end-start).count() * 1000.0;
    printf("Runtime update + inference: %.2f ms\n", ms);
}
