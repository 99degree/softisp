#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;
int main() {
    std::cout << "Start\n" << std::flush;
    auto* interp = Interpreter::createFromFile("test_final.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    std::cout << "Loaded\n" << std::flush;
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed\n"; return 1; }
    std::cout << "Session OK\n" << std::flush;
    auto* in = interp->getSessionInput(sess, nullptr);
    std::cout << "Input OK: " << (void*)in << "\n" << std::flush;
    auto inType = in->getType();
    std::cout << "Type code=" << (int)inType.code << "\n" << std::flush;
    interp->resizeTensor(in, {1, 1, 4, 4});
    std::cout << "Resized\n" << std::flush;
    interp->resizeSession(sess);
    std::cout << "Session resized\n" << std::flush;
    // Create small test
    auto* hostIn = Tensor::create({1, 1, 4, 4}, inType, nullptr, Tensor::CAFFE);
    std::cout << "Host created: " << (void*)hostIn << "\n" << std::flush;
    if (inType.code == 2) {
        float* d = (float*)hostIn->host<void>();
        for (int i = 0; i < 16; i++) d[i] = (float)i;
    } else {
        int32_t* d = (int32_t*)hostIn->host<void>();
        for (int i = 0; i < 16; i++) d[i] = i;
    }
    in->copyFromHostTensor(hostIn);
    std::cout << "Copied input\n" << std::flush;
    interp->runSession(sess);
    std::cout << "Ran session\n" << std::flush;
    auto* out = interp->getSessionOutput(sess, nullptr);
    if (out) std::cout << "Output OK\n" << std::flush;
    Tensor::destroy(hostIn);
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
