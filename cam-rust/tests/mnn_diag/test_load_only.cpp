#include <MNN/Interpreter.hpp>
#include <iostream>
using namespace MNN;
int main() {
    std::cout << "Loading...\n" << std::flush;
    auto* interp = Interpreter::createFromFile("isp_pipeline_fixed.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    std::cout << "Loaded\n" << std::flush;
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    if (!sess) { std::cerr << "session failed\n"; return 1; }
    std::cout << "Session created\n" << std::flush;
    auto* in = interp->getSessionInput(sess, nullptr);
    if (!in) { std::cerr << "no input\n"; return 1; }
    std::cout << "Input OK\n" << std::flush;
    auto inshape = in->shape();
    std::cout << "Shape: ";
    for (auto d : inshape) std::cout << d << " ";
    std::cout << "\n" << std::flush;
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    std::cout << "Done\n";
    return 0;
}
