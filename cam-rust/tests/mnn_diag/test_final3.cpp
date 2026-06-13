#include <MNN/Interpreter.hpp>
#include <iostream>
using namespace MNN;
int main() {
    std::cout << "1\n" << std::flush;
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    if (!interp) { std::cerr << "load failed\n"; return 1; }
    std::cout << "2\n" << std::flush;
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    std::cout << "3 sess=" << (void*)sess << "\n" << std::flush;
    if (!sess) { std::cerr << "session failed\n"; return 1; }
    auto* in = interp->getSessionInput(sess, nullptr);
    std::cout << "4 in=" << (void*)in << "\n" << std::flush;
    if (!in) return 1;
    auto s = in->shape();
    std::cout << "5 shape: "; for (auto d : s) std::cout << d << " "; std::cout << "\n" << std::flush;
    std::cout << "6 done\n";
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
