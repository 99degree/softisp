#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <cstdio>
using namespace MNN;
extern "C" int main() {
    printf("1\n");
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    printf("2 interp=%p\n", interp);
    if (!interp) { printf("load fail\n"); return 1; }
    // Just load, create session, get input - no data operations
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    printf("3 sess=%p\n", sess);
    if (!sess) return 1;
    auto* in = interp->getSessionInput(sess, nullptr);
    printf("4 in=%p\n", in);
    if (!in) return 1;
    auto s = in->shape();
    printf("5 shape empty=%d\n", (int)s.empty());
    for (auto d : s) printf(" dim=%d", d);
    printf("\n");
    auto t = in->getType();
    printf("6 type code=%d bits=%d\n", t.code, t.bits);
    
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    printf("7 Done\n");
    return 0;
}
