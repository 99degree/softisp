#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <MNN/expr/Executor.hpp>
#include <stdio.h>
#include <string.h>
#include "MNN_generated.h"
#include <flatbuffers/flatbuffers.h>

int main() {
    auto interp = std::shared_ptr<MNN::Interpreter>(
        MNN::Interpreter::createFromFile("test_standard_8x8.mnn"));
    if (!interp) { printf("FAIL: open\n"); return 1; }
    
    auto net = interp->getBackend(nullptr, nullptr); // get net info
    printf("Model ops:\n");
    for (int i = 0; i < net->oplists.size(); i++) {
        auto& op = net->oplists[i];
        if (!op) { printf("  [%d] null\n", i); continue; }
        const char* typeName = MNN::EnumNameOpType(op->type);
        const char* extraType = "";
        if (op->main.Is<MNN::ExtraT>()) {
            extraType = op->main.As<MNN::ExtraT>()->type.c_str();
        }
        printf("  [%d] %s", i, typeName);
        if (*extraType) printf(" (%s)", extraType);
        printf(" inputs:[");
        for (int j = 0; j < op->inputIndexes.size(); j++)
            printf("%s%d", j?",":"", op->inputIndexes[j]);
        printf("] outputs:[");
        for (int j = 0; j < op->outputIndexes.size(); j++)
            printf("%s%d", j?",":"", op->outputIndexes[j]);
        printf("]\n");
    }
    return 0;
}
