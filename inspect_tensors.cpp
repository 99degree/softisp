#include <stdio.h>
#include <string.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <flatbuffers/flatbuffers.h>
#include "MNN_generated.h"

int main() {
    FILE* f = fopen("test_standard_8x8.mnn", "rb");
    if (!f) { printf("FAIL: can't open\n"); return 1; }
    fseek(f, 0, SEEK_END);
    size_t sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz);
    fread(buf.data(), 1, sz, f);
    fclose(f);

    auto* net = MNN::GetNet(buf.data());
    printf("Model: %zu ops, %zu tensors\n\n",
           (size_t)net->oplists()->size(), (size_t)net->tensorName()->size());

    printf("=== Tensors ===\n");
    for (int i = 0; i < (int)net->tensorName()->size(); i++) {
        printf("  t[%d] = '%s'\n", i, net->tensorName()->GetAsString(i)->c_str());
    }
    printf("\n");

    for (int i = 0; i < (int)net->oplists()->size(); i++) {
        auto* op = net->oplists()->Get(i);
        auto type = op->type();

        printf("op[%d]: %s (", i, MNN::EnumNameOpType(type));
        if (op->inputIndexes()) {
            for (int k = 0; k < (int)op->inputIndexes()->size(); k++)
                printf("%s%d", k>0?",":"", (int)op->inputIndexes()->Get(k));
        }
        printf(")->(");
        if (op->outputIndexes()) {
            for (int k = 0; k < (int)op->outputIndexes()->size(); k++)
                printf("%s%d", k>0?",":"", (int)op->outputIndexes()->Get(k));
        }
        printf(")");

        if (type == MNN::OpType_Extra) {
            auto* extra = op->main_as_Extra();
            printf(" type=\"%s\"", extra->type()->c_str());
        } else if (type == MNN::OpType_ConvertTensor) {
            auto* info = op->main_as_TensorConvertInfo();
            printf(" %s->%s",
                   MNN::EnumNameMNN_DATA_FORMAT(info->source()),
                   MNN::EnumNameMNN_DATA_FORMAT(info->dest()));
        }
        printf("\n");
    }
    return 0;
}
