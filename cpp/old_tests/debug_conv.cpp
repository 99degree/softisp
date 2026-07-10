#include <stdio.h>
#include <vector>
#include <MNN/Interpreter.hpp>
#include "MNN_generated.h"

int main() {
    FILE* f = fopen("test_standard_8x8.mnn", "rb");
    fseek(f, 0, SEEK_END); size_t sz = ftell(f); fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz);
    fread(buf.data(), 1, sz, f);
    fclose(f);

    auto* net = MNN::GetNet(buf.data());
    for (int i = 0; i < (int)net->oplists()->size(); i++) {
        auto* op = net->oplists()->Get(i);
        if (op->type() != MNN::OpType_Convolution) continue;
        auto* conv = op->main_as_Convolution2D();
        auto* c = conv->common();
        printf("op[%d]: k=%d/%d s=%d/%d in=%d out=%d g=%d w=%zu\n",
               i,
               c->kernelX(), c->kernelY(),
               c->strideX(), c->strideY(),
               c->inputCount(), c->outputCount(),
               c->group(),
               conv->weight()->size());
    }
    return 0;
}
