#include <stdio.h>
#include <MNN/Interpreter.hpp>
#include "MNN_generated.h"

int main() {
    FILE* f = fopen("test_standard_8x8.mnn", "rb");
    if (!f) return 1;
    fseek(f, 0, SEEK_END);
    size_t sz = ftell(f); fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz);
    fread(buf.data(), 1, sz, f); fclose(f);
    auto* net = MNN::GetNet(buf.data());
    
    for (int i = 0; i < (int)net->oplists()->size(); i++) {
        auto* op = net->oplists()->Get(i);
        if (op->type() == MNN::OpType_Convolution || 
            op->type() == MNN::OpType_ConvolutionDepthwise) {
            auto* c = op->main_as_Convolution2D();
            auto* cm = c->common();
            printf("op[%d]: %s k=%dx%d s=%dx%d ic=%d oc=%d group=%d weights=%dB\n",
                   i, MNN::EnumNameOpType(op->type()),
                   cm->kernelX(), cm->kernelY(),
                   cm->strideX(), cm->strideY(),
                   cm->inputCount(), cm->outputCount(),
                   cm->group(),
                   (int)c->weight()->size());
        }
    }
    return 0;
}
