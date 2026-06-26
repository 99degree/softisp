#include <stdio.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <flatbuffers/flatbuffers.h>
#include "MNN_generated.h"

int main() {
    // Read the model file
    FILE* f = fopen("test_standard_8x8.mnn", "rb");
    if (!f) { printf("FAIL: can't open\n"); return 1; }
    fseek(f, 0, SEEK_END);
    size_t sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz);
    fread(buf.data(), 1, sz, f);
    fclose(f);

    // Parse flatbuffer
    auto* net = MNN::GetNet(buf.data());
    printf("Model: %zu ops, %zu tensors\n\n",
           net->oplists()->size(), net->tensorName()->size());

    for (int i = 0; i < (int)net->oplists()->size(); i++) {
        auto* op = net->oplists()->Get(i);
        auto type = op->type();
        auto typeName = MNN::EnumNameOpType(type);
        printf("op[%d]: %s", i, typeName);

        if (type == MNN::OpType_Extra) {
            auto* extra = op->main_as_Extra();
            printf(" type=\"%s\"", extra->type()->c_str());
            // Print attributes
            if (extra->attr()) {
                for (int a = 0; a < (int)extra->attr()->size(); a++) {
                    auto* attr = extra->attr()->Get(a);
                    if (attr->key()->str() == "spirv") {
                        printf(" [SPIR-V:%dB]", (int)attr->tensor()->int8s()->size());
                    } else if (attr->key()->str() == "optimized_dispatch") {
                        printf(" [opt=1]");
                    } else if (attr->key()->str() == "group_size") {
                        auto* d = attr->tensor()->int32s();
                        printf(" [GS=%dx%d]", d->Get(0), d->Get(1));
                    } else if (attr->key()->str() == "const") {
                        auto* f = attr->tensor()->float32s();
                        if (f && f->size() > 0) {
                            printf(" [C:%.1f", f->Get(0));
                            for (int k = 1; k < std::min(4, (int)f->size()); k++)
                                printf(",%.1f", f->Get(k));
                            printf("]");
                        }
                    }
                }
            }
        } else if (type == MNN::OpType_Convolution) {
            auto* conv = op->main_as_Convolution2D();
            if (conv && conv->common()) {
                auto& c = *conv->common();
                printf(" k=%dx%d s=%d c=%d->%d",
                       c.kernelX(), c.kernelY(),
                       c.strideX(), c.inputCount(), c.outputCount());
            }
        } else if (type == MNN::OpType_Scale) {
            auto* s = op->main_as_Scale();
            if (s) printf(" ch=%d", s->channels());
        } else if (type == MNN::OpType_BinaryOp) {
            auto* b = op->main_as_BinaryOp();
            if (b) printf(" op=%d", b->opType());
        } else if (type == MNN::OpType_Pooling) {
            auto* p = op->main_as_Pool();
            if (p) printf(" k=%dx%d type=%d", p->kernelX(), p->kernelY(), p->type());
        } else if (type == MNN::OpType_Cast) {
            // cast
        } else if (type == MNN::OpType_ReLU6 || type == MNN::OpType_ReLU) {
            // relu
        }
        printf("\n");
    }
    return 0;
}
