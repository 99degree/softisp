#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <MNN/MNNDefine.h>
#include "MNN_generated.h"
#include "TensorflowOp_generated.h"

static const char* opName(int t) {
    static char buf[32];
    switch (t) {
        case 0: return "AbsVal"; case 1: return "QAdd"; case 2: return "ArgMax";
        case 7: return "BinaryOp"; case 9: return "Cast"; case 10: return "Concat";
        case 11: return "Const"; case 12: return "Conv"; case 13: return "ConvDW";
        case 17: return "Deconv"; case 18: return "DeconvDW";
        case 22: return "Eltwise"; case 23: return "ELU"; case 25: return "Exp";
        case 26: return "ExpandDims"; case 27: return "Fill"; case 28: return "Flatten";
        case 30: return "Gather"; case 31: return "GatherV2";
        case 39: return "MatMul"; case 47: return "Pool"; case 49: return "PReLU";
        case 65: return "Range"; case 68: return "Reduce";
        case 69: return "ReLU"; case 70: return "ReLU6"; case 73: return "Reshape";
        case 74: return "Resize"; case 77: return "Scale"; case 78: return "Selu";
        case 80: return "Sigmoid"; case 79: return "Seq2Out";
        case 81: return "Softmax"; case 84: return "Slice"; case 85: return "SpaceToBatch";
        case 86: return "StridedSlice"; case 91: return "StridedSlice2";
        case 92: return "Transpose"; case 100: return "Unary";
        case 112: return "Squeeze"; case 113: return "Unsqueeze";
        case 120: return "BroadTo"; case 136: return "Extra";
        case 144: return "LayerNorm"; case 146: return "GroupNorm";
        case 200: return "InstNorm";
        default:
            snprintf(buf, sizeof(buf), "Op(%d)", t);
            return buf;
    }
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        std::ifstream f(argv[i], std::ios::binary | std::ios::ate);
        if (!f.is_open()) { fprintf(stderr, "Cannot open %s\n", argv[i]); continue; }
        size_t sz = f.tellg();
        f.seekg(0);
        std::vector<uint8_t> buf(sz);
        f.read((char*)buf.data(), sz);

        flatbuffers::Verifier v(buf.data(), buf.size());
        if (!MNN::VerifyNetBuffer(v)) { fprintf(stderr, "Verify fail: %s\n", argv[i]); continue; }
        auto net = MNN::UnPackNet(buf.data());
        if (!net) { fprintf(stderr, "Unpack fail: %s\n", argv[i]); continue; }

        printf("=== %s (%zu ops) ===\n", argv[i], net->oplists.size());
        for (int j = 0; j < (int)net->oplists.size(); j++) {
            auto& op = net->oplists[j];
            if (!op) { printf("  [%2d] NULL\n", j); continue; }
            printf("  [%2d] %-12s", j, opName((int)op->type));
            if (op->name.size()) printf("  \"%s\"", op->name.c_str());
            printf("  in=[");
            for (int k = 0; k < (int)op->inputIndexes.size(); k++) {
                if (k) printf(","); printf("%d", op->inputIndexes[k]);
            }
            printf("] out=[");
            for (int k = 0; k < (int)op->outputIndexes.size(); k++) {
                if (k) printf(","); printf("%d", op->outputIndexes[k]);
            }
            printf("]\n");
        }
        printf("\n");
    }
    return 0;
}
