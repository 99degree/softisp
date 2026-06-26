#include <stdio.h>
#include <vector>
#include "MNN_generated.h"
#include "CaffeOp_generated.h"
int main() {
    FILE* f = fopen("test_4k_fhd.mnn", "rb");
    fseek(f, 0, SEEK_END); size_t sz = ftell(f); fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz); fread(buf.data(), 1, sz, f); fclose(f);
    auto* net = MNN::GetNet(buf.data());
    for (int i = 0; i < (int)net->oplists()->size(); i++) {
        auto* op = net->oplists()->Get(i);
        if (op->type() == MNN::OpType_Input) {
            auto* inp = op->main_as_Input();
            if (inp && inp->dims()) {
                printf("Input dims: [");
                for (int j = 0; j < (int)inp->dims()->size(); j++)
                    printf("%d%c", inp->dims()->Get(j), j+1 < (int)inp->dims()->size() ? ',' : ']');
                printf("\n");
            }
        }
    }
    return 0;
}
