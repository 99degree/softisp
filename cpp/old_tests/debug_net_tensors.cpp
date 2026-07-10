#include <stdio.h>
#include <vector>
#include "MNN_generated.h"

int main() {
    FILE* f = fopen("test_standard_8x8.mnn", "rb");
    fseek(f, 0, SEEK_END); size_t sz = ftell(f); fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz);
    fread(buf.data(), 1, sz, f);
    fclose(f);

    auto* net = MNN::GetNet(buf.data());
    printf("Ops: %d\n", (int)net->oplists()->size());
    printf("Tensors: %d\n", (int)net->tensorName()->size());
    
    // Print each op with its input/output tensor indices
    for (int i = 0; i < (int)net->oplists()->size(); i++) {
        auto* op = net->oplists()->Get(i);
        printf("op[%d]: name=%s type=%d in=[", i, 
               op->name() ? op->name()->c_str() : "(anon)", (int)op->type());
        if (op->inputIndexes()) {
            for (int j = 0; j < (int)op->inputIndexes()->size(); j++)
                printf("%d%c", op->inputIndexes()->Get(j), j+1 < (int)op->inputIndexes()->size() ? ',' : ']');
        } else printf("]");
        printf(" out=[");
        if (op->outputIndexes()) {
            for (int j = 0; j < (int)op->outputIndexes()->size(); j++)
                printf("%d%c", op->outputIndexes()->Get(j), j+1 < (int)op->outputIndexes()->size() ? ',' : ']');
        } else printf("]");
        printf("\n");
    }
    
    // Print tensor names
    printf("\nTensor names:\n");
    for (int i = 0; i < (int)net->tensorName()->size(); i++) {
        printf("  [%d] %s\n", i, net->tensorName()->Get(i) ? net->tensorName()->Get(i)->c_str() : "(null)");
    }
    return 0;
}
