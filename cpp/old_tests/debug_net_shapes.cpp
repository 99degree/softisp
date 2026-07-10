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
    printf("Tensors: %d\n", net->tensorName() ? (int)net->tensorName()->size() : 0);
    
    // Check extraTensorDescribe
    auto* descs = net->extraTensorDescribe();
    if (descs) {
        printf("extraTensorDescribe: %d entries\n", (int)descs->size());
        for (int i = 0; i < (int)descs->size(); i++) {
            auto* d = descs->Get(i);
            printf("  [%d] index=%d name=%s\n", i, d->index(), d->name() ? d->name()->c_str() : "(null)");
            if (d->blob()) {
                printf("       blob dims=%d data=%p", d->blob()->dims() ? (int)d->blob()->dims()->size() : 0, (void*)d->blob()->int32s());
                if (d->blob()->int32s() && d->blob()->int32s()->size() > 0) {
                    printf(" [");
                    for (int j = 0; j < (int)d->blob()->int32s()->size(); j++)
                        printf("%d%c", d->blob()->int32s()->Get(j), j+1 < (int)d->blob()->int32s()->size() ? ',' : ']');
                }
                printf("\n");
            }
        }
    } else {
        printf("No extraTensorDescribe\n");
    }

    // Check extraTensorDescribe via GetPointer
    auto* infos = net->extraTensorDescribe();
    if (infos) {
        printf("extraTensorDescribe (fb): %d entries\n", (int)infos->size());
    } else {
        printf("No extraTensorDescribe (fb)\n");
    }
    return 0;
}
