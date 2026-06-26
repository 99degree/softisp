#include <stdio.h>

int main() {
    float x[5] = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
    printf("sizeof(int32_t)=%zu, sizeof(float)=%zu\n", sizeof(int32_t), sizeof(float));
    printf("sizeof(int[5])=%zu, sizeof(float[5])=%zu\n", sizeof(int[5]), sizeof(float[5]));

    // Test with byte reassembly
    void* raw = malloc(sizeof(int32_t) * 5 + sizeof(float) * 8);
    int32_t* intp = (int32_t*)raw;
    float* fp = (float*)((uint8_t*)raw + sizeof(int32_t) * 5);
    // Write raw bits using int32_t (matches flatbuffers int32s byte layout)
    for (int i = 0; i < 5; i++) intp[i] = (int32_t)(1.0f + i * 0.5f * 8388608.0f);
    // Read them back as float
    for (int i = 0; i < 5; i++) printf("int %d -> float %.6f\n", intp[i], fp[i]);
    free(raw);
    return 0;
}
