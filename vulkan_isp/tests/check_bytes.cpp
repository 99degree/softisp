#include <stdio.h>
#include <stdint.h>
#include <string.h>

int main() {
    // Our int32s data
    std::vector<int32_t> data;
    data.push_back(8);     // input_w = 8
    data.push_back(8);     // input_h = 8
    data.push_back(4);     // output_w = 4
    data.push_back(4);     // output_h = 4
    data.push_back(1023);  // sensor_max
    float floats[] = {0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 200.0f, 300.0f, 400.0f};
    for (int i = 0; i < 8; i++) {
        int32_t bits;
        memcpy(&bits, &floats[i], sizeof(bits));
        printf("float %f -> int32 %d (0x%08x)\n", floats[i], bits, bits);
        data.push_back(bits);
    }
    
    printf("\nFull buffer bytes:\n");
    for (size_t i = 0; i < data.size(); i++) {
        uint8_t* b = (uint8_t*)&data[i];
        printf("  [%2zu] 0x%02x%02x%02x%02x (%d)\n", i, b[0], b[1], b[2], b[3], data[i]);
    }
    
    // Expected shader reads:
    // uint input_w = data[0] = 8 ✓
    // Actually, the shader reads uint which is the same bit pattern as int32 for positive values
    printf("\nShader sees:\n");
    printf("  input_width  = %d (as uint)\n", data[0]);
    printf("  output_width = %d (as uint)\n", data[2]);
    printf("  sensor_max   = %d (as uint)\n", data[4]);
    printf("  wb_r         = %f (as float, from int32 bits)\n", *(float*)&data[9]);
    
    return 0;
}
