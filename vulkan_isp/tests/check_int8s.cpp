#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <memory>
#include "schema/current/MNN_generated.h"

int main() {
    flatbuffers::FlatBufferBuilder fbb(65536);
    
    // Create a Blob with int8s data
    struct UniformData {
        uint32_t input_width;
        uint32_t input_height;
        uint32_t output_width;
        uint32_t output_height;
        uint32_t sensor_max;
        float blc_r, blc_gr, blc_gb, blc_b;
        float wb_r, wb_gr, wb_gb, wb_b;
    } data = {
        8, 8, 4, 4, 1023,
        0.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 1.0f, 1.0f, 1.0f
    };
    
    printf("Original data:\n");
    printf("  input_width=%u\n", data.input_width);
    printf("  output_width=%u\n", data.output_width);
    printf("  sensor_max=%u\n", data.sensor_max);
    printf("  wb_r=%f\n", data.wb_r);
    
    // Create BlobT
    std::unique_ptr<MNN::BlobT> blob(new MNN::BlobT);
    blob->dataType = MNN::DataType_DT_INT8;
    blob->int8s.resize(sizeof(data));
    memcpy(blob->int8s.data(), &data, sizeof(data));
    
    // Pack
    auto blobOffset = MNN::Blob::Pack(fbb, blob.get());
    fbb.Finish(blobOffset);
    
    // Now unpack to verify
    auto blobPtr = fbb.GetBufferPointer();
    auto blobSize = fbb.GetSize();
    
    auto blobUnpacked = MNN::GetBlob(blobPtr);
    printf("\nFlatbuffer size: %zu\n", blobSize);
    printf("dataType=%d\n", blobUnpacked->dataType());
    
    auto int8s = blobUnpacked->int8s();
    if (int8s) {
        printf("int8s size=%d\n", int8s->size());
        UniformData* unpacked = (UniformData*)int8s->data();
        printf("Unpacked data:\n");
        printf("  input_width=%u\n", unpacked->input_width);
        printf("  output_width=%u\n", unpacked->output_width);
        printf("  sensor_max=%u\n", unpacked->sensor_max);
        printf("  wb_r=%f\n", unpacked->wb_r);
    } else {
        printf("int8s is NULL!\n");
        printf("  Checking float32s: %p\n", (void*)blobUnpacked->float32s());
        printf("  Checking int32s: %p\n", (void*)blobUnpacked->int32s());
    }
    
    return 0;
}
