# ISP Opsets — VulkanFuse Compute Shader Pipeline

## Opset Structure

Each ISP stage is a **standalone op** registered under `OpType_Extra` 
with a unique type string. Ops are composed sequentially in a flatbuffer model.

```
isp.unpack_blc  — Bayer RAW → RGGB + Black Level Correction
isp.demosaic_ccm— RGGB → RGB (Hamilton-Adams demosaic + CCM)
isp.fcs         — Flat Field Correction  
isp.ee          — Edge Enhancement (unsharp mask)
isp.ldci        — Local Dynamic Contrast Improvement
isp.display     — Brightness/Contrast/Saturation + Gamma (sRGB)
```

## Files

| File | Description |
|------|-------------|
| `isp_opset.h` | Opset API: op descriptors + `IspPipelineBuilder` for composing pipelines |
| `test_all_opsets.cpp` | Standalone verification of all 6 opsets individually |
| `test_pipeline_opset.cpp` | End-to-end 6-stage pipeline benchmark via `IspPipelineBuilder` |
| `test_unpack_opset.cpp` | Individual opset test (unpack_blc example) |

## Building

```bash
BASE=/path/to/MNN
INCS="-I\$BASE/include -I\$BASE -I\$BASE/schema/current -I\$BASE/3rd_party/flatbuffers/include"
LIBS="-L\$BASE/build_vk/OFF -L\$BASE/build_vk/express/OFF -L\$BASE/build_vk/source/backend/vulkan/OFF -lMNN -lMNN_Express -lMNN_Vulkan -lpthread -ldl"

g++ -std=c++11 \$INCS -o test_all_opsets test_all_opsets.cpp \$LIBS
./test_all_opsets

g++ -std=c++11 \$INCS -o test_pipeline_opset test_pipeline_opset.cpp \$LIBS
./test_pipeline_opset 480 360
```

## Adding a New Op

1. Write SPIR-V shader with bindings: 0=SSBO(uniforms), 1=SSBO(input), 2=SSBO(output)
2. Add `OpDesc(...)` factory function in `isp_opset.h`
3. Add SPIR-V path to the test's `stages[]` array

## Performance (Adreno GPU, 480×360)

| Configuration | Time | FPS | Status |
|---------------|------|-----|--------|
| All 6 opsets | ~25ms | 40 | ✅ <30ms |
