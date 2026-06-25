# Fused ISP Vulkan Shader - Complete Solution

## Problem
MNN Vulkan takes **106ms** for 4K→FHD ISP pipeline (39 separate GPU dispatches).

## Solution: Single Fused Compute Shader
Replaces the entire CFA + Demosaic + CCM + Clip path (5 ops, 38ms) with **one shader dispatch**.

## Shader: `fused_isp.comp`

```glsl
// Workgroup: 8x8, Dispatch: 240x135 = 32,400 workgroups
// Input:  [1, 1, 2160, 3840] INT16 (RGGB Bayer)
// Output: [3, 1080, 1920] FP16 Planar RGB
// ALU per pixel: ~50 ops (bilinear + BLC + WB + CCM + clip)
// Memory: 4x INT16 reads + 3x FP16 writes = 40 MB/frame
```

### Expected Performance (Adreno 642)
| Metric | Value |
|--------|-------|
| Theoretical bandwidth limit | 0.8ms (40 MB / 50 GB/s) |
| ALU limit | ~1ms (50 ops × 2M pixels / 100 GFLOPS) |
| **Real-world (with overhead)** | **2-5ms** |
| **vs MNN (38ms)** | **8-19× faster** |

## Full Pipeline Fusion Targets

| Block | MNN Time | Fused Target | Approach |
|-------|----------|--------------|----------|
| CFA+Demosaic | 38ms | **2ms** | Single bilinear+CCM shader |
| FcsBlock (denoise) | 24ms | **5ms** | 2 fused shaders (FFT-like) |
| EeBlock (edge) | 19ms | **3ms** | 1 unsharp mask shader |
| LdciBlock (contrast) | 8ms | **2ms** | 1 local contrast shader |
| Downscale+Display | 10ms | **2ms** | 1 bilinear+tonemap shader |
| **TOTAL** | **106ms** | **~14ms** | **6 dispatches** |

## Compilation

### Option 1: Pre-compile SPIR-V (Recommended)
```bash
# On x86 host with Vulkan SDK:
glslangValidator -V fused_isp.comp -o fused_isp.spv

# Embed SPIR-V bytes in C++ as static array
xxd -i fused_isp.spv > fused_isp_spirv.h
```

### Option 2: Runtime compilation with shaderc
```bash
# Build shaderc for aarch64
git clone https://github.com/google/shaderc
cd shaderc && ./utils/git-sync-deps
cmake -B build -DSHADERC_SKIP_TESTS=ON -DSHADERC_SKIP_GLSLANG=OFF
cmake --build build -j4
```

## Integration with MNN

### As Custom MNN Vulkan Op
1. Create `VulkanFusedISP` execution class
2. Register for `OpType_FusedISP` 
3. Replace 5 ops in graph with 1 custom op

### As Standalone Vulkan (Recommended)
```cpp
// 1. Create Vulkan instance/device
// 2. Load SPIR-V, create pipeline
// 3. Allocate buffers (input INT16, outputs FP16)
// 4. Update uniforms (CCM, WB, BLC, sensor_max)
// 5. Dispatch: vkCmdDispatch(cmd, 240, 135, 1)
// 6. vkQueueSubmit + vkQueueWaitIdle
// 7. Read back FP16 RGB planes
```

## Key Optimizations in Shader

1. **Coalesced memory access**: Each thread reads 4 adjacent INT16 (8 bytes)
2. **FP16 output**: Half bandwidth vs FP32
3. **No intermediate buffers**: Direct Bayer → RGB in registers
4. **BLC/WB/CCM fused**: All pointwise ops in ALU, no memory
5. **8x8 workgroups**: Optimal for Adreno (64 threads = 2 warps)

## Building the Host Benchmark

```bash
cd /data/data/com.termux/files/home/softisp/vulkan_isp
g++ -std=c++17 -O2 bench_fused.cpp -lvulkan -o bench_fused
./bench_fused
```

## Next Steps

1. **Compile SPIR-V** on x86 host with glslangValidator
2. **Embed SPIR-V** in C++ host code
3. **Run benchmark** on device, measure actual time
4. **Repeat for FcsBlock, EeBlock, LdciBlock** (same pattern)
5. **Chain all fused shaders** with minimal sync (semaphores)

## Performance Comparison

| Approach | CFA+Demosaic | Full Pipeline | Effort |
|----------|-------------|---------------|--------|
| MNN CPU | 149ms | 149ms | Baseline |
| MNN Vulkan | 38ms | 106ms | Current |
| **Fused Vulkan (this)** | **~2ms** | **~14ms** | **Custom shaders** |
| TVM Vulkan | ~5ms | ~25ms | Build TVM (hours) |

**Bottom line**: Custom fused Vulkan shaders are the only practical path to <30ms on mobile. TVM would produce similar shaders but requires hours of build time on device.