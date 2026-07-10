# 4K→4K ARGB8888 Optimization Analysis

## Current Performance: 42.1 FPS @ 4K→4K ARGB

### Bottleneck Analysis

| Metric | 4K→FHD (139 FPS) | 4K→4K (42 FPS) | Ratio |
|--------|------------------|----------------|-------|
| Output pixels | 2.1M | 8.3M | 4× |
| Frame time | 7.2 ms | 23.7 ms | 3.3× |
| Memory bandwidth | ~85 MB/s | ~340 MB/s | 4× |

**Primary bottleneck: Output resolution 4× increase**

---

## Optimization Opportunities

### 1. Tile-based Rendering (High Impact)
- Split 4K into 2×2 tiles (1920×1080 each)
- Process tiles in parallel across Vulkan queues
- Expected: 2.5-3× speedup (limited by memory bandwidth)

### 2. Format Optimization (Medium Impact)
- Current: ARGB8888 (4 bytes/pixel, packed INT32 output)
- Alternative: BGRA float (4×4 bytes, direct GPU write)
- Expected: 10-15% reduction in format conversion overhead

### 3. Kernel Fusion (Medium Impact)
- Current: ~12 GPU dispatches for full pipeline
- Target: Fuse Bilateral+Vignette+Saturation+Colorspace into single Extra op
- Expected: 2-3 dispatch reduction

### 4. Stats Downsampling (High Impact)
- Current: Full-res zone stats (4K)
- Target: Downsample to 1080p for AWB/AE stats
- Expected: 10-15% reduction in aux block overhead

### 5. Precision Optimization (Low-Medium Impact)
- FP16 for post-process blocks (Gamma, AutoContrast, Sharpen)
- Expected: 5-10% bandwidth reduction

---

## Projected Performance

| Optimization | Projected FPS | Cumulative |
|-------------|--------------|------------|
| Baseline | 42 | 42 |
| + Tile rendering (2×2) | ~100 | 100 |
| + Stats downsampling | ~115 | 115 |
| + Kernel fusion | ~125 | 125 |
| + FP16 post-process | ~135 | 135 |

**Target: ~130-140 FPS (matching 4K→FHD performance)**

---

## Implementation Priority

### Phase 1: Tile Rendering (Week 1-2)
1. Modify `GraphComposer` to output tiled tensors
2. Add `PipelineProfile::tile_count` parameter
3. Implement Vulkan multi-queue tile dispatch

### Phase 2: Stats Downsampling (Week 1)
1. Add `stats_downsample_factor` to `PipelineProfile`
2. Insert `ResizeBlock` before zone stats blocks
3. Update `build_aux_blocks` to use downsampled tensor

### Phase 3: Kernel Fusion (Week 2-3)
1. Add `isp.postprocess_fused` Extra op in MNN
2. GLSL shader combining Bilateral+Vignette+Saturation+Colorspace
3. Add fusion rule in `IspChainFusion.cpp`

### Phase 4: FP16 Support (Week 3)
1. Enable FP16 for post-process blocks via `OutputFormat::Float16Rgb`
2. Update shaders for FP16 math
2. Add FP16 model variants

---

## Memory Bandwidth Analysis

| Operation | 4K Read | 4K Write | Total |
|-----------|---------|----------|-------|
| Input (packed) | 33 MB | - | 33 MB |
| Unpack/Demosaic | 33 MB | 132 MB | 165 MB |
| Post-process | 132 MB | 132 MB | 264 MB |
| Output (ARGB) | - | 33 MB | 33 MB |
| **Total/frame** | **165 MB** | **297 MB** | **462 MB** |
| **At 42 FPS** | **6.9 GB/s** | **12.5 GB/s** | **19.4 GB/s** |

**Snapdragon 8 Gen 2 LPDDR5X: ~64 GB/s peak** → 30% utilization, room for 2-3× improvement

---

## Tile Rendering Design

```
4K (3840×2160) → 4 tiles of 1920×1080
┌─────────┬─────────┐
│ Tile 0  │ Tile 1  │
├─────────┼─────────┤
│ Tile 2  │ Tile 3  │
└─────────┴─────────┘
```

Each tile:
- Independent Vulkan command buffer
- Shared shader pipelines (bound once per frame)
- Overlap compute with memory copy via double-buffering

---

## Code Locations for Optimization

| Task | Files |
|------|-------|
| Tile rendering | `cam-isp/src/pipeline/build.rs`, `unified_pipeline.rs` |
| Stats downsample | `profile.rs` (PipelineProfile), `profile.rs` (build_aux_blocks) |
| Kernel fusion | `MNN/tools/converter/source/optimizer/postconvert/IspChainFusion.cpp` |
| FP16 shaders | `vulkan_isp/isp_shaders/`, `IspChainFusion.cpp` |
| Vulkan multi-queue | `cam-isp/src/mnnengine.rs`, `cam-isp/src/mnn/` |

---

## Success Criteria

| Metric | Target |
|--------|--------|
| 4K→4K FPS | >120 FPS |
| P99 latency | <12 ms |
| Memory bandwidth | <60% peak |
| Power efficiency | <2W ISP portion |