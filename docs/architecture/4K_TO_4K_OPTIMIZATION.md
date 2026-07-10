# 4K→4K ARGB8888 Optimization Implementation Plan

## Current Status
- **4K→FHD (4K→1080p):** 128-152 FPS ✅
- **4K→4K (3840×2160→3840×2160):** 42.1 FPS ❌ (target: 130+ FPS)

## Root Cause Analysis

| Factor | 4K→FHD | 4K→4K | Impact |
|--------|--------|-------|--------|
| Output pixels | 2.1M | 8.3M | 4× |
| Memory bandwidth | ~85 MB/s | ~340 MB/s | 4× |
| GPU dispatch size | 13×12 workgroups | 120×27 workgroups | 9× |
| VulkanFuse dispatch time | ~5ms | ~18ms | 3.6× |

**Primary bottleneck:** Output resolution 4× increase → memory bandwidth saturation

---

## Optimization Strategy

### Phase 1: Tile-based Rendering (Highest Impact)
**Target: 2.5-3× speedup (→ 100-120 FPS)**

Split 4K into 2×2 tiles (1920×1080 each), process in parallel across Vulkan queues.

**Implementation:**
1. Modify `UnifiedPipeline::process_with_warp` to support tile processing
2. Create 4 tile sessions per frame (TL, TR, BL, BR)
3. Each tile processes 1920×1080 (same as FHD)
4. Combine results with zero-copy buffer stitching

**Files to modify:**
- `cam-isp/src/unified_pipeline.rs` - Add `process_tiled` method
- `cam-isp/src/mnnengine.rs` - Add multi-session tile support
- `cam-isp/src/profile.rs` - Add `tile_count` field to `PipelineProfile`

**Key insight:** Each tile is essentially an FHD frame → reuses existing FHD performance.

### Phase 2: Stats Downsampling (High Impact)
**Target: 15% speedup (→ 115-120 FPS)**

Current: Stats blocks (zone_stats, channel_means, tone_stats, histogram) run at full 4K resolution.

**Implementation:**
1. Add `stats_downsample_max: u32` to `PipelineProfile` (already exists but set to 0 for UNIFIED)
2. Set `stats_downsample_max = 1080` for UNIFIED profile
3. Stats blocks read from downscaled Bayer (existing `aux_hook_ds` infrastructure)

**Config change in `profile.rs`:**
```rust
pub const UNIFIED: Self = Self {
    stats_downsample_max: 1080,  // was 0
    ...
};
```

### Phase 3: Kernel Fusion (Medium Impact)
**Target: 2-3 fewer GPU dispatches (→ 5-10% speedup)**

Fuse post-process blocks into single Extra ops:

| Current Blocks | Fused Extra Op |
|---------------|----------------|
| Bilateral + Vignette | `isp.bilateral_vignette` |
| Saturation + Colorspace | `isp.saturation_colorspace` |
| Gamma + Sharpen + Wavelet + AutoContrast | `isp.gamma_sharpen_wavelet_ac` |
| Normalize | (keep separate - different input format) |

**Files to modify:**
- `MNN/tools/converter/source/optimizer/postconvert/IspChainFusion.cpp` - Add fusion rules
- `MNN/tools/converter/source/optimizer/onnxextra/IspOnnxOps.cpp` - Register new ops
- `vulkan_isp/isp_shaders/` - New GLSL shaders

### Phase 4: Precision Optimization (Low-Medium Impact)
**Target: 5-10% bandwidth reduction**

Use FP16 for post-process blocks:
- Gamma, AutoContrast, Sharpen: native FP16 support
- WaveletDenoise: AveragePool works in FP16
- Display: FP16 output already supported

**Implementation:** Add `use_fp16_postproc` flag to `PipelineProfile`, conditional Cast ops in block nodes.

---

## Implementation Timeline

| Week | Phase | Deliverable |
|------|-------|-------------|
| 1-2 | Phase 1 | Tile-based rendering (2×2 tiles) |
| 3 | Phase 2 | Stats downsampling config |
| 4-5 | Phase 3 | Post-process kernel fusion |
| 6 | Phase 4 | FP16 precision support |
| 7 | Integration | Testing, tuning, validation |

---

## Expected Results

| Phase | Projected 4K→4K FPS | Cumulative |
|-------|---------------------|------------|
| Baseline | 42 | 42 |
| Phase 1 (Tile) | 100-120 | 100-120 |
| Phase 2 (Stats) | 115-135 | 115-135 |
| Phase 3 (Fusion) | 125-145 | 125-145 |
| Phase 4 (FP16) | 135-155 | **135-155** |

**Target: ≥130 FPS (matching 4K→FHD performance)**

---

## Technical Details: Tile-based Rendering

### Tile Layout (2×2)
```
┌─────────────┬─────────────┐
│   Tile 0    │   Tile 1    │  1920×1080 each
│  (0,0)      │  (1920,0)   │
├─────────────┼─────────────┤
│   Tile 2    │   Tile 3    │
│  (0,1080)   │  (1920,1080)│
└─────────────┴─────────────┘
```

### Tile Processing Flow
```rust
async fn process_tiled(&mut self, raw_data: &[u8], width: u32, height: u32) -> IspResult<IspFrame> {
    let tile_w = width / 2;
    let tile_h = height / 2;
    
    // Create 4 tile sessions
    let tiles = [
        (0, 0, tile_w, tile_h),     // TL
        (tile_w, 0, tile_w, tile_h), // TR
        (0, tile_h, tile_w, tile_h), // BL
        (tile_w, tile_h, tile_w, tile_h), // BR
    ];
    
    // Process all tiles in parallel (4 Vulkan queues)
    let futures: Vec<_> = tiles.iter().map(|(x, y, w, h)| {
        self.process_tile(raw_data, *x, *y, *w, *h)
    }).collect();
    
    let results = join_all(futures).await;
    
    // Stitch tiles into output buffer (zero-copy)
    self.stitch_tiles(results, width, height)
}
```

### Overlap Handling
- Add 16-pixel overlap between tiles for convolution operations
- Total tile size: 1936×1096 (with overlap)
- Discard overlap during stitching

### Zero-Copy Stitching
- Pre-allocate output buffer: `Vec<u8>` sized for 4K ARGB
- Each tile writes directly to its region in output buffer
- No memcpy needed - Vulkan writes directly to host-visible memory

---

## Files to Modify (Priority Order)

1. **`cam-isp/src/unified_pipeline.rs`** - Add `process_tiled` method
2. **`cam-isp/src/mnnengine.rs`** - Multi-session tile support
3. **`cam-isp/src/profile.rs`** - Add `tile_count`, update UNIFIED stats_downsample
4. **`cam-isp/src/pipeline/types.rs`** - Tile metadata in IspBlock/Frame
5. **`MNN/tools/converter/source/optimizer/postconvert/IspChainFusion.cpp`** - Fusion rules
6. **`vulkan_isp/isp_shaders/`** - New fused shaders

---

## Testing Strategy

1. **Unit test:** Tile stitching correctness (compare tiled vs non-tiled output)
2. **Integration test:** 4K→4K stress test with tiles enabled
3. **Performance test:** Compare FPS with/without tiling
4. **Visual test:** Check tile boundaries for artifacts

```bash
# Test command
cargo run --release -p cam-isp --example stress_unified_4k_to_4k --features mnn -- --tiles 4
```