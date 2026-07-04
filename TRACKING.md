# SoftISP — Project Tracking

## Current State: Full GPU ISP Pipeline (MNN Vulkan backend)

GPU-accelerated camera ISP pipeline running on MNN's Vulkan backend.
Non-packed INT16 sensor input → full GPU processing → display output.

### Performance (Vulkan, Snapdragon 8 Gen 2)

| Mode | Resolution | Latency | FPS | Notes |
|---|---|---|---|---|
| Non-packed INT16 | 4K→FHD | **17.3 ms** | **57.7** | Fused unpack+demosaic+display, Vulkan |
| Non-packed INT16 | HD→HD | ~5 ms | **200** | Minimal preset, Vulkan |
| Non-packed INT16 | Full pipeline | 17.3 ms | 57.7 | 6-stage, all fusions active |
| Dynamic Tile WG | 4K→FHD | 17.3 ms | 57.7 | Mali: 32×8, Adreno: 64×4, Apple: 16×16 |

### Pipeline Fusion Stats

| Metric | Value |
|--------|-------|
| ONNX ops (unfused) | 32+ |
| GPU dispatches | **12** (after all fusions) |
| Fusion rules active | R1, R9, R10, R10b, R11, R11b, R12, R12b |
| Const buffer size | 26 floats (BLC, WB, CCM, FCS, gamma, display) |

### Pipeline Profiles

| Profile | Block Sequence | Use Case |
|---|---|---|
| LITE | unpack → display | Preview, minimal latency |
| MED | unpack → ee → display | Edge enhancement + preview |
| HEAVY | unpack → fcs → ee → ldci → display | Full cosmetic pipeline |
| PRO | unpack → gamma → sharpen → contrast → denoise → display | Photo capture |
| TEST | unpack → display (Identity) | Pipeline validation |

### 4K→FHD Pipeline (Non-packed INT16, Fused)

```
RawInput(INT16 [1,1,2160,3840])
  → [GPU: Extra(isp.unpack_demosaic)] → [1,3,540,960] float32
  → [GPU: Extra(isp.ee_ldci)] → [1,3,540,960]
  → [GPU: Extra(isp.display)] → [1,4,540,960] float32
```

### Fusion Rules (IspChainFusion)

| Rule | Pattern | Result | Status |
|---|---|---|---|
| R1 | unpack_blc (Cast→Conv + Concat→Conv) | `isp.unpack_blc` | ✅ |
| R1b | Rust packed-int16 Concat→Conv | `isp.unpack_blc` | ✅ |
| R2 | Conv 1×1 4ch→3ch | `isp.demosaic_ccm` | ✅ |
| R3a | Conv 3×3 group=3 | `isp.ee` | ✅ |
| R3b | Mul+Add | `isp.fcs` | ✅ |
| R4 | AvgPool+Sub+Mul+Add | `isp.ldci` | ✅ |
| R5 | Pow+Clip | `isp.display` | ✅ |
| R6 | Pow+Clip (alt) | `isp.display` | ✅ |
| R7 | fcs+display | `isp.fcs_display` | ✅ |
| R8 | fcs+display (adjacent) | `isp.fcs_display` | ✅ |
| R9 | ee+ldci → ee_ldci | `isp.ee_ldci` | ✅ |
| R10 | unpack_demosaic | `isp.unpack_demosaic` | ✅ |
| R10b | unpack_demosaic (bilinear) | `isp.unpack_demosaic` | ✅ |
| R11 | unpack_demosaic + fcs | N/A (FCS absorbed) | ✅ |
| R11b | unpack_demosaic + display | `isp.unpack_demosaic` (display gamma fused) | ✅ |
| R12 | unpack_demosaic + fcs | `isp.unpack_demosaic` (FCS fused) | ✅ |
| R12b | unpack_demosaic + fcs (MHC) | `isp.unpack_demosaic` | ✅ |

### 44 ISP Blocks

| Category | Blocks |
|----------|--------|
| **Input** | RawInputBlock, UnpackBlock, UnpackCfaBlock, NormalizeBlock, UnpackBayerToFp16Block |
| **Demosaic** | BayerDemosaicBlock, BayerWbBlock, DemosaicBlock, DemosaicCcmBlock, DemosaicInterpBlock, CfaBlock |
| **Color** | CcmBlock, ColorSpaceBlock, GammaBlock, ToneBlock |
| **Enhance** | EeBlock, LdciBlock, FcsBlock, SharpenBlock, AutoContrastBlock |
| **Warp** | WarpGridBlock, ChromaticAberrationBlock |
| **Denoise** | TemporalDenoiseBlock, NoiseEstimateBlock |
| **Effects** | HdrMergeBlock, StereoDepthBlock, GrayscaleBlock, PyramidBlock |
| **Resize** | DynResizeBlock, AspectCropBlock, AdaptiveDownscaleBlock, ResizeBlock |
| **Stats** | CoarseHistogramBlock, ChannelMeansBlock, ToneStatsBlock |
| **Display** | DisplayBlock |
| **Other** | IdentityBlock, FastDemosaicBlock, BlcBlock, FlipBlock, LensShadingBlock (merged), NormalizeBlock, MbAlignBlock, SmartPadBlock |

### 490 Tests

| Suite | Count |
|-------|-------|
| Lib unit tests | 432 |
| Integration (new_blocks, builder methods) | 56 |
| E2E (isp_pipeline) | 2 |
| **Total** | **490** |

All 36 examples compile, 0 warnings.

## Completed Milestones

### ✅ Pipeline Fusion
- 12+ fusion rules reduce 32+ ONNX ops → 12 GPU dispatches
- R11b fuses display gamma into unpack_demosaic const buffer
- R12b fuses FCS + MHC demosaic into single Extra op
- Pipeline-order fusion (Pass2 walks Extras in tensor chain order)

### ✅ Vulkan Backend
- Dynamic Tile Workgroup (Mali: 32×8, Adreno: 64×4, Apple: 16×16)
- Early-Z rejection via valid_bounds dispatch culling
- Runtime hot-swap for 3A parameters (no model rebuild)
- FP16 const buffers (f32→f16 packing, halved bandwidth)
- Concurrent session mutex fix (vkQueueSubmit race condition)
- Output descriptor rebind fix

### ✅ Runtime Parameterization
- `MNNVulkanHotSwapConstBuffer` — C API for live 3A updates
- `GpuWorkgroupProfile` — GPU auto-detect lookup table
- Per-session workgroup override
- Named presets: fast_4k, low_power, portrait, universal

### ✅ E2E Test Harness
- ONNX generation → MNN conversion → Vulkan inference → output verification
- All 5 profile variants (LITE, MED, HEAVY, PRO, TEST)
- 4K→FHD benchmark with P50/P99 latencies

### ✅ GPU Deshake
- Hybrid: Grayscale+Pyramid+Warp on GPU, Diamond search on CPU
- GridSampler native on Vulkan
- 6504 bytes SPIR-V for bilinear warp

### ✅ Pipeline Composer API
- GraphComposer: compose_auto, compose_full, compose_full_at, compose_and_convert, compose_benchmark, compose_report, pipeline_flops_estimate, pipeline_memory_estimate, validate_pipeline, validate_with_fixes, wire_blocks, pipeline_report, pipeline_summary
- PipelineBuilder: 20+ builder methods, 5 named presets, to_mnn, to_config, validate, block_ids, remove_block, replace_block, all_stats, from_config, summary
- PipelineStats, PipelineDiff, PipelineSnapshot, PipelineSerializer, OptProfile

### ✅ ISP Correction Blocks
- WarpGridBlock (GDC + EIS + LensShading + Rotation)
- ChromaticAberrationBlock (per-channel radial offset)
- AutoContrastBlock (S-curve with shadow lift)
- TemporalDenoiseBlock (motion-adaptive blending)
- HdrMergeBlock (multi-exposure fusion)
- NoiseEstimateBlock (Laplacian-based)
- SharpenBlock (unsharp mask)
- ColorSpaceBlock (RGB↔YUV via Conv)
- StereoDepthBlock (SAD block-matching)
- GammaBlock (Log→Mul→Exp, optional shadow lift)
- AspectCropBlock (center-crop via Slice)
- DynResizeBlock (dynamic resolution, hot-swappable scales)

## Design Decisions

### Why binning demosaic, not interpolation
The CFA Conv with stride=2 treats each 2×2 Bayer block as one pixel with
4 color channels (R, Gr, Gb, B). This is pixel binning: the output is
H/2×W/2 — half resolution, no pixel replication.

Interpolation demosaic (bilinear, MHC) reconstructs full resolution H×W.
Bilinear: ~4ms FHD. MHC: ~8ms FHD with edge-aware quality.

### Why Cast is fused, not removed
Vulkan doesn't support standalone `Cast` (OpType_Cast=9). IspChainFusion
converts `Cast+Conv` into a single Extra op with a custom SPIR-V shader
that performs the type conversion inline during the convolution.

### Why no intermediate ConvertTensor ops
Between Extra ops, ConvertTensor would reshape CHW planar data and
corrupt it. The pipeline order ensures direct Extra→Extra tensor chains
are continuous without ConvertTensor.

### Why output_value_info differs by mode
- PackedInt32: input width = W/2, output = H/2 × W/2/sw
- NativeInt16: input width = W, output = H/2 × W/sw

## Testing
```bash
# Run all lib tests (432)
cargo test --lib -p cam-isp --features mnn

# E2E (serial, requires Vulkan)
cargo test --test test_e2e_isp_pipeline -p cam-isp --features mnn -- --ignored --test-threads=1

# Integration (35)
cargo test --test test_new_blocks -p cam-isp --features mnn

# 4K→FHD benchmark
LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
  cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn

# Camera ISP example
cargo run --example camera_isp --features mnn -p cam-isp -- --width 640 --height 480
```
