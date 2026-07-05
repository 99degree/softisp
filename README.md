# SoftISP — ISP Pipeline (MNN Vulkan)

GPU-accelerated camera ISP pipeline. Rust → ONNX → MNN → Vulkan SPIR-V.

## Architecture

```
Bayer RAW → [Extra(isp.unpack_demosaic)] → [Extra(isp.ee_ldci)] → [Extra(isp.display)] → RGB
              BLC+WB+CCM+demosaic+FCS      EE + LDCI fused          sRGB gamma + format
```

**12 GPU dispatches** (minimum after all fusions), 579 tests, 0 warnings.

## Performance (Vulkan, Snapdragon 8 Gen 2)

| Resolution | Latency | FPS | Notes |
|---|---|---|---|
| HD 1280×720 | ~5 ms | **200** | Minimal preset |
| 4K→FHD 3840→1920 | 17.3 ms | **57.7** | Full pipeline, all fusions |
| 4K→FHD 3840→1920 | 9.0 ms | **111** | LITE profile (unpack→display) |

## Features

- **44 ISP blocks**: input, demosaic, color, enhance, warp, denoise, effects, stats, display
- **579 tests**: 544 lib + 24 integration + 2 e2e
- **36 examples** compile, 0 warnings
- **12 fusion rules**: R1–R12b (IspChainFusion.cpp)
- **Runtime 3A**: hot-swap const buffers, workgroup presets
- **GDC + EIS**: WarpGridBlock with radial lens shading
- **E2E**: ONNX → MNN → Vulkan → verify

## Pipeline Profiles

| Profile | Builder Method | Use Case |
|---|---|---|
| LITE | `minimal_preset(w, h)` | Preview/focus |
| MED | demosaic → ee → display | Edge enhancement |
| HEAVY | fcs → ee → ldci → display | Full cosmetic |
| PRO | `photo_preset(w, h)` | Photo capture |
| TEST | Identity op | Validation |

## PipelineBuilder (Fluent API)

```rust
let onnx = PipelineBuilder::new(1920, 1080)
    .unpack()
    .demosaic(2)         // 0=binning, 1=bilinear, 2=MHC
    .gamma(2.2)
    .sharpen(0.6)
    .contrast(1.3)
    .denoise(0.03)
    .display()
    .compose()
    .unwrap();
```

One-liner: `PipelineBuilder::photo_preset(1920, 1080).compose()`

## Blocks (44 total)

| Category | Blocks |
|----------|--------|
| **Input** | RawInputBlock, UnpackBlock, UnpackCfaBlock, NormalizeBlock, UnpackBayerToFp16Block |
| **Demosaic** | BayerDemosaicBlock, BayerWbBlock, DemosaicBlock, DemosaicCcmBlock, DemosaicInterpBlock, CfaBlock, FastDemosaicBlock |
| **Color** | CcmBlock, ColorSpaceBlock, GammaBlock, ToneBlock |
| **Enhance** | EeBlock, LdciBlock, FcsBlock, SharpenBlock, AutoContrastBlock |
| **Warp** | WarpGridBlock (GDC+EIS+LensShading+Rotation), ChromaticAberrationBlock |
| **Denoise** | TemporalDenoiseBlock, NoiseEstimateBlock |
| **Effects** | HdrMergeBlock, StereoDepthBlock, GrayscaleBlock, PyramidBlock |
| **Resize** | DynResizeBlock, AspectCropBlock, AdaptiveDownscaleBlock, ResizeBlock |
| **Stats** | CoarseHistogramBlock, ChannelMeansBlock, ToneStatsBlock |
| **Display** | DisplayBlock (RGBA/ARGB/AGBR/FloatRgb) |
| **Other** | IdentityBlock, BlcBlock (instance-aware), MbAlignBlock, SmartPadBlock |

## Build

```bash
cd ~/MNN/build_vk && make MNNConvert -j$(nproc)
cp tools/converter/OFF/libMNNConvertDeps.so ../cam-rust/lib/aarch64-v8a/
cd ~/softisp/cam-rust && cargo build -p cam-isp --features mnn
```

## Test

```bash
cargo test --lib -p cam-isp --features mnn               # 544 lib tests
cargo test --tests -p cam-isp --features mnn                # 24 integration
```

## Fusion Rules (IspChainFusion.cpp)

| Rule | Pattern | → | Status |
|---|---|---|---|
| R1 | unpack_blc | `isp.unpack_blc` | ✅ |
| R2 | Conv 1×1 4ch→3ch | `isp.demosaic_ccm` | ✅ |
| R3a | Conv 3×3 group=3 | `isp.ee` | ✅ |
| R3b | Mul+Add | `isp.fcs` | ✅ |
| R4 | AvgPool+Sub+Mul+Add | `isp.ldci` | ✅ |
| R5 | Pow+Clip | `isp.display` | ✅ |
| R9 | ee+ldci | `isp.ee_ldci` | ✅ |
| R10 | unpack+demosaic | `isp.unpack_demosaic` | ✅ |
| R11b | unpack_demosaic+display | unpack absorbs display gamma | ✅ |
| R12b | unpack_demosaic+fcs (MHC) | `isp.unpack_demosaic` | ✅ |

## Key Design Decisions

- **Fused Cast**: INT16→F32 fused into unpack Conv shader (no CPU roundtrip)
- **No intermediate ConvertTensor**: Extra→Extra tensor chains avoid CHW data corruption
- **Binning stride**: Conv stride=4 for 4K→FHD downscale (not Resize — Vulkan unsupported)
- **FP16 const buffers**: f32→f16 packing halves GPU→shader bandwidth
- **Precision_High**: Vulkan backend requires this for correct fp32 output
- **Rpath**: Embedded in binary for `libMNN.so` runtime linking
