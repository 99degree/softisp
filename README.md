# SoftISP

A complete camera ISP pipeline in Rust with Vulkan GPU acceleration.

## What It Does

Converts raw Bayer sensor data into display-ready images in real-time:

```
Raw Sensor → Unpack → BLC → WB → Demosaic → CCM → Tone → Denoise → Display
```

The pipeline is data-driven: every stage is an `IspBlock` that can be composed,
converted to ONNX, fused into GPU "Extra" ops, and executed on CPU (pure Rust)
or on the GPU (MNN Vulkan). Per-frame parameters (gain, saturation, sharpen,
gamma, CCM weights, tone curve, warp grid, …) are feedable at runtime.

> **Note on ONNX usage:** The ONNX graph is an *optimization/dispatch pattern*,
> not a guaranteed-correct algorithm specification. softISP emits ONNX primarily
> so that MNN's fusion rules rewrite the standard ops into custom ISP *opset*
> shaders (e.g. `isp.unpack_packed`, `isp.argb_convert`, `isp.gamma`). The ONNX
> op sequence is there to *trigger* MNN's custom-opset code path — it does **not**
> assert that the numerical behavior of every block has been independently
> verified against the pure-Rust reference. Treat ONNX/MNN output as a GPU fast
> path; the authoritative pipeline semantics live in `CpuEngine`.
>
> **If you implement a new algorithm as an `IspBlock`**, you must make sure the
> ONNX op sequence you emit is actually *recognized* by `libMNNConvertDeps` and
> gets fused into the intended custom `isp.*` opset. If the pattern is **not**
> recognized, the converter silently falls back to the **standard ONNX→MNN opset**
> path. Worse, a *partially* matching (but semantically different) pattern can be
> mis-classified as an existing `isp.*` opset and executed via the wrong custom
> shader — producing **incorrect output with no error**. Always validate a new
> block's GPU/MNN result numerically against the `CpuEngine` reference.

## Key Features

- **52 ISP blocks** across input, black-level, white-balance, demosaic, color,
  tone, denoise, warp, and display categories (see [Pipeline](#pipeline-blocks)).
- **Vulkan GPU acceleration** — full fused pipeline runs in ~12 GPU dispatches
  (4K→FHD ~57 FPS on Snapdragon 8 Gen 2).
- **Software fallback** — `CpuEngine` implements the full 11-stage ISP in pure Rust
  (with AVX2/SSE2/NEON/NEON-FP16/NEON-DOTPROD SIMD backends).
- **Deshake** — software video stabilization (block-matching motion estimation +
  trajectory smoothing) with an optional fully-GPU pipeline; integrates with
  `WarpGridBlock` for GDC + EIS + lens-shading + rotation.
- **Neural controller** — ONNX-based 3A (AWB/AE/AF/CCM/tone/scene) and a
  `NeuralController` that emits ISP params.
- **Multiple profiles** — LITE / MED / HEAVY / PRO / UNIFIED (and scene-adaptive ISP).
- **HAL integration** — bridges for Android Camera HAL3, V4L2, and AOSP binder HAL.
- **Zero-copy memory** — CMA / ION / memfd buffer manager for camera → ISP → MNN.
- **740+ lib tests** pass with `--features mnn`, 0 warnings.

## Quick Start

```rust
// Create a GPU engine (falls back to CPU automatically)
let mut engine = select_engine_by_name("mnn_vulkan")?;

// Process a frame
let params = ProcessParams::new(3840, 2160, &raw_bayer);
let frame = engine.process(&params)?;
```

Build & test:

```bash
cargo test -p cam-isp --features mnn          # 740+ lib tests
cargo run --example camera_isp -p cam-isp --features mnn -- --width 640 --height 480
```

## Performance (Vulkan GPU, Snapdragon 8 Gen 2)

| Resolution | GPU Latency | GPU FPS | CPU Latency | CPU FPS | Speedup |
|---|---|---|---|---|---|
| HD (1280×720) | 1.47 ms | **680** | 442 ms | 2 | **340×** |
| FHD (1920×1080) | 2.28 ms | **438** | 983 ms | 1 | **438×** |
| 4K (3840×2160) | 12.87 ms | **78** | 3904 ms | 0.25 | **312×** |

4K→FHD down-scale (the common preview/record path) sustains **~57 FPS** on GPU.

## Pipeline Blocks

The `cam-isp` crate ships **52 block implementations** (`src/blocks/`). They are
grouped by function:

### Input & Unpacking
- `RawInputBlock` — ingest INT16/INT32/FP32 sensor tensor
- `NormalizeBlock` — black-level normalize to [0,1]
- `CfaBlock` / `UnpackCfaBlock` — CFA unpack (native + packed variants)
- `UnpackBlock` / `UnpackBayerFp16Block` — packed-Bayer → planar (incl. FP16)
- `BayerProcBlock` — combined Bayer preprocessing
- `HdrDebayerBlock` / `HdrMergeBlock` / `HdrToneBlock` — multi-exposure HDR

### Black Level & White Balance
- `BlcBlock` / `Blc50Block` — black-level correction (10-bit / 12-bit)
- `BayerWbBlock` — per-channel Bayer white balance

### Demosaic
- `DemosaicBlock` / `BayerDemosaicBlock` — bilinear / MHCC demosaic
- `DemosaicCcmBlock` — fused demosaic + CCM
- `DemosaicInterpBlock` — interpolation variants

### Color & Tone
- `CcmBlock` — color correction matrix
- `ToneBlock` — S-curve tone mapping (shadow lift / highlight roll)
- `GammaBlock` — gamma / sRGB
- `ColorSpaceBlock` — colorspace conversion
- `SaturationBlock` — chroma control
- `FcsBlock` — false-color suppression
- `AutoContrastBlock` — adaptive contrast
- `LdciBlock` — local dynamic contrast
- `EeBlock` — edge enhancement
- `GrayscaleBlock` — luminance extract
- `VignettingBlock` — lens vignette correction
- `WatermarkBlock` — overlay

### Warp & Geometry (GPU)
- `WarpBlock` / `GpuWarpBlock` / `RuntimeWarpBlock` — GDC + EIS + lens-shading + rotation
  (grid computed on GPU, fed via runtime tensors `gdc_k1/k2/k3`, `zoom`, `vcm`, `eis_dx/eis_dy`)
- `ChromaticAberrationBlock` — lateral CA correction
- `AspectCropBlock` / `DynResizeBlock` / `ResizeBlock` / `AdaptiveDownscaleBlock` — geometry

### Denoise & Enhancement
- `BilateralBlock` — edge-preserving denoise
- `WaveletDenoiseBlock` — wavelet threshold denoise
- `TemporalDenoiseBlock` — multi-frame temporal denoise
- `LaplacianPyramidBlock` / `PyramidBlock` — multi-scale
- `NoiseEstimateBlock` — noise profiling
- `SharpenBlock` — unsharp mask
- `SuperResBlock` — super-resolution
- `StereoDepthBlock` — stereo depth-from-disparity

### Output
- `DisplayBlock` — sRGB gamma + format packing (RGBA / ARGB / AGBR)
- `IdentityBlock` / `PassthroughBlock` — no-ops for graph shaping
- `StatsBlock` — histogram / statistics extraction
- `StageBlock` / `PluginBlock` — composable sub-pipelines

### Block fusion (GPU)
`IspChainFusion` fuses 32+ standard ONNX ops into **12 Vulkan "Extra" dispatches**
(R1–R12b): `unpack_demosaic`, `ee_ldci`, `display`, plus warp/resize ops. This is
what makes the full 4-stage pipeline run in ~12 GPU dispatches.

## Deshake (Video Stabilization)

`src/deshake/` provides software video stabilization:

- **`DeshakeEngine`** (`deshake/mod.rs`) — block-matching motion estimation
  (`estimate_motion`) → EMA trajectory smoothing → `motion_to_grid()` → warp grid.
- **`motion`** — diamond search, full search, parabolic refinement, weighted median, SAD.
- **`warp`** — bilinear interpolation, crop, border handling.
- **`gpu_pipeline`** — `DeshakeGpuPipeline`, a fully GPU-accelerated stabilization
  path that reuses `WarpGridBlock` (GridSample on Vulkan).

Flow:

```
Frame N-1 ─┐
           ├→ estimate_motion → (dx,dy) → EMA smooth → motion_to_grid()
Frame N  ─┘                                                   ↓
                                              WarpGridBlock (GPU GridSample)
                                                          ↓
                                                 Stabilized Frame
```

EIS is also exposed standalone via `EisEngine` (`eis.rs`), driven by gyroscope
samples, and composable into `WarpGridBlock`.

## HAL / ISP Integration

`src/integration.rs` connects the ISP engine to camera stacks:

- **`ZeroCopyBufferManager`** — CMA / ION / memfd allocation for camera ↔ ISP ↔ MNN.
- **`CameraIspService`** — high-level service wrapping an `IspEngine`; auto-builds
  the CPU pipeline so `process_raw_frame()` works out of the box.
- **`AndroidHalIspBridge`** — bridges Android Camera HAL3 frames into the ISP.
- **`V4l2IspBridge`** — bridges V4L2 captures (via `cam-hal-linux`) into the ISP,
  with `process_frame()` / `process_batch()` and `BridgeStats`.

Real capture lives in `cam-hal-linux` (V4L2 adapter) and `cam-hal-android`
(Camera2 HAL3 + AHardwareBuffer); the bridges above run only the ISP stage.

## Architecture (10 crates)

| Crate | Purpose |
|-------|---------|
| `cam-types` | Core types: Frame, FrameFormat, ToneParams, BlockDef |
| `cam-isp` | ISP engine: CpuEngine, MnnEngine, OnnxEngine, 52 blocks, controllers, deshake, HAL integration |
| `cam-hal` | Hardware abstraction: `ICameraAdapter` trait, buffer management |
| `cam-hal-linux` | V4L2 adapter via `rscam` |
| `cam-hal-android` | Android Camera2 HAL3 (camera3.h FFI, AHardwareBuffer) |
| `cam-core` | PipelineManager, HAL bridge |
| `cam-onnx` | ONNX Runtime bindings (`ort` v2.0.0-rc.12) |
| `cam-motion` | MotionCompensator (EIS/deshake glue) |
| `cam-binder` | Android AIDL-style binder HAL + ISP session integration |
| `cam-app` | Binary entry, ONNX model generator |

## Status

- ✅ 52 ISP blocks, **740+ lib tests** (`--features mnn`), 0 warnings
- ✅ Full GPU ISP pipeline (MNN Vulkan): 4K→FHD ~57 FPS
- ✅ `CpuEngine` — 11-stage software ISP with SIMD backends
- ✅ Deshake (CPU + GPU) and EIS stabilization
- ✅ Runtime-feedable params (29 per-frame inputs across 11 blocks)
- ✅ 12 fusion rules (`IspChainFusion`) → 12 GPU dispatches
- ✅ `PipelineBuilder` fluent API + 5 presets + config roundtrip
- ✅ HAL integration module (Android HAL3 / V4L2 / binder) + zero-copy buffers
- ✅ 3A controllers (AWB/AE/AF/CCM/Tone/Scene) + NeuralController
- ✅ ONNX model generator (pure-Rust protobuf encoder) + MNN same-process conversion

## Documentation

- **[Architecture](docs/architecture/ARCHITECTURE.md)** — system design
- **[Summary](docs/SUMMARY.md)** — detailed overview
- **[Pipeline Blocks](docs/architecture/PIPELINE_BLOCKS.md)** — block reference
- **[Runtime Params](docs/api/RUNTIME_PARAMS.md)** — per-frame parameter API
- **[MNN/Vulkan Guide](docs/guides/MNN_VULKAN_GUIDE.md)** — GPU setup

## Testing

```bash
cargo test -p cam-isp                   # lib tests
cargo test -p cam-isp --features mnn    # with GPU/ONNX backends (740+ tests)
cargo test --workspace --features mnn   # full workspace
```

## License

- **Code:** GPL-3.0-or-later — see [LICENSE](LICENSE)
- **Models:** CC BY-NC 4.0 (non-commercial)
