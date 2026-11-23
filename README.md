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
- **Vulkan GPU acceleration** — the fused chain is 12 blocks at 4K: 8 image stages
  plus 4 statistics taps, of which 4 stages are single `isp.*` dispatches.
  Measured end to end: **3.1 FPS** at 3840×2160 and **5.7 FPS**
  at 1920×1080 for `FUSED`, **7.2 / 9.6 FPS** for `WHOLE`, against 1.9 / 4.0 for the
  primitive chain. See [Performance](#performance-on-device-vulkan-gpu).
- **Software fallback** — `CpuEngine` implements the full 11-stage ISP in pure Rust
  (with AVX2/SSE2/NEON/NEON-FP16/NEON-DOTPROD SIMD backends).
- **Deshake** — software video stabilization (block-matching motion estimation +
  trajectory smoothing) with an optional fully-GPU pipeline; integrates with
  `WarpGridBlock` for GDC + EIS + lens-shading + rotation.
- **Neural controller** — ONNX-based 3A (AWB/AE/AF/CCM/tone/scene) and a
  `NeuralController` that emits ISP params.
- **Multiple profiles** — LITE / MED / HEAVY / PRO / UNIFIED, plus **FUSED** and
  **WHOLE** (whole-pipeline `isp.pipeline`).
- **HAL integration** — bridges for Android Camera HAL3, V4L2, and AOSP binder HAL.
- **Zero-copy memory** — CMA / ION / memfd buffer manager for camera → ISP → MNN.
- **801 lib tests** pass with `--features rectifier`, 0 failures.

## Prebuilt APK

A debug build of the camera app is checked in at
[`releases/softisp-camera-debug.apk`](releases/softisp-camera-debug.apk) — 37 MB,
arm64-v8a, package `com.softisp.camera`, signed with the standard debug keystore.

```bash
adb install -r releases/softisp-camera-debug.apk
```

It ships the whole fused stack: `libMNN.so`, the `libMNN_Vulkan.so` plugin with the
`isp.*` shaders embedded, and `libMNNConvertDeps.so` for the same-process ONNX→MNN
conversion. Nothing is fetched at run time.

Building it yourself:

```bash
cd cam-rust && cargo apk2 build -p cam-camera-app   # debug profile
# -> cam-rust/target/debug/apk/softisp-camera.apk
```

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
cargo test -p cam-isp --features rectifier    # 801 lib tests
cargo run --example camera_isp -p cam-isp --features mnn -- --width 640 --height 480
```

## Performance (on-device, Vulkan GPU)

End-to-end, whole chain, raw Bayer in → planar RGB out, 8 frames warmup then
20–30 measured. `p50` over the run, not an average of block times.

**Measure with tracing off.** Every ISP trace — the per-command markers, the
zero-copy log, the `[VulkanFuse-PERF]` dispatch report — is behind one switch that
is off by default:

```bash
MNN_ISP_TRACE=1 <command>   # turn them all on, for one debugging run
```

With it off a timing run adds zero bytes to `/sdcard/Download/isp_vlog_trace` and
emits no per-dispatch lines; with it on, one three-profile run writes 26 KB. An op
that falls back off the GPU is still reported unconditionally, because that costs
two full-frame copies and is worth more than any tuning.

**3840×2160** (`PERF_FRAMES=40 cargo test -p cam-isp --test probe_4k_perf --features mnn -- --ignored --nocapture`)

| Profile | Blocks | p50 | p95 | FPS |
|---|---|---|---|---|
| MED (primitives) | 19 | 467.6 ms | 488.0 ms | 2.1 |
| **FUSED** | 13 | **140.0 ms** | 216.4 ms | **7.1** |
| FUSED, statistics off | 13 | 116.5 ms | 135.0 ms | 8.6 |
| WHOLE (`isp.pipeline`) | 2 | 135.8 ms | 182.6 ms | 7.4 |
| UNIFIED (FUSED + all stages + tiles) | 13 | 151.9 ms | 204.4 ms | 6.6 |

**1920×1080** (`PERF_W=1920 PERF_H=1080 PERF_FRAMES=25 …`)

| Profile | Blocks | p50 | p95 | FPS |
|---|---|---|---|---|
| MED (primitives) | 19 | 132.3 ms | 156.7 ms | 7.6 |
| **FUSED** | 13 | **74.5 ms** | 83.5 ms | **13.4** |
| FUSED, statistics off | 13 | 70.3 ms | 79.2 ms | 14.2 |
| WHOLE (`isp.pipeline`) | 2 | 19.7 ms | 22.1 ms | 50.8 |
| UNIFIED (FUSED + all stages) | 13 | 72.6 ms | 80.7 ms | 13.8 |

Fusion buys **2.5×** at 4K and **1.8×** at FHD. Collapsing the whole chain into one
dispatch (`WHOLE`) was worth 3.8× and 2.4× while the statistics still reduced the
full frame; with them pooled, `FUSED` is within ~2× of `WHOLE` at 4K and ~4× at FHD
while keeping the 3A loop closed. `UNIFIED` mirrors FUSED with tiled rendering at 4K
and matches it within noise, confirming the fusion is the driver.

Absolute numbers move with device temperature — the same configuration measured
437.1 ms then 298.0 ms back to back (`probe_4k_drift`). Compare rows *within* a run.

### Where the 4K time goes

The 3A statistics were the whole problem. Each block used to shrink the frame for
itself, so four blocks meant four traversals of a 100 MB tensor:

| Removed | Saves | Share of the chain |
|---|---|---|
| all statistics | 225.3 ms | 67% |
| `tone_stats` | 105.6 ms | 31% |
| `zone_stats` | 82.4 ms | 24% |
| `channel_means` | 40.6 ms | 12% |
| saturation | 33.7 ms | 10% |
| `ee` | −104.4 ms | noise |

They now share one `AveragePool` (`stats_downscale_max = 540`), and the 3A branches
cost **~6% of the frame** instead of 38%. The numbers they hand the controller do
not move: a mean is exact under a box filter, so the calibration quad means are
`[0.9962, 1.0000, 0.9987]` pooled against `[0.9963, 1.0000, 0.9987]` at full
resolution.

The two run-time levers still apply on top: `set_stats_interval(n)` measures every
`n` frames, and `set_stats_enabled(false)` skips the readback entirely (a frame that
skipped reports `stats_measured == false`). Anything under ~10% in the table above is
inside run-to-run spread — `no ee` coming out negative is the honest reminder.

### Historical

The earlier figures in this README (680 / 438 / 78 FPS at HD / FHD / 4K) were
per-block micro-benchmarks taken before the MNN DCE and fused-opset work. They
measured a chain that did not run the same ops this one does; the numbers above
supersede them.

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

### Fused `isp.*` opset

Blocks can emit a purpose-built MNN op instead of a chain of ONNX primitives. The
fused op is one Vulkan dispatch with its coefficients fed as per-frame tensors, so a
stage that was five elementwise ops becomes one pass over the frame.

`FuseStages` gates them one at a time, and each one in `FuseStages::ALL` is backed by
a probe that compares it against its primitive form:

| Stage | Replaces | Verified difference |
|---|---|---|
| `isp.unpack_blc_normal_cfa` | cast, div, sub, clip, CFA conv | mean\|Δ\| 0.00010 |
| `isp.demosaic_ccm` + folded WB | demosaic, WB mul, CCM conv | pixel-identical |
| `isp.ee` | 3×3 unsharp conv | peak\|Δ\| 0.0136 |
| `isp.ldci` | local-contrast chain | mean\|Δ\| ≤ 0.001 |
| `isp.downscale` | `Resize` | mean\|Δ\| 0.00037 |
| `isp.display` | gamma pow + clip | mean\|Δ\| ≤ 0.001 |
| `isp.cfa_downscale` | statistics tap `Resize` | statistics only |
| `isp.pipeline` | the *whole* RAW→display chain | mean 0.6019 vs MED’s 0.6243 (box-filter demosaic) |

`PipelineProfile::FUSED` enables the per-stage set — 8 blocks instead of MED's 15:

```
isp.unpack_blc_normal_cfa → isp.demosaic_ccm → tone → isp.ee
  → saturation → isp.display                     (+ two stats taps)
```

`PipelineProfile::WHOLE` goes further and emits a single `isp.pipeline` dispatch for
the entire chain (2 blocks, ~80–115 ms at 1296×970 against MED's ~190–260 ms). It
trades away the 3A readback, edge-directed demosaic and the EIS warp, and it is not
the app default for that reason.

Measured at 1296×970, same stage set, on-device:

| Profile | Blocks | ms |
|---|---|---|
| MED (primitives) | 15 | 166–242 |
| FUSED | 8 | 150 |
| WHOLE | 2 | 80–115 |

The statistics are also a lever: `stats_downscale_max` reduces the CFA quad before
the 3A blocks read it (`isp.cfa_downscale`), and
`MnnEngine::set_stats_interval(n)` measures them every `n` frames instead of every
frame. A skipped frame reports `IspAuxOutput::stats_measured == false`, so a
consumer can tell "nothing new" from "the readback failed".

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

- ✅ 52 ISP blocks, **801 lib tests** (`--features rectifier`), 0 failures
- ✅ Full GPU ISP pipeline (MNN Vulkan), measured end to end on-device:
  - 3840×2160: MED 1.9 FPS · `FUSED` **3.1** · `WHOLE` **7.2**
  - 1920×1080: MED 4.0 FPS · `FUSED` **5.7** · `WHOLE` **9.6**
  - the 3A statistics are 67% of the 4K cost, hence `set_stats_interval` / `set_stats_enabled`
- ✅ `CpuEngine` — 11-stage software ISP with SIMD backends
- ✅ Deshake (CPU + GPU) and EIS stabilization
- ✅ Runtime-feedable params (29 per-frame inputs across 11 blocks)
- ✅ Fused `isp.*` opset — `FUSED` profile: 12 blocks at 4K where MED needs 19
  (323 vs 530 ms); `WHOLE` profile: 2 blocks at 139 ms
- ✅ 3A statistics on a reduced CFA quad (`isp.cfa_downscale`) and a per-frame
  `stats_interval`, with `stats_measured` telling a consumer a frame skipped them
- ✅ `PipelineBuilder` fluent API + 5 presets + config roundtrip
- ✅ HAL integration module (Android HAL3 / V4L2 / binder) + zero-copy buffers
- ✅ 3A controllers (AWB/AE/AF/CCM/Tone/Scene) + NeuralController
- ✅ ONNX model generator (pure-Rust protobuf encoder) + MNN same-process conversion
- ✅ ONNX `TypeProto`/`TensorShapeProto` encoding correct — no more MNN DCE
- ✅ `synth_bayer` module (5 generators) for stress-test inputs

## Documentation

- **[Architecture](docs/architecture/ARCHITECTURE.md)** — system design
- **[Summary](docs/SUMMARY.md)** — detailed overview
- **[Pipeline Blocks](docs/architecture/PIPELINE_BLOCKS.md)** — block reference
- **[Runtime Params](docs/api/RUNTIME_PARAMS.md)** — per-frame parameter API
- **[MNN/Vulkan Guide](docs/guides/MNN_VULKAN_GUIDE.md)** — GPU setup

## Testing

```bash
cargo test -p cam-isp --features rectifier           # 801 lib tests
cargo test -p cam-isp --features mnn                 # GPU/ONNX backends
cargo test --workspace --features mnn                # full workspace
```

## License

- **Code:** GPL-3.0-or-later — see [LICENSE](LICENSE)
- **Models:** CC BY-NC 4.0 (non-commercial)
