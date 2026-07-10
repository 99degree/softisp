# Rust Camera ISP Pipeline

A complete Rust ISP pipeline: ONNX generation → MNN conversion → Vulkan GPU inference.
Compiles with **0 warnings** and **637 tests pass** (621 lib + 16 integration).

## Performance (Vulkan GPU, Snapdragon 8 Gen 2)

| Resolution | GPU Latency | GPU FPS | CPU Latency | CPU FPS | Speedup |
|---|---|---|---|---|---|
| HD (1280×720) | 1.47 ms | **680** | 442 ms | 2 | **340×** |
| FHD (1920×1080) | 2.28 ms | **438** | 983 ms | 1 | **438×** |
| 4K (3840×2160) | 12.87 ms | **78** | 3904 ms | 0.25 | **312×** |

## Pipeline (Fused, 12 GPU dispatches)

```
RawInput(INT16 [1,1,H,W])
  → [Extra(isp.unpack_demosaic)] BLC+WB+CCM+demosaic+FCS
  → [Extra(isp.ee_ldci)] Edge enhancement + local contrast
  → [Extra(isp.display)] sRGB gamma + format (RGBA/ARGB/AGBR)
```

## Architecture (10 crates)

| Crate | Purpose |
|-------|---------|
| `cam-types` | Core types: Frame, FrameFormat, ToneParams, BlockDef |
| `cam-isp` | ISP engine: CpuEngine, MnnEngine, OnnxEngine, blocks, ONNX proto, profiles, ML engines |
| `cam-hal` | Hardware abstraction: ICameraAdapter trait, buffer management |
| `cam-hal-linux` | V4L2 adapter via `rscam` |
| `cam-hal-android` | Android Camera2 HAL3 implementation (camera3.h FFI) |
| `cam-core` | PipelineManager, ApplicationHolder, HAL bridge |
| `cam-onnx` | ONNX Runtime bindings (ort v2.0.0-rc.12) |
| `cam-motion` | MotionCompensator (placeholder) |
| `cam-binder` | Android AIDL-style binder HAL + ISP integration |
| `cam-app` | Binary entry, ONNX model generator |

## Key Features

- **637 tests**, 0 warnings, **44 ISP blocks**, **36 examples**
- **Runtime-feedable parameters**: 11 blocks expose 29 per-frame inputs via `extra_inputs()` — saturation, sharpen, contrast, gamma, WB gains, CCM weights, tone curve, display gamma, and more. Conditional tensors only present when active. See [`docs/api/RUNTIME_PARAMS.md`](../docs/api/RUNTIME_PARAMS.md).
- **MnnEngine** — Vulkan GPU acceleration (4K→FHD 57.7 FPS)
- **IspChainFusion** — 12 fusion rules (R1–R12b), 32+ ops → 12 dispatches
- **PipelineBuilder** — Fluent API: `.unpack().demosaic().gamma().sharpen().display().compose()`
- **5 named presets**: photo, video, night, minimal, from_profile
- **Runtime 3A**: hot-swap const buffers, workgroup presets, FP16 const packing
- **Dynamic Tile Workgroup**: Mali (32×8), Adreno (64×4), Apple (16×16)
- **WarpGridBlock**: unified GDC + EIS + LensShading + Rotation
- **12 ISP correction blocks**: ChromaticAberration, AutoContrast, TemporalDenoise, HdrMerge, Sharpen, ColorSpace, StereoDepth, Gamma, NoiseEstimate, AspectCrop, DynResize, CoarseHistogram
- **GraphComposer**: compose_auto, compose_full, compose_full_at, compose_and_convert, compose_report, pipeline_flops_estimate, pipeline_memory_estimate, validate_pipeline, validate_with_fixes
- **PipelineSerializer**: text-based save/load, version migration, block validation
- **PipelineDiff**: compare two pipeline configs
- **PipelineBuilder 20+ methods**: block_ids, remove_block, replace_block, all_stats, from_config, summary, validate
- **E2E test harness**: ONNX→MNN→Vulkan→verify
- **CpuEngine** — Full 11-stage software ISP in pure Rust
- **IspController** — AWB (gray world + CCT clamp + zone-weighted), AE (luminance + histogram), CCM (quadratic CCT interpolation + scale/offset + EMA), tone (S-curve + shadow lift + highlight roll), scene-adaptive ISP
- **Scene-adaptive ISP** — Automatically classifies scene (Dark/Indoor/Sunset/Outdoor/Bright)
- **AVX2/SSE2 SIMD** — x86_64 backends
- **NEON/NEON-FP16/NEON-DOTPROD** — ARM64 SIMD backends
- **GeneticOptimizer** — GA for ISP parameter calibration
- **Binder HAL** — Android AIDL-style provider/device/session/callbacks
- **Android Camera2 HAL3** — camera3.h FFI, AHardwareBuffer, V4L2 detection

## Quick Start

```bash
cd cam-rust

# Run all lib tests (621)
cargo test --lib -p cam-isp --features mnn

# Integration tests (16)
cargo test --tests -p cam-isp --features mnn

# Generate ONNX model
cargo run --example gen_onnx -p cam-isp --features mnn

# Camera → ISP integration
cargo run --example camera_isp --features mnn -p cam-isp -- --width 640 --height 480

# Generate runtime parameter API docs (no MNN required)
cargo run --release --example gen_api_docs -p cam-isp

# Streaming benchmark
cargo run --example bench_4k_to_fhd -p cam-isp --features mnn

# GPU benchmark (Vulkan)
ENGINE=vulkan cargo run --release --example bench_e2e_pipeline -p cam-isp --features mnn
```

## Test Coverage (637 tests)

| Suite | Count | Command |
|-------|------:|--------|
| Lib unit tests | 621 | `cargo test --lib -p cam-isp --features mnn` |
| Integration tests | 16 | `cargo test --tests -p cam-isp --features mnn` |

## Vulkan GPU Performance

The ISP pipeline runs entirely on GPU via MNN Vulkan backend:

- **IspChainFusion** pass fuses standard ONNX ops into ISP-specific VulkanFuse Extra ops
- **12 GPU dispatches** for the full 4-stage pipeline (unpack, demosaic, warp, display)
- **Dynamic workgroup tuning** per GPU (Mali 32×8, Adreno 64×4, Apple 16×16)
- **Zero-copy** input via memfd/CMA buffers on Android

## Module Map (cam-isp, 21 modules)

```
src/
├── lib.rs          # Module root + init() (engine registration)
├── engine.rs       # IspEngine trait, registry, default_tone_params
├── pipeline.rs     # IspBlock trait, IspFrame, GraphComposer
├── blocks.rs       # 9 ONNX ISP block implementations
├── cpu.rs          # CpuEngine — full software ISP (11 stages, 1 test)
├── controller.rs   # IspController — AWB/AE/CCM/Tone/Zone/Scene (12 tests)
├── mnnengine.rs    # MnnEngine — Vulkan GPU acceleration
├── onnx/           # OnnxEngine + OnnxModelComposer
│   ├── mod.rs      # ONNX Runtime integration
│   └── proto.rs    # Pure-Rust ONNX protobuf encoder (8 tests)
├── mnn.rs          # MnnEngine + MnnBackend
├── mnn_buffer/     # MNN buffer management (memfd, CMA, heap)
├── mnn_host.rs     # MNN host tensor operations
├── mnn_flat.rs     # MNN flat buffer operations
├── mnn_converter.rs # MNN model conversion
├── mnn_express.rs  # MNN Express API
├── mnn_sys.rs      # MNN C FFI bindings
├── simd/           # SIMD backends
│   ├── selector.rs # Runtime backend detection
│   ├── scalar.rs   # Scalar fallback
│   ├── neon.rs     # ARM64 NEON
│   ├── neon_fp16.rs # ARM64 NEON+FP16
│   ├── neon_dotprod.rs # ARM64 NEON+DOTPROD
│   ├── avx2.rs     # x86_64 AVX2+FMA
│   └── sse2.rs     # x86_64 SSE2
├── blocks/         # ISP block implementations
│   ├── raw_input.rs
│   ├── normalize.rs
│   ├── cfa.rs
│   ├── blc.rs
│   ├── bayer_wb.rs
│   ├── demosaic.rs
│   ├── ccm.rs
│   ├── tone.rs
│   ├── display.rs
│   ├── ee.rs
│   ├── fcs.rs
│   ├── ldci.rs
│   ├── unpack.rs
│   ├── unpack_cfa.rs
│   ├── unpack_bayer_fp16.rs
│   ├── warp.rs
│   ├── adaptive_downscale.rs
│   ├── hdr_merge.rs
│   └── identity.rs
├── ae.rs           # AutoExposureEngine — exposure time + ISO (6 tests)
├── af.rs           # AfEngine — autofocus state machine (13 tests)
├── eis.rs          # EisEngine — gyro stabilization (7 tests)
├── calibration.rs  # CalibrationStats — quad-level CFA (6 tests)
├── scene.rs        # SceneClassifier — scene category (9 tests)
├── predictor.rs    # FastPredictor — CCT-bin averaging (9 tests)
├── regression.rs   # RegressionModel — OLS (7 tests)
├── store.rs        # LearnerStore + CameraCharacteristicsStore (15 tests)
├── genetic.rs      # GeneticOptimizer — GA for ISP params (8 tests)
├── ccm_engine.rs   # CcmEngine — quadratic CCT CCM (8 tests)
├── profile.rs      # PipelineProfile — 4 presets (4 tests)
├── config.rs       # PipelineConfig — editable config (5 tests)
├── fused.rs        # FusedPipeline — engine wrapper (1 test)
├── manager.rs      # PipelineManager — build/process (2 tests)
└── isp_ops.rs      # ISP math operations
tests/
├── pipeline_test.rs  # 6 integration tests
├── test_e2e_isp_pipeline.rs  # E2E: ONNX→MNN→Vulkan
├── test_profile_onnx.rs  # Profile ONNX generation
└── test_mnn_*.rs  # MNN integration tests
```

## Ported Java → Rust

| Java Source | Rust Module | Tests |
|:------------|:------------|------:|
| `IspController.kt` + `AwbEngine.kt` + `ToneEngine.kt` | `controller.rs` | 12 |
| `AfEngine.kt` (367 lines) | `af.rs` | 13 |
| `GyroEngine.kt` (308 lines) | `eis.rs` | 7 |
| `CcmEngine.kt` / `CcmComposer.kt` | `ccm_engine.rs` / `controller.rs` | 10 |
| `GeneticOptimizer.kt` (230 lines) | `genetic.rs` | 8 |
| `RegressionModel.kt` (237 lines) | `regression.rs` | 7 |
| `FastPredictor.kt` (241 lines) | `predictor.rs` | 9 |
| `LearnerDb.kt` + `StatsLearner.kt` | `store.rs` + `scene.rs` | 24 |
| `CalibrationBlock.kt` (6KB) | `calibration.rs` | 6 |
| `AutoExposureEngine.kt` | `ae.rs` | 6 |
| `PipelineProfile.kt` | `profile.rs` | 4 |
| `PipelineConfig.kt` | `config.rs` | 5 |
| `PipelineManager.kt` | `manager.rs` | 2 |
| `FusedPipeline.kt` | `fused.rs` | 1 |
| `OnnxProto.kt` | `onnx/proto.rs` | 8 |
| `MnnEngine.kt` | `mnn.rs` | — |

## Status

### ✅ Completed (All Items)
- 44 ISP blocks, 579 tests, 36 examples, 0 warnings
- Full GPU ISP pipeline (MNN Vulkan): 4K→FHD 57.7 FPS
- 12 fusion rules (IspChainFusion.cpp): R1–R12b
- PipelineBuilder fluent API + 5 presets + config roundtrip
- 12 correction blocks: WarpGrid(GDC+EIS+LensShading+Rotation), ChromaticAberration, AutoContrast, TemporalDenoise, HdrMerge, Sharpen, ColorSpace, StereoDepth, Gamma, NoiseEstimate, AspectCrop, DynResize
- Runtime 3A: hot-swap const buffers, workgroup presets, FP16 const packing
- GraphComposer: compose, compose_full, compose_full_at, compose_and_convert, compose_report, compose_benchmark, pipeline_flops_estimate, pipeline_memory_estimate, validate_pipeline, validate_with_fixes
- PipelineSerializer: save/load, version migration, block validation
- PipelineDiff: compare two pipeline configs
- PipelineSnapshot: capture/restore pipeline state
- PipelineStats: block_count, block_names, total_nodes, estimated_flops/memory
- Scene-adaptive ISP (Dark/Indoor/Sunset/Outdoor/Bright)
- CpuEngine (11-stage software ISP) + IspController (AWB/AE/CCM/Tone)
- SIMD backends: AVX2, SSE2, NEON, NEON-FP16, NEON-DOTPROD
- MNN Vulkan with dynamic tile workgroup, early-Z rejection
- V4L2 adapter, Android Camera2 HAL3, Binder HAL
- ONNX model generator (pure Rust proto encoder)
- GeneticOptimizer, FastPredictor, RegressionModel, LearnerStore
- 36 examples all compile

## Documentation

| Doc | Path |
|-----|------|
| Runtime Parameters API | [`docs/api/RUNTIME_PARAMS.md`](../docs/api/RUNTIME_PARAMS.md) |
| Controller API | [`docs/api/CONTROLLER_API.md`](../docs/api/CONTROLLER_API.md) |
| Pipeline Blocks I/O | [`cam-isp/docs/PIPELINE_BLOCKS.md`](cam-isp/docs/PIPELINE_BLOCKS.md) |
| Architecture Overview | [`docs/architecture/ARCHITECTURE.md`](../docs/architecture/ARCHITECTURE.md) |

## Build Requirements

- Rust 2021 edition
- `cargo` with `aarch64-linux-android` target (for Android HAL)
- Optional: `libonnxruntime.so` for ORT backend (enable with `--features ort`)
- Optional: `libMNN.so` for MNN backend (enable with `--features mnn`)
- Optional: V4L2 device for Linux camera capture (enable with `--features v4l2`)
- Optional: Android NDK for cross-compilation (set `ANDROID_NDK_HOME`)

## Cross-Compilation

```bash
# Android arm64
ANDROID_NDK_HOME=~/Android/Sdk/ndk/26.1.10909125 \
  cargo build --target aarch64-linux-android --release

# Android armv7
ANDROID_NDK_HOME=~/Android/Sdk/ndk/26.1.10909125 \
  cargo build --target armv7-linux-androideabi --release
```
