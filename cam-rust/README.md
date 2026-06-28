# Rust Camera ISP Pipeline

A complete Rust port of the `cam_app` Java/Kotlin camera ISP pipeline — plus significant enhancements.
Compiles with **0 warnings** and **224 tests pass** (218 unit + 6 integration).

## Pipeline (11 stages in CpuEngine)

```
RawInput(INT16) → Normalize(FLOAT) → DPC(median) → Gaussian Denoise → AWB(controller)
→ BLC/WB → LSC(radial) → Malvar Demosaic(RGB) → [IspController stats feedback]
→ CCM(3×3) → AE(gain) → Tone(gamma+contrast+sat+unsharp) → Display(UINT8 BGRA)
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

- **CpuEngine** — Full 11-stage software ISP in pure Rust (no external ML deps)
- **MnnEngine** — Vulkan GPU acceleration (HD 114 FPS, FHD 50 FPS, 4K 14 FPS)
- **OnnxEngine** — ONNX Runtime inference (ort v2.0.0-rc.12)
- **IspController** — AWB (gray world + CCT clamp + zone-weighted), AE (luminance + histogram), CCM (quadratic CCT interpolation + scale/offset + EMA), tone (S-curve + shadow lift + highlight roll), scene-adaptive ISP
- **Scene-adaptive ISP** — Automatically classifies scene (Dark/Indoor/Sunset/Outdoor/Bright) and adjusts AWB, exposure, contrast, saturation, gamma per frame
- **AVX2/SSE2 SIMD** — 8-wide/4-wide f32 operations (normalize, CCM, AE gain, display)
- **NEON/NEON-FP16/NEON-DOTPROD** — ARM64 SIMD backends
- **AutoExposureEngine** — Computes exposure time + ISO from scene stats
- **GeneticOptimizer** — Genetic algorithm for ISP parameter calibration (8 genes, tournament selection, blend crossover, elitism)
- **FastPredictor** — Per-CCT-bin averaging, O(1) update/predict
- **RegressionModel** — 4-feature OLS regression for 10 ISP targets
- **LearnerStore** — In-memory ring buffer + disk persistence (CSV, 1 MB auto-trim)
- **EisEngine** — Gyro stabilization with warp grid (7 tests)
- **AfEngine** — Autofocus state machine (13 tests)
- **CalibrationStats** — Quad-level sensor metadata (6 tests)
- **CcmEngine** — Quadratic CCT-based CCM interpolation (8 tests)
- **Malvar demosaic** — Gradient-based directional interpolation
- **Defective pixel correction** — 3×3 median hot pixel removal
- **Lens shading correction** — Radial gain vignetting removal
- **ONNX model generator** — Pure Rust proto encoder, 20 nodes (2719 bytes)
- **PipelineProfile** — LITE/MED/HEAVY/PRO presets with feature flags
- **PipelineManager** — Build/process lifecycle orchestrator
- **Binder HAL** — AIDL-style provider/device/session/callbacks + ISP integration
- **Android Camera2 HAL3** — camera3.h FFI, AHardwareBuffer, V4L2 detection
- **Vulkan→CPU auto-fallback** — Graceful backend degradation
- **FP16 output** — Float16Rgb/Float16Bgra (halves GPU→CPU bandwidth)
- **Bayer pattern configurability** — RGGB/GRBG/GBRG/BGGR
- **8K support** — ONNX generation + MNN conversion at 7680×4320
- **HDR merge block** — Multi-exposure fusion with luminance weight maps

## Quick Start

```bash
cd cam-rust

# Run all tests (224)
cargo test --lib

# Single frame through ISP pipeline
cargo run --example pipeline -p cam-isp -- --out out.png

# Camera → ISP integration
cargo run --example camera_isp --features mnn -p cam-isp -- --width 640 --height 480

# Continuous streaming
cargo run --example stream_isp --features mnn -p cam-isp -- --width 640 --height 480 --fps 30

# Generate ONNX model
RUST_LOG=info cargo run -p cam-app -- --width 1280
```

## Test Coverage (224 tests)

| Module | Tests | Description |
|--------|------:|-------------|
| `controller.rs` | 12 | AWB, AE, CCM, tone, zone stats, scene classification |
| `store.rs` | 15 | LearnerStore ring buffer, CCT indexing, CSV, disk persistence |
| `predictor.rs` | 9 | Per-CCT-bin averaging, confidence |
| `regression.rs` | 7 | OLS, Gauss-Jordan, 4-feature predictions |
| `scene.rs` | 9 | Dark/Indoor/Sunset/Outdoor/Bright classification |
| `calibration.rs` | 6 | Quad-level CFA stats |
| `af.rs` | 13 | State machine, VCM↔diopter, peak detection |
| `eis.rs` | 7 | Gyro integration, warp grid |
| `ccm_engine.rs` | 8 | Quadratic CCT, sanitize |
| `ae.rs` | 6 | Exposure time, ISO |
| `genetic.rs` | 8 | GA optimization convergence |
| `onnx/proto.rs` | 8 | Pure-Rust ONNX protobuf encoder |
| `pipeline_test.rs` (integration) | 6 | Gray, gradient, color, convergence, edge |
| `profile.rs` | 4 | Pipeline presets |
| `config.rs` | 5 | Config builder |
| `manager.rs` | 2 | Build/process lifecycle |
| `fused.rs` | 1 | Engine wrapper |
| `binder/` | 7 | Binder HAL + ISP integration tests |
| `cam-onnx/` | 2 | ONNX Runtime wrapper tests |

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

### ✅ Completed
- Base ISP pipeline (CpuEngine + IspController) — 11 stages, feedback loop
- Scene-adaptive ISP (Dark/Indoor/Sunset/Outdoor/Bright adjustments)
- GeneticOptimizer — GA for ISP parameter calibration
- LearnerStore disk persistence (CSV, 1 MB auto-trim)
- Pipeline profile/config/manager/fused — complete
- AutoExposureEngine + BrightnessEngine — complete
- CcmEngine (quadratic CCT) + CcmComposer (scale/offset/EMA) — complete
- ONNX model generator — 20 nodes, 2719 bytes
- V4L2 adapter — Linux capture via rscam
- MNN C++ wrapper — complete
- **MNN Vulkan ISP pipeline** — 3-dispatch fused pipeline (HD 114 FPS, FHD 50 FPS, 4K 14 FPS)
- **IspChainFusion converter pass** — ONNX→Extra op fusion with named params (blc/wb/ccm/fcs/ee/ldci/display)
- **E2E test harness** — ONNX gen → MNN convert → Vulkan inference → verify
- **FP16 output** — Float16Rgb/Float16Bgra (halves GPU→CPU bandwidth)
- **Bayer pattern configurability** — RGGB/GRBG/GBRG/BGGR via --bayer-pattern
- **8K support** — ONNX generation + MNN conversion verified at 7680×4320
- **HDR merge block** — Multi-exposure fusion with luminance weight maps
- **Vulkan→CPU auto-fallback** — Graceful backend degradation
- **AVX2/SSE2 SIMD backends** — 8-wide/4-wide f32 operations
- **NEON/NEON-FP16/NEON-DOTPROD** — ARM64 SIMD backends
- **ONNX Runtime wrapper** — cam-onnx with ort v2.0.0-rc.12
- **Binder HAL** — AIDL-style provider/device/session/callbacks
- **Binder ISP integration** — IspCameraSession bridges camera → ISP
- **Android Camera2 HAL3** — camera3.h FFI, AHardwareBuffer, V4L2 detection
- **Android NDK ABI support** — build.rs with NDK detection
- **Timestamp passthrough** — ProcessParams.timestamp_ns → IspFrame
- **Streaming examples** — camera_isp.rs, stream_isp.rs
- Malvar demosaic, DPC, LSC, histogram, zone stats — complete
- **224 tests, 0 warnings, 21 modules, 10 crates**

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
