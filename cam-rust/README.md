# Rust Camera ISP Pipeline

A complete Rust port of the `cam_app` Java/Kotlin camera ISP pipeline. Compiles with **0 warnings** and **40 tests pass**.

## Pipeline (11 stages)

```
RawInput(INT16) → Normalize → DPC → Denoise → AWB → BLC/WB → LSC
→ Malvar Demosaic(RGB) → [stats → IspController] → CCM → AE → Tone → Display(UINT8)
```

## Architecture (10 crates)

| Crate | Purpose |
|-------|---------|
| `cam-types` | Core types: Frame, ToneParams, BlockDef |
| `cam-isp` | ISP engine: CpuEngine, IspController, blocks, ONNX proto, profiles |
| `cam-hal` | Hardware abstraction: ICameraAdapter trait |
| `cam-hal-linux` | V4L2 adapter via `rscam` |
| `cam-hal-android` | Android Camera2 NDK stub |
| `cam-core` | PipelineManager, ApplicationHolder |
| `cam-onnx` | ONNX Runtime bindings (placeholder) |
| `cam-motion` | MotionCompensator (placeholder) |
| `cam-binder` | Android AIDL-style binder HAL |
| `cam-app` | Binary entry, ONNX model generator |

## Key Features

- **CpuEngine** — Full 11-stage software ISP in pure Rust (no external ML deps)
- **IspController** — AWB (gray world + CCT clamp), AE (luminance + histogram), CCM, tone
- **AutoExposureEngine** — Computes exposure time + ISO from scene stats
- **PipelineProfile** — LITE/MED/HEAVY/PRO presets with feature flags
- **PipelineConfig** — Editable config builder with chain preview
- **PipelineManager** — Build/process lifecycle orchestrator
- **FusedPipeline** — Engine-agnostic runtime wrapper
- **Malvar demosaic** — Gradient-based directional interpolation
- **ONNX model generator** — Pure Rust proto encoder, 20 nodes

## Quick Start

```bash
cd cam-rust

# Run all tests
cargo test --all

# Single frame through ISP pipeline
cargo run --example pipeline -p cam-isp -- --out out.png

# Multi-frame convergence demo (AWB/AE stabilization)
cargo run --example pipeline -p cam-isp -- --frames 30 --verbose

# Generate ONNX model
RUST_LOG=info cargo run -p cam-app -- --width 1280
```

## Module Map (cam-isp)

```
src/
├── lib.rs          # Module root + init() (engine registration)
├── engine.rs       # IspEngine trait, registry, default_tone_params
├── pipeline.rs     # IspBlock trait, IspFrame, GraphComposer
├── blocks.rs       # 9 ONNX ISP block implementations
├── cpu.rs          # CpuEngine — full software ISP (11 stages)
├── controller.rs   # IspController — AWB/AE/CCM/Tone state machine
├── ae.rs           # AutoExposureEngine — exposure time + ISO
├── profile.rs      # PipelineProfile — 4 presets + build_blocks()
├── config.rs       # PipelineConfig — editable config builder
├── fused.rs        # FusedPipeline — engine-agnostic wrapper
├── manager.rs      # PipelineManager — build/process lifecycle
├── onnx/           # OnnxEngine + OnnxModelComposer
│   └── proto.rs    # Pure-Rust ONNX protobuf encoder
└── mnn.rs          # MnnEngine + MnnBackend
tests/
└── pipeline_test.rs  # 6 integration tests (gray, gradient, color, convergence, edge)
```

## Status

✅ Base ISP pipeline (CpuEngine + IspController) — complete
✅ Pipeline profile/config/manager/fused — complete
✅ AutoExposureEngine + BrightnessEngine — complete
✅ ONNX model generator — complete
✅ V4L2 adapter — complete
✅ MNN C++ wrapper — complete
✅ Binder HAL — complete
✅ 40 tests, 0 warnings, 10 crates

❌ Zone stats / multi-illuminant AWB — not ported
❌ StatsLearner calibration — not ported
❌ AF engine — not ported
❌ EIS / gyro stabilization — not ported
❌ HDR merge — not ported
❌ Android Camera2 NDK — stub only
❌ ONNX Runtime / MNN inference — needs .so libs
