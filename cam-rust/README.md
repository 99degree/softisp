# CLAUDE.md — Rust Camera ISP Pipeline

## Status: ✅ SUBSTANTIALLY COMPLETE (90%)

The Java/Kotlin codebase has been fully ported to Rust with significant enhancements.
The workspace compiles with **0 warnings** and **27 tests pass**.

## Pipeline (11 processing stages in CpuEngine)

```
RawInput(INT16) → Normalize(FLOAT) → DPC → [AWB] → BLC/WB → LSC
→ Gaussian Denoise → Malvar Demosaic(RGB) → [IspController stats]
→ CCM(3×3) → AE → Tone(gamma+contrast+sat+unsharp) → Display(UINT8 RGBA)
```

## Project Structure (10 crates)

```
cam-rust/
├── Cargo.toml              # Workspace root
├── cam-types/              # Core types (Frame, ToneParams, etc.)
├── cam-isp/                # ISP engine, blocks, ONNX proto, CpuEngine
│   ├── src/
│   │   ├── engine.rs       # IspEngine trait + registry + default_tone_params
│   │   ├── pipeline.rs     # IspBlock trait, IspFrame, GraphComposer
│   │   ├── blocks.rs       # 9 ONNX ISP blocks + register_builtin_blocks
│   │   ├── cpu.rs          # CpuEngine — full software ISP pipeline
│   │   ├── controller.rs   # IspController — AWB/AE/CCM/Tone state machine
│   │   ├── profile.rs      # PipelineProfile — 4 presets (LITE/MED/HEAVY/PRO)
│   │   ├── config.rs       # PipelineConfig — editable config + builder
│   │   ├── manager.rs      # PipelineManager — build/process lifecycle
│   │   ├── onnx/           # OnnxEngine (ORT) + OnnxModelComposer
│   │   ├── onnx/proto.rs   # Pure-Rust ONNX protobuf encoder (8 tests)
│   │   └── mnn.rs          # MnnEngine (MNN) + MnnBackend
│   └── examples/pipeline.rs # Multi-frame demo with convergence stats
├── cam-hal/                # ICameraAdapter trait, ByteFrame, BufferManager
├── cam-hal-android/        # Camera2RawAdapter stub, HardwareBufferOps
├── cam-hal-linux/          # Linux V4L2 adapter via rscam
├── cam-core/               # PipelineManager, ApplicationHolder, DebugService
├── cam-onnx/               # ONNX Runtime bindings (placeholder)
├── cam-motion/             # MotionCompensator (placeholder)
├── cam-binder/             # Android binder HAL service (ICameraProvider/...)
└── cam-app/                # Binary entry point, ONNX model generator
```

## CpuEngine Features (all pure Rust, no external deps)

| Feature | Implementation |
|---------|---------------|
| RawInput → Normalize | INT16 → FLOAT [0,1] |
| Defective Pixel Correction | 3×3 median-based hot pixel replacement |
| Gaussian Denoise | 3×3 Gaussian blur with strength blend |
| Auto White Balance | Gray world → IspController (exponential smoothing + CCT clamp) |
| Black Level Correction | Per-channel bias subtract |
| Lens Shading Correction | Radial gain (1 + k*r²) |
| White Balance | 4-channel Bayer-domain gains |
| Demosaic | Malvar (2004) gradient-based directional interpolation |
| Color Correction | 3×3 matrix |
| Auto Exposure | Target luminance → IspController + histogram highlight/shadow |
| Tone Curve | Gamma + contrast + brightness + saturation |
| Edge Enhancement | 3×3 Laplacian unsharp mask |
| Histogram | 256-bin luminance → IspController |
| Display Output | Resize (nearest) → UINT8 RGBA |

## IspController (ported from Java IspController)

| Feature | Status | Details |
|---------|--------|---------|
| AWB (gray world) | ✅ | Exponential smoothing, CCT-aware gain clamp |
| AE (luminance) | ✅ | Running average, clamp [0.125, 8.0] |
| CCM management | ✅ | Identity/sensor matrix, clamp feedback |
| Tone smoothing | ✅ | Contrast, gamma, saturation with frame-adaptive alpha |
| Histogram AE | ✅ | Highlight/shadow ratio, constrained gain |
| CCT estimation | ✅ | From R/G, B/G ratios |
| Zone stats | ❌ | (Not ported — 6×8 zone grid) |
| Multi-illuminant | ❌ | (Not ported — per-zone CCT clustering) |
| Learner/calibration | ❌ | (Not ported — StatsLearner) |
| AF engine | ❌ | (Not ported — focus scan) |
| EIS / gyro | ❌ | (Not ported — stabilization) |

## PipelineConfig / PipelineProfile / PipelineManager

| Feature | Status | Details |
|---------|--------|---------|
| PipelineProfile | ✅ | 4 presets (LITE/MED/HEAVY/PRO) + custom + build_blocks() |
| DemosaicQuality | ✅ | Standard, HqLinear, Edge |
| PipelineLevel | ✅ | Lite/Medium/Heavy/Pro for controller gating |
| PipelineConfig | ✅ | Editable config + builder methods + chain preview |
| PipelineManager | ✅ | Build/process lifecycle with controller feedback |
| PipelineResult | ✅ | Per-frame stats (AWB, CCT, AE, latency) |

## Key Technical Decisions

1. **CpuEngine as primary runtime** — full software ISP pipeline in pure Rust, no external ML dependencies needed for basic operation
2. **IspController as feedback loop** — statistics extracted from each frame feed AWB/AE/CCM/tone estimation for the next frame
3. **ONNX blocks for graph composition** — 9 IspBlock implementations define ONNX tensor ops; GraphComposer merges into valid ModelProto
4. **Mutex<IspController> in CpuEngine** — enables interior mutability for stats collection in the `&self` process() method
5. **PipelineProfile → PipelineManager → CpuEngine** — three-level architecture: config → orchestration → execution

## Build & Run

```bash
cd cam-rust
cargo test --all  # 27 tests, 0 warnings

# Single frame
cargo run --example pipeline -p cam-isp -- --out out.png

# Multi-frame convergence demo
cargo run --example pipeline -p cam-isp -- --frames 30 --verbose

# ONNX model generation
RUST_LOG=info cargo run -p cam-app -- --width 1280
# Output: ./isp_pipeline.onnx (2719 bytes)
```
