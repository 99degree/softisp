# Cam-Rust — Pure Rust Android Camera ISP Pipeline

A complete rewrite of the Java/Kotlin `cam_app` Android camera ISP pipeline in **pure Rust**.
Generates an **ONNX model on-the-fly** from 9 processing blocks, runs inference via ONNX Runtime
or Alibaba MNN.

## Architecture

```
cam-app/ (binary entry point)
 ├─ cam-types/        Core types: Frame, FrameFormat, IspBlock, ToneParams
 ├─ cam-isp/          ISP engine trait, 9 processing blocks, ONNX proto encoder, GraphComposer
 │  ├─ onnx/proto.rs  Pure-Rust ONNX protobuf wire encoder (no external protobuf lib)
 │  ├─ pipeline.rs    IspBlock trait, IspFrame, GraphComposer, PipelineBuilder
 │  ├─ engine.rs      IspEngine trait, EngineFactory, registry, select_engine()
 │  ├─ blocks.rs      9 ISP blocks: RawInput→Normalize→CFA→BLC→WB→Demosaic→CCM→Tone→Display
 │  ├─ onnx/mod.rs    OnnxEngine (ORT) and OnnxModelComposer
 │  └─ mnn.rs         MnnEngine (MNN)
 ├─ cam-hal/          Hardware abstraction layer: ICameraAdapter, ByteFrame, BufferManager
 ├─ cam-hal-android/  Android NDK implementation: Camera2RawAdapter, HardwareBufferOps
 ├─ cam-onnx/         ONNX Runtime Rust bindings (placeholder)
 ├─ cam-motion/       Motion estimation / EIS (placeholder)
 └─ cam-core/         PipelineManager, ApplicationHolder, DebugService
```

## Pipeline Blocks (9 blocks, in order)

| # | Block | ONNX Ops | Input Shape | Output Shape |
|---|-------|----------|-------------|--------------|
| 1 | RawInputBlock | — | — | INT16[1,1,H,W] |
| 2 | NormalizeBlock | Cast→Div | INT16[1,1,H,W] | FLOAT[1,1,H,W] |
| 3 | CfaBlock | Conv(2×2,stride2) | FLOAT[1,1,H,W] | FLOAT[1,4,H/2,W/2] |
| 4 | BlcBlock | Sub | FLOAT[1,4,H/2,W/2] | FLOAT[1,4,H/2,W/2] |
| 5 | BayerWbBlock | Mul | FLOAT[1,4,H/2,W/2] | FLOAT[1,4,H/2,W/2] |
| 6 | DemosaicBlock | SpaceToDepth→Conv(1×1) | FLOAT[1,4,H/2,W/2] | FLOAT[1,3,H/2,W/2] |
| 7 | CcmBlock | Gemm | FLOAT[1,3,H/2,W/2] | FLOAT[1,3,H/2,W/2] |
| 8 | ToneBlock | Sub→Mul→Add→Clip→Pow | FLOAT[1,3,H/2,W/2] | FLOAT[1,3,H/2,W/2] |
| 9 | DisplayBlock | Resize→Transpose→Pad→Gather→Mul→Cast | FLOAT[1,3,H,W] | UINT8[1,H_out,W_out,4] |

**Output**: 2719-byte ONNX ModelProto with 20 nodes, 11 initializers, 8 inputs, 1 output.

## Quick Start

```bash
# Build
cargo build -p cam-app

# Run — composes ONNX model from 9 pipeline blocks, saves to ./isp_pipeline.onnx
RUST_LOG=info cargo run -p cam-app -- --width 1280
```

## ONNX Model Structure

The generated model (2719 bytes) includes:

**Runtime inputs** (7 extra tensors):
- `NormalizeBlock/sensor_max` — float[1] — camera sensor max value
- `BlcBlock/blc` — float[4,1,1] — per-channel black level correction
- `BayerWbBlock/gains` — float[4,1,1] — white balance gains
- `ToneBlock/contrast` — float[1] — contrast factor
- `ToneBlock/brightness` — float[1] — brightness offset
- `ToneBlock/gamma_recip` — float[1] — 1/gamma value
- `DisplayBlock/sizes` — int64[4] — target output size [N,C,H,W]

**Initializers** (11 baked tensors):
- CFA unpack weights [4,1,2,2] and bias [4]
- Demosaic Bayer→RGB weights [3,4,1,1] and bias [3]
- Identity CCM matrix [3,3] and zero bias [3]
- ToneBlock half constant [0.5]
- DisplayBlock scale [255], pads [0,0,0,0,0,0,0,1], const_val [1], reorder indices [2,1,0,3]

## Engine Selection

Engines are registered with priorities (higher = preferred):

| Engine | Priority |
|--------|----------|
| MNN Vulkan | 95 |
| ONNX NNAPI | 90 |
| MNN CPU_NEON | 75 |
| ONNX CPU | 70 |
| ONNX XNNPACK | 80 |

The `select_engine()` function returns the highest-priority engine.
MNN is preferred for GPU inference; ONNX Runtime with NNAPI is the fallback.

## Feature Flags

- `ort` — Enables ONNX Runtime inference via the `ort` crate (requires `libonnxruntime.so`)

```bash
# Build with ORT inference support
cargo build -p cam-isp --features ort
```

## Tests

```bash
# Run ONNX proto encoder tests
cargo test -p cam-isp -- onnx::proto

# Run all tests
cargo test
```

## Cross-Compilation for Android

```bash
rustup target add aarch64-linux-android

# Build static library for Android HAL service
cargo build --target aarch64-linux-android --release -p cam-app
```

## Generated Model Validation

The ONNX model can be validated with any ONNX-compatible tool:

```bash
# Using Python onnxruntime
python3 -c "
import onnxruntime as ort
session = ort.InferenceSession('isp_pipeline.onnx')
print(f'Inputs: {[i.name for i in session.get_inputs()]}')
print(f'Outputs: {[o.name for o in session.get_outputs()]}')
"
```
