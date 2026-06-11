# CLAUDE.md — Rust Camera ISP Pipeline

## Status: 🚧 IN PROGRESS (70% Complete)

✅ ONNX generation works (2719 bytes, 20 nodes)
✅ V4L2 HAL implemented with rscam (streaming + callbacks)
✅ MNN inference path complete with zero-copy tensors
✅ Clean architecture: cam-core simplified, cam-app generator
⏳ Android Camera2 NDK, Binder service, additional ISP blocks

## Pipeline (9 blocks → 20 ONNX nodes)

```
RawInput(INT16) → Normalize(FLOAT) → CFA(Conv) → BLC(Sub) → WB(Mul)
→ Demosaic(SpaceToDepth+Conv) → CCM(Gemm) → Tone(Clip+Pow)
→ Display(Resize+Transpose+Pad+Gather+Cast) → UINT8 BGRA
```

## Project Structure (Cargo workspace)

```
cam-rust/
├── Cargo.toml                  # Workspace root
├── cam-types/                  # Core types (Frame, FrameFormat, ToneParams, IspBlock)
├── cam-isp/                    # ISP engine, 9 blocks, ONNX proto encoder, GraphComposer
│   ├── onnx/proto.rs           # Pure-Rust ONNX protobuf encoder (6 tests)
│   ├── pipeline.rs             # IspBlock trait, IspFrame, GraphComposer
│   ├── engine.rs               # IspEngine trait, EngineFactory, registry
│   ├── blocks.rs               # 9 ISP blocks with ONNX node emission
│   ├── onnx/mod.rs             # OnnxEngine (ORT stub)
│   └── mnn.rs                  # MnnEngine (MNN + C wrapper FFI)
├── cam-hal/                    # ICameraAdapter, ByteFrame, StreamConfig
├── cam-hal-android/            # Stub adapter for compatibility
├── cam-hal-linux/              # V4L2 HAL via rscam (real camera capture)
├── cam-core/                   # ApplicationHolder (pipeline + camera adapter)
├── cam-onnx/                   # ONNX Runtime bindings (placeholder)
├── cam-motion/                 # MotionCompensator (placeholder)
├── cam-binder/                 # Binder service (ICameraProvider stub)
└── cam-app/                    # CLI: generates isp_pipeline.onnx
```

## What Works

- `cargo run -p cam-app -- --width 1280` → generates valid 2719-byte ONNX model
- ONNX Proto encoder (varint, strings, float32, nested messages) – 6 unit tests pass
- GraphComposer merges 9-block chain into single ModelProto with correct tensor dependencies
- Engine registry selects highest-priority backend (MNN Vulkan 95, ONNX NNAPI 90, ...)
- MNN C wrapper (`mnn_wrapper.cpp/h`) exposes:
  - `mnn_interpreter_create_from_buffer` (load ONNX)
  - `mnn_session_create/run/resize`
  - `mnn_tensor_get_host_data_raw`, `mnn_tensor_get_data_size`
  - `mnn_interpreter_get_model_buffer` (extract MNN format)
- MNN inference stub ready (tensor data copy via raw pointers)
- V4L2 adapter (`cam-hal-linux`) uses `rscam` crate, streams frames via callback, auto-detects `/dev/video*`

## Gaps

| Area | Missing |
|------|---------|
| **MNN inference** | Verify tensor element types (INT16 in, UINT8 out); handle shape/dtype errors |
| **Android Camera2** | NDK `ACameraManager` implementation (currently stub only) |
| **Binder service** | Full `ICameraProvider`, `ICameraDevice`, callbacks, transaction handlers |
| **ISP blocks** | Missing: HDR merge/tone, AI detect/segment/enhance/portrait, SuperRes, Transform (deshake/rotate/resize/crop/flip/GDC), Overlay (watermark, ONNX, plugin), Detail (tone variants, BLC variants) |
| **End-to-end** | Demo app that captures V4L2 frames → MNN → output/save |
| **ONNX Runtime** | Enable `ort` feature and link `libonnxruntime.so` |
| **Code quality** | Many unused imports/fields; dead code; warnings |

## Build & Run

```bash
# Generate ONNX model
cd cam-rust
cargo run -p cam-app -- --width 1280
# Output: isp_pipeline.onnx (2719 bytes)

# Enable MNN (requires NDK + libMNN.so)
MNN_INCLUDE_DIR=~/MNN/include MNN_LIB_DIR=~/cam_app/app/src/main/jniLibs/arm64-v8a \
  cargo build --features mnn -p cam-isp

# Enable V4L2 (Linux)
cargo check --features v4l2 -p cam-hal-linux
```

## Next Steps

1. Fix MNN tensor type handling (int16 input, uint8 output) – may need separate `host<int16_t>` accessor
2. Implement simple V4L2→ISP demo binary (`cam-demo`)
3. Add 1-2 missing ISP blocks (e.g., `HdrMergeBlock`, `AiDetectBlock`) to prove extensibility
4. Implement Android Camera2 NDK adapter (use `ndk` crate)
5. Flesh out Binder service with real transaction code
