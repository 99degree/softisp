# CLAUDE.md — Rust Camera ISP Pipeline

## Status: 🚧 IN PROGRESS (60% Complete)

Core ISP pipeline and ONNX generation working. MNN inference partially implemented. Camera HALs stubbed. Full Android integration pending.

## Pipeline (9 blocks → 20 ONNX nodes)

```
RawInput(NT16) → Normalize(FLOAT) → CFA(4ch) → BLC → WB → Demosaic(RGB) → CCM → Tone → Display(UINT8)
```

## Project Structure

```
cam-rust/
├── Cargo.toml                  # Workspace root
├── README.md                   # Full documentation
├── isp_pipeline.onnx           # Generated ONNX model (2719 bytes)
├── cam-types/                  # Core types (Frame, FrameFormat, IspBlock, ToneParams)
├── cam-isp/                    # ISP engine, 9 blocks, ONNX proto encoder, GraphComposer
│   ├── onnx/proto.rs           # Pure-Rust ONNX protobuf encoder (6 tests pass)
│   ├── pipeline.rs             # IspBlock trait, IspFrame, GraphComposer
│   ├── engine.rs               # IspEngine trait, EngineFactory, engine registry
│   ├── blocks.rs               # 9 ISP processing blocks with ONNX node emission
│   ├── onnx/mod.rs             # OnnxEngine (ORT), OnnxModelComposer
│   └── mnn.rs                  # MnnEngine (MNN), MnnBackend enum
├── cam-hal/                    # ICameraAdapter, ByteFrame, BufferManager
├── cam-hal-android/            # Camera2RawAdapter, HardwareBufferOps, GpuBuffer
├── cam-onnx/                   # ONNX Runtime bindings (placeholder)
├── cam-motion/                 # MotionCompensator (placeholder)
├── cam-core/                   # PipelineManager, ApplicationHolder, DebugService
└── cam-app/                    # Binary entry point
```

## Task / Todo

### High Priority
- [x] ONNX Proto Encoder (pure Rust, 6 tests)
- [x] GraphComposer (merge blocks → ONNX)
- [x] 9 Pipeline Blocks (RawInput, Normalize, CFA, BLC, WB, Demosaic, CCM, Tone, Display)
- [x] Engine Registry (priority-ordered)
- [x] OnnxEngine (build + ORT process stub)
- [x] MnnEngine (build + C wrapper + process)
- [x] MNN C wrapper (mnn_wrapper.cpp/h)
- [x] MNN FFI bindings (mnn_sys.rs)
- [x] ONNX → MNN conversion (getModelBuffer)
- [x] Binder service skeleton (cam-binder)
- [x] V4L2 HAL via rscam (cam-hal-linux)
- [ ] Complete MNN inference (tensor types: INT16 input, UINT8 output)
- [ ] Implement Android Camera2 via NDK ACameraManager
- [ ] Wire V4L2 to ISP pipeline end-to-end
- [ ] Add missing ISP blocks (HDR, AI, Transform, Overlay, Detail)
- [ ] Full ICameraProvider bindings with callbacks
- [ ] CaptureSession with buffer management
- [ ] Test on real hardware (Linux V4L2 camera)
- [ ] Clean up unused imports/fields

### Medium Priority
- [ ] DualCameraManager (synchronized capture)
- [ ] IMU/EIS stabilization
- [ ] Calibration modules (AutoCalibrator, ColorCalibrator, VcmLscCalibrator)
- [ ] Plugin system
- [ ] Transform processing
- [ ] FrameState management
- [ ] BlockRegistry (dynamically load block types)
- [ ] ONNX Runtime inference (enable `ort` feature)

### Low Priority
- [ ] Documentation: API docs, user guide
- [ ] CI/CD: GitHub Actions, Android builds
- [ ] Performance benchmarks
- [ ] Memory pool optimization
- [ ] Error handling improvements

## Key Deliverables

| Feature | Status | Details |
|---------|--------|---------|
| ONNX Proto Encoder | ✅ Done | Pure Rust, 6 tests, generates valid ModelProto |
| GraphComposer | ✅ Done | Merges block fragments into ONNX model |
| 9 Pipeline Blocks | ✅ Done | RawInput → Normalize → CFA → BLC → WB → Demosaic → CCM → Tone → Display |
| Engine Registry | ✅ Done | Priority-ordered (MNN Vulkan 95, ONNX NNAPI 90, etc.) |
| OnnxEngine | ✅ Done | Build (compiles ONNX model) + Process (ORT stub) |
| MnnEngine | ⚠️ 80% | Build + C wrapper + FFI; process implemented but tensor types may need fixes |
| MNN C Wrapper | ✅ Done | mnn_wrapper.cpp/h with conversion & raw data access |
| V4L2 HAL | ✅ Done | cam-hal-linux via rscam crate, device enumeration, streaming |
| Binder Service | ✅ Skeleton | cam-binder with provider/device/session stubs |
| CAM-App Binary | ✅ Done | CLI entry, pipeline build, engine registration, model save |
| ONNX Model | ✅ Valid | 2719 bytes, 20 nodes, 11 initializers, 8 inputs, 1 output |
| README | ✅ Done | Full architecture, usage, feature flags |

## Remaining (Optional Enhancements)

- **ORT Inference**: Enable with `--features ort` (requires libonnxruntime.so)
- **MNN Conversion**: Implement ONNX→MNN converter via C wrapper
- **NDK Camera**: Full ACameraManager/ACameraDevice implementation
- **Block Calibration**: Real ISP block params (CCM matrices, BLC values, etc.)

## Build & Run

```bash
cd cam-rust
cargo build -p cam-app
RUST_LOG=info cargo run -p cam-app -- --width 1280
# Output: ./isp_pipeline.onnx (2719 bytes)
```
