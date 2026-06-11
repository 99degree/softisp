# CLAUDE.md — Rust Camera ISP Pipeline

## Status: 🚧 IN PROGRESS (80% Complete)

✅ ONNX generation (2719 bytes, 20 nodes, 11 initializers, 8 inputs, 1 output)
✅ V4L2 HAL via rscam (streaming thread, callbacks, auto-detect /dev/video*)
✅ MNN inference with zero-copy tensor data and shape setting
✅ Binder service skeleton (ICameraProvider, ICameraDevice, ICameraDeviceSession)
✅ Unit test for pipeline composition (GraphComposer produces valid ONNX)
✅ Clean architecture: cam-core simplified, cam-app generator
⏳ Android Camera2 NDK implementation (stub only)
⏳ Additional ISP blocks (HDR, AI, Transform, Overlay, Detail)
⏳ ONNX Runtime backend (ort feature) integration
⏳ End-to-end demo: V4L2 capture → MNN inference → output

## Pipeline (9 blocks → 20 ONNX ops)

```
RawInput(INT16) → Normalize(FLOAT) → CFA(Conv) → BLC(Sub) → WB(Mul)
→ Demosaic(SpaceToDepth+Conv) → CCM(Gemm) → Tone(Clip+Pow)
→ Display(Resize+Transpose+Pad+Gather+Cast) → UINT8 BGRA
```

## Architecture

```
cam-app (binary) – ONNX model generator
  └─ cam-isp (engines, blocks, GraphComposer, ONNX proto)
      └─ cam-hal (trait)
          ├─ cam-hal-android (stub adapter for trait compatibility)
          └─ cam-hal-linux (V4L2 adapter using rscam, streaming via callback)
cam-core (ApplicationHolder – holds pipeline and optional camera adapter)
cam-binder (Android Binder service – provider/device/session stubs)
```

## Key Components

| Crate | Purpose | Status |
|-------|---------|--------|
| `cam-types` | Core types (Frame, FrameFormat, ToneParams, IspBlock) | ✅ |
| `cam-isp` | ISP engine, 9 blocks, ONNX proto encoder, GraphComposer | ✅ |
| `cam-hal` | `ICameraAdapter`, `ByteFrame`, `StreamConfig` | ✅ |
| `cam-hal-android` | Stub adapter (compatibility) | ✅ |
| `cam-hal-linux` | V4L2 adapter (`rscam`), callback streaming | ✅ |
| `cam-core` | `ApplicationHolder` (pipeline + camera holder) | ✅ |
| `cam-onnx` | ONNX Runtime bindings (placeholder) | ✅ |
| `cam-motion` | MotionCompensator (placeholder) | ✅ |
| `cam-binder` | Binder service (AIDL-style interfaces) | 🚧 Skeleton |
| `cam-app` | CLI: `cargo run -- --width 1280` → generates `isp_pipeline.onnx` | ✅ |

## MNN C Wrapper (`cam-isp/mnn_sys`)

Exposes:
- `mnn_interpreter_create_from_buffer`
- `mnn_session_create` / `mnn_session_run` / `mnn_session_resize`
- `mnn_tensor_get_host_data_raw` – raw pointer to tensor buffer (zero-copy)
- `mnn_tensor_get_data_size` – size in bytes
- `mnn_interpreter_get_model_buffer` – get MNN format buffer (for conversion)
- `mnn_interpreter_save_model` – save model to file

Rust wrappers (`mnn_sys.rs`):
- `MnnInterpreterSafe`, `MnnSessionSafe`, `MnnTensorSafe`
- `as_bytes()` / `as_bytes_mut()` for safe zero-copy access

## V4L2 HAL (`cam-hal-linux`)

- Uses `rscam` crate for pure-Rust V4L2 access
- Auto-enumerates `/dev/video*` devices
- `V4l2CameraAdapter`:
  - `initialize()` spawns a background streaming thread
  - Frame callback via `ICameraAdapter::set_frame_callback`
  - `start_streaming()` / `stop_streaming()` control loop
  - `capture_frame()` placeholder (not used in callback mode)

## Binder Service (`cam-binder`)

- `ICameraProvider`: `getCameraIdList`, `getCameraDevice`, `setCallback`, etc.
- `ICameraDevice`: `open`, `getCharacteristics`, `close`
- `ICameraDeviceSession`: `processCaptureRequest`, `flush`, `close`
- Transactions implemented as Parcel reads/writes
- Registers as `"media.camera"` via `ServiceManager::add_service`

### Build with Android

```bash
cargo build --features android -p cam-binder
# Requires Android NDK (libbinder_ndk.so) and `rsbinder` crate with NDK support.
```

## How to Build & Run

```bash
# Generate ONNX model (host)
cd cam-rust
cargo run -p cam-app -- --width 1280
# Output: isp_pipeline.onnx (2719 bytes)

# Run tests
cargo test -p cam-isp          # ONNX proto tests + pipeline composition test
```

## Known Gaps

- MNN inference path not fully validated on real model (tensor types: INT16 input, UINT8 output expected, but MNN may auto-convert)
- Android Camera2 NDK implementation missing (only stub)
- Binder service not integrated with real camera capture (should forward to V4L2 adapter)
- Missing many ISP blocks from original (HDR, AI, Transform, Overlay, Detail)
- ONNX Runtime backend (`ort` feature) not fully linked (needs `libonnxruntime.so`)
- Unused imports and dead code warnings throughout

## Next Steps

1. Validate MNN inference with real RAW frame data (V4L2)
2. Implement `NdkCameraAdapter` using Android NDK `ACameraManager`
3. Wire V4L2 adapter into `CameraDeviceSession` capture callbacks
4. Add missing ISP blocks (at least one per category: HDR merge, AI detect, Resize transform)
5. Enable ONNX Runtime backend and test with same pipeline
6. Clean up warnings and dead code across crates

---

*Original project: `cam_app` (Java/Kotlin) ported to pure Rust + optional C++ FFI.*
