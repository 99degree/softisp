# Project Roadmap & Outstanding Work

> Last updated: 2026-07-13. See [`AGENTS.md`](../AGENTS.md) §14 for the canonical pointer.

## Status (2026-07-13)

- **cam-isp tests:** **754** lib tests pass with `--features mnn`, **0 failures, 0 warnings**
  (up from 741). Added 13 new colorspace conversion unit tests (HSV↔RGB roundtrip,
  LAB, interleaved buffer helpers).
- **P2 items completed:** VCM injection → GpuWarpParams; HSV↔RGB/LAB CPU math
  with ONNX YCbCr weight chain; gyro-aware HDR alignment (EisEngine fallback).
- **HAL/ISP integration module** (`cam-isp/src/integration.rs`): `ZeroCopyBufferManager`
  (CMA/ION/memfd), `CameraIspService` (auto-builds the CPU pipeline), `AndroidHalIspBridge`,
  `V4l2IspBridge` (`process_frame` / `process_batch` + `BridgeStats`). 6 integration tests pass.
- **P0 hardening (done):**
  - `IspEngine::as_any` / `as_any_mut` are now **required** trait methods (removed the
    `unimplemented!()` default) — a missing downcast impl is a compile error, not a runtime panic.
  - `HdrWorker::align_frames()` replaced the identity TODO with real **EIS-style global-motion
    alignment** (block-matching translation estimation, 4×4 grid + median voting).
- **Pipeline:** 52 ISP blocks; GPU fused pipeline (~12 Vulkan dispatches, 4K→FHD ~57 FPS);
  `CpuEngine` (11-stage pure-Rust + SIMD); `deshake` (CPU `DeshakeEngine` + GPU `DeshakeGpuPipeline`);
  3A controllers (AWB/AE/AF/CCM/tone/scene) + `NeuralController`; ONNX/MNN same-process conversion.
- **README:** relicensed to **GPL-3.0-or-later**; added 52-block catalog, Deshake section, and
  HAL-integration section.

## Outstanding Work (Prioritized)

### P1 — Real HAL ↔ ISP Integration (core goal; scaffolding exists)
- Wire `AndroidHalIspBridge` into `cam-hal-android/src/adapter.rs`
  (connect its `FrameProcessor` to our `IspFrameProcessor`).
- Wire `V4l2IspBridge` into `cam-binder/src/v4l2_aidl_bridge.rs`
  (replace the `bayer_to_rgb_quick` placeholder).
- Exercise the true zero-copy path end-to-end (`process_raw_frame_zc` still copies today).
- **Constraint:** `cam-hal-android` / `cam-binder` pull Android/NDK deps and require the
  `aarch64-linux-android` target toolchain — they do **not** build in this Termux env.

### P2 — Feature Completeness ✅ (Completed 2026-07-13)
- **Neural controller zoom output → IspParams → GpuWarpParams** — already wired
  via `GpuWarpParams::from_isp_params()` which reads `isp.zoom` and `isp.vcm_position`.
- **VCM position from AfEngine → IspParams.vcm_position** — `UnifiedPipeline` now
  has `set_vcm_position()` which injects AF engine VCM after neural controller
  inference, before warp param construction. ✅
- **`blocks/colorspace.rs` HSV→RGB/LAB** — Identity placeholders replaced with real
  Rust CPU math functions (`rgb_to_hsv_pixel`, `hsv_to_rgb_pixel`, `rgb_to_lab_pixel`,
  plane/interleaved helpers). ONNX YCbCr updated with proper Mul-by-weight chain.
  Full GPU ONNX conditional (Where/Floor) graph deferred to follow-up. ✅
- **`HdrWorker::align_frames()` gyro/EisEngine** — Added optional `EisEngine` to
  `HdrWorker`. `align_frames()` now tries gyro-based translation (via integrated
  angular velocity → focal-length pixel shift) before falling back to image-based
  block-matching. ✅

### P3 — Hardening / CI
- Run `cargo clippy --workspace --all-targets --features mnn -D warnings` as a gate
  (cam-isp is clean; other crates are unverified in this environment).
- Add workspace-level integration tests exercising the bridges (in `cam-binder`).
- Keep benchmark numbers device-labeled consistently (SD888 vs Snapdragon 8 Gen 2).

## Build & Test

```bash
cargo test -p cam-isp --features mnn     # 740+ lib tests (incl. integration)
cargo test -p cam-isp                   # default/no-feature build also compiles
cargo clippy -p cam-isp --lib --features mnn -- -D warnings
cargo run --example camera_isp -p cam-isp --features mnn -- --width 640 --height 480
```

## Key Files

| File | Role |
|------|------|
| `cam-isp/src/integration.rs` | HAL/ISP bridges (Android HAL3, V4L2, zero-copy buffers) |
| `cam-isp/src/hdr.rs` | `HdrWorker` + `align_frames` (EIS alignment) |
| `cam-isp/src/engine.rs` | `IspEngine` trait (`as_any` now required) |
| `cam-isp/src/deshake/` | `DeshakeEngine` + GPU pipeline |
| `cam-isp/src/blocks/` | 52 ISP block implementations |
| `docs/ROADMAP.md` | This document |
