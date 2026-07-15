# Project Roadmap & Outstanding Work

> Last updated: 2026-07-15. See [`AGENTS.md`](../AGENTS.md) §14 for the canonical pointer.

## Status (2026-07-15)

- **cam-isp tests:** **754** lib tests pass with `--features mnn`, **0 failures, 0 warnings**
  (up from 741). Added 13 new colorspace conversion unit tests (HSV↔RGB roundtrip,
  LAB, interleaved buffer helpers).
- **P2 items completed:** VCM injection → GpuWarpParams; HSV↔RGB/LAB CPU math
  with ONNX YCbCr weight chain; gyro-aware HDR alignment (EisEngine fallback).
- **True zero-copy investigation (2026-07-15):** MNN exposes the external-memory
  sibling API `Tensor::setDevicePtr(ptr, MNN_MEMORY_AHARDWAREBUFFER=14)`. The
  **OpenCL** backend already imports an AHardwareBuffer (wraps the CMA/dma-buf fd
  from V4L2) as device memory, but the **Vulkan** backend does **not** consume
  `setDevicePtr`/external fd at all (only the primitive + `VK_EXT_external_memory_dma_buf`
  headers exist). Recorded as **P2b** below. Also fixed a stale `(noop)` comment in
  `hdr.rs` (`align_frames` is real block-matching, not a noop).
- **`mnn_run_true_zero_copy` hardening:** the C++ stub that ignored its input
  `buffer` (`(void)buffer;`) was rewritten to bind the caller's CMA mmap ptr as the
  MNN input/output tensor host via `Tensor::buffer().host` (zero-copy on the CPU
  path). Remaining gap (Vulkan host→device copy) tracked under P2b.
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

### P1 — Real HAL ↔ ISP Integration ✅ (wired with feature gates)

- Wire `AndroidHalIspBridge` into `cam-hal-android/src/adapter.rs` — Use
  `set_processor()` with a closure that delegates to
  `AndroidHalIspBridge::process_android_frame()`. A convenience method is not
  added to avoid a `cam-isp` dependency in `cam-hal-android`. The wiring
  happens at the `cam-binder` level where both deps are available. ✅
- Wire `V4l2IspBridge` into `cam-binder/src/v4l2_aidl_bridge.rs` — Added
  `isp_service: Mutex<Option<CameraIspService>>` field gated behind
  `#[cfg(feature = "mnn")]`. When `set_isp_service()` is called, the ISP
  pipeline replaces `bayer_to_rgb_quick()` fallback for Bayer→RGB. The mnn
  feature propagates automatically. ✅
- Exercise the true zero-copy path end-to-end (`process_raw_frame_zc` still
  copies today). — Pending: requires AHardwareBuffer at runtime.
- **Constraint resolved:** `cam-binder` now propagates `mnn` feature to
  `cam-isp`, so `integration.rs` module is accessible when built with
  `--features mnn`. Both `cam-hal-android` and `cam-binder` compile without
  NDK in Termux.

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

### P2b — True Zero-Copy via External Memory (Pending)

For large MIPI Bayer (e.g. 48 MP raw ≈ 100+ MB), binding a host pointer
(`Tensor::buffer().host = ptr`, as `mnn_run_true_zero_copy` now does) is correct on
the CPU path but on a **Vulkan** session still forces MNN to allocate a GPU buffer
and copy host→device — i.e. a heap staging copy we want to eliminate.

- **Correct architecture:** CMA buffer → `mmap` → fd passed to V4L2/MIPI (sensor
  DMA-writes in place) → import that fd into Vulkan as **external device memory**
  and wrap it as an MNN tensor via `Tensor::setDevicePtr(fd, MNN_MEMORY_AHARDWAREBUFFER)`.
  The GPU then reads the sensor's DMA pages directly: no CPU copy, no heap staging.
- **MNN capability today:** `setDevicePtr` + `MNN_MEMORY_AHARDWAREBUFFER (=14)` exist;
  the **OpenCL** backend already imports AHardwareBuffer, but the **Vulkan** backend
  does not consume it yet (no `VkImportMemoryFdInfoKHR` / AHardwareBuffer import in
  `source/backend/vulkan`). Headers (`VK_EXT_external_memory_dma_buf`, `vkGetMemoryFdKHR`)
  and the OpenCL reference impl are present.
- **Plan (enhance our custom MNN fork — feasible):** implement external-memory import
  in the Vulkan backend (mirror `OpenCLBackend.cpp`): in buffer allocation, when
  `mBuffer.flags == MNN_MEMORY_AHARDWAREBUFFER`, use `VkImportMemoryFdInfoKHR`
  (dma-buf) / `AHardwareBuffer` import instead of `vkAllocateMemory`. Then expose a
  new FFI (`mnn_run_external_zero_copy` / extend `mnn_run_true_zero_copy` with an fd +
  memoryType) so `ZeroCopyBufferManager` can hand the V4L2 dma-buf straight to MNN.
- **Enhancement surface:** `cam-isp/mnn_sys/mnn_vulkan_stubs.cpp` holds weak no-op
  Vulkan extension stubs (e.g. `MNNVulkanHotSwapConstBuffer`) that our custom MNN
  build overrides — the natural place to add the import entry point.
- **Status:** investigation complete; implementation deferred (requires building the
  custom MNN Vulkan backend with external-memory support).

### P3 — Hardening / CI ✅ (workspace clean)
- **`cargo clippy --workspace --all-targets --features mnn -D warnings` passes** on all
  7 buildable crates (cam-types, cam-hal, cam-motion, cam-onnx, cam-core, cam-hal-linux,
  cam-app, cam-isp). Zero Rust warnings. Only 2 NDK crates excluded (cam-hal-android,
  cam-binder). ✅
- **19 bridge integration tests** in `cam-binder/tests/bridge_integration.rs` — covers
  `V4l2AidlBridge` (construction, sensor config, synthetic capture, capture loop, stats,
  error display), `HardwareBufferBridge` (pool, formats, acquire/release), and
  `CameraIspService` wiring (mnn-gated). All pass with and without `--features mnn`. ✅

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
