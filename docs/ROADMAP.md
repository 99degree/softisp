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
- **External-memory zero-copy surface (cam-isp, VERIFIED):** added the FFI
  `mnn_run_external_zero_copy` (mnn_wrapper.cpp) using `Tensor::setDevicePtr(fd,
  MNN_MEMORY_AHARDWAREBUFFER)`, the Rust extern + `MNN_MEMORY_AHARDWAREBUFFER`
  const (mnn_sys.rs), `MnnEngine::run_external_zero_copy` (mnnengine.rs), and
  `CameraIspService::process_raw_frame_external` (integration.rs, extracts the
  CMA `dma_fd` from the input buffer and drives the MNN session directly).
  Builds clean and passes `cargo clippy -p cam-isp --lib --features mnn -D warnings`.
  The MNN *backend* import this depends on is still pending (see P2b).
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

### P2b — True Zero-Copy via External Memory (Implemented, unverified)

For large MIPI Bayer (e.g. 48 MP raw ≈ 100+ MB), binding a host pointer
(`Tensor::buffer().host = ptr`, as `mnn_run_true_zero_copy` now does) is correct on
the CPU path but on a **Vulkan** session still forces MNN to allocate a GPU buffer
and copy host→device — i.e. a heap staging copy we want to eliminate.

**Verified call-flow difference (OpenCL vs Vulkan) — 2026-07-15:**
Both backends override `Backend::onAcquire(tensor, storageType)` as the hook where the
tensor's storage buffer is created. After `setDevicePtr(dev, MNN_MEMORY_AHARDWAREBUFFER)`
writes `tensor->buffer().device = dev` + `flags`, the two backends diverge completely:

| Aspect | OpenCL | Vulkan |
|---|---|---|
| Import hook | `onAcquire` → `_allocHostBuffer` (`OpenCLBackend.cpp:747`) | `onAcquire` (`VulkanBackend.cpp:218`) |
| Import API | CL `CL_MEM_ANDROID_AHARDWAREBUFFER_HOST_PTR_QCOM` (Adreno) / `CL_IMPORT_TYPE_ANDROID_HARDWARE_BUFFER_ARM` (Mali) | `VkImportMemoryFdInfoKHR` (Linux dma-buf) / `VkImportAndroidHardwareBufferInfoANDROID` (Android AHB) |
| Platform | **Android AHardwareBuffer only** (`#ifdef __ANDROID__` + `isSupportAHD()`) | external-memory ext enabled per-platform |
| Memory slot | `TensorUtils::setSharedMem(tensor, …)` (per-tensor shared slot) | `tensor->buffer().device = (uint64_t)VulkanBuffer*` |
| Release | `CLSharedMemReleaseBuffer` — does **not** free the AHB (external owner keeps it) | `VulkanMemRelease` — frees through the pool (**wrong** for external) |

**Implication — mirroring OpenCL is NOT a copy-paste.** The Vulkan change must:
1. Branch in `VulkanBackend::onAcquire` on `tensor->buffer().flags == MNN_MEMORY_AHARDWAREBUFFER`.
2. Import the external handle into a `VulkanBuffer` (new import ctor in `VulkanBuffer` /
   `VulkanDevice::allocMemory` via `VkImportMemoryFdInfoKHR`).
3. Use a release path that does **not** free the imported `VkDeviceMemory` (the CMA/dma-buf
   owns it).
4. Cover **both** platforms: Linux V4L2 = dma-buf fd (`VK_KHR_external_memory_fd`);
   Android HAL3 = AHardwareBuffer (`VK_ANDROID_external_memory_android_hardware_buffer`).
   OpenCL gives **no** Linux dma-buf precedent — that path is net-new Vulkan code.

- **Correct architecture:** CMA buffer → `mmap` → fd passed to V4L2/MIPI (sensor
  DMA-writes in place) → import that fd into Vulkan as **external device memory**
  and wrap it as an MNN tensor via `Tensor::setDevicePtr(fd, MNN_MEMORY_AHARDWAREBUFFER)`.
  The GPU then reads the sensor's DMA pages directly: no CPU copy, no heap staging.
- **MNN capability today:** `setDevicePtr` + `MNN_MEMORY_AHARDWAREBUFFER (=14)` exist;
  the **OpenCL** backend already imports AHardwareBuffer, but the **Vulkan** backend
  does not consume it yet (no `VkImportMemoryFdInfoKHR` / AHardwareBuffer import in
  `source/backend/vulkan`). Headers (`VK_EXT_external_memory_dma_buf`, `vkGetMemoryFdKHR`)
  and the OpenCL reference impl are present.
- **cam-isp side — DONE (verified):** FFI `mnn_run_external_zero_copy`, Rust extern
  + `MNN_MEMORY_AHARDWAREBUFFER` const, `MnnEngine::run_external_zero_copy`, and
  `CameraIspService::process_raw_frame_external` (extracts CMA `dma_fd`, binds it via
  `setDevicePtr`, runs the MNN session). Compiles + clippy-clean. It is inert on
  Vulkan today because the backend does not import the fd yet.
- **MNN backend side — implemented in MNN fork (`1b94cc09`, unverified — needs NDK
  rebuild + on-device validation):** mirrors `source/backend/opencl/core/OpenCLBackend.cpp`
  (`_allocHostBuffer`, ~lines 749–1027). Concrete changes committed to the MNN fork:
  1. `VulkanInstance` enables `VK_KHR_external_memory_capabilities` (when available);
     `VulkanDevice` enables `VK_KHR_external_memory_fd` and `createBuffer` now takes a
     `pNext` (for `VkExternalMemoryBufferCreateInfo`).
  2. `VulkanMemory` gained an external wrapper ctor whose destructor does **not** free
     the imported `VkDeviceMemory`.
  3. `VulkanBuffer::createExternal()` creates the `VkBuffer` with
     `VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT` and imports the fd via
     `VkImportMemoryFdInfoKHR`, iterating memory types (preferring device-local) until
     `vkAllocateMemory`+`vkBindBufferMemory` succeed.
  4. `VulkanBackend::onAcquire` detects `mBuffer.flags == MNN_MEMORY_AHARDWAREBUFFER`,
     imports once per fd via `VulkanBuffer::createExternal`, caches the imported buffer
     in `TensorUtils::setSharedMem`, and re-imports when the fd changes between frames;
     `VulkanExternalMemRelease` keeps it alive without freeing the fd.
  Android AHB import is intentionally out of scope (MNN's Vulkan headers lack
  `VkImportAndroidHardwareBufferInfoANDROID`). The external path is gated by the
  `MNN_MEMORY_AHARDWAREBUFFER` flag and never fires in default MNN usage.
- **Verification status:** cam-isp surface verified by `cargo clippy` (no GPU/camera
  needed). The MNN backend change **cannot be built or runtime-tested in Termux**
  (no `cmake`/`ninja`/`vulkan` in PATH; no GPU/V4L2 device) — it must be built on the
  MNN NDK machine and validated on-device (V4L2 dma-buf → Vulkan import → inference).
- **Enhancement surface:** `cam-isp/mnn_sys/mnn_vulkan_stubs.cpp` holds weak no-op
  Vulkan extension stubs (e.g. `MNNVulkanHotSwapConstBuffer`) that our custom MNN
  build overrides — the natural place to add the import entry point.
- **Status:** cam-isp surface verified; MNN Vulkan backend import **implemented** in
  the MNN fork (`1b94cc09`) but **unverified** (no NDK/cmake/Vulkan/device in Termux).
  Remaining: build the custom MNN with external-memory support, copy the `.so` into
  `cam-rust/lib/arm64-v8a/`, and validate on-device (V4L2 dma-buf → Vulkan import →
  inference) before claiming true zero-copy end-to-end.

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
