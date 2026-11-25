# Project Roadmap & Outstanding Work

> Last updated: 2026-11-25. See [`AGENTS.md`](../AGENTS.md) §14 for the canonical pointer.

## Status (2026-11-25) — packed-preview perf work

- **Runtime tensor uploads de-duplicated** (`d378a372`). `set_extra_inputs()`
  queued ~60 writes/frame through `copyFromHostTensor`; ~18 of them were
  defaults immediately overwritten by explicit values. Writes are now buffered
  by name and flushed once with the final value. Device measurement before the
  change: 173 ms of 311 ms/frame was tensor upload latency.
- **Pipelined dual engine** (this session): the app loop runs two ISP workers,
  one pinned per Vulkan engine, with sequence-numbered results reordered on the
  presenting thread, so camera capture/staging overlaps inference.
- **Per-phase perf probe** (this session): `probe_4k_perf` now aggregates prep,
  infer, post and total durations, and its ignored MNN test links on Android
  because `cam-isp/build.rs` always emits the NDK system libraries. The false
  packed-u8 output-channel warning and the probe's packed-byte blank-frame check
  are fixed. It is split into per-profile ignored tests (`perf_4k_fused`,
  `perf_4k_med`, `perf_4k_heavy`, `perf_4k_whole`, `perf_4k_unified`,
  `perf_4k_dual`); `perf_4k` remains the FUSED-only breakage check and
  `perf_4k_all` is the explicit full matrix. See
  `docs/isp/20261125-perf-phase-instrumentation.md`.
- **Stats readback no longer paid twice** (this session): both app workers now
  share one `Arc<Mutex<IspController>>` via `MnnEngine::share_controller()`,
  engine A keeps stats on, and engine B calls `set_stats_enabled(false)` before
  workers start. `DualGpuEngine` delegates `controller()` to engine A instead of
  carrying a second unused instance. Unit tests cover shared handles and the
  stats switch; two unrelated `cam-isp --lib` failures remain unchanged.
- **HEAVY MNN test repaired** (this session): `test_mnn_engine` now fills the
  new zero-copy `ProcessParams` fields via `ProcessParams::new()`, passes the
  profile's real head block for pre-converted models, uses the real sensor
  height, and derives expected frame bytes from `IspFrame` geometry/format.
  HEAVY converts 91 ops, runs, and returns a valid frame; a standalone ignored
  `test_mnn_heavy_profile` was added. PRO still fails conversion on unsupported
  ops and is tracked separately.
- **P2b re-verified against the current MNN tree:** the AHB `setDevicePtr`
  surface in cam-isp is a stub — Vulkan `onAcquire` ignores the flag and
  `mnn_run_true_zero_copy` binds host pointers Vulkan also ignores. The
  in-repo correctness bugs (input tensor name, discarded FFI returns) are
  fixed; the APK is deliberately **not** wired to it. Full end-to-end remains
  blocked on camera AHB lifetime + an output-surface AHB design (see P2b).
  The MNN Vulkan backend no longer uses a process-wide published AHB: AHB
  import is per-tensor via `setDevicePtr(..., MNN_MEMORY_AHARDWAREBUFFER)` and
  `TensorUtils::sharedMem`, so two workers can bind different buffers without
  serialization. Rebuilt via `scripts/build_mnn_libs.sh arm64-v8a`.

## Status (2026-07-21) — MNN Dead-Code Elimination Fixed

- **Root cause of impossibly-fast MNN inference identified and fixed.**
  MNN was DCE-ing the **entire 20-block HEAVY ISP pipeline** to identity
  (≈ 0.1 ms/frame, ≈ 7151 FPS) for two independent reasons:
  1. **`Proto::tensor_type()` (proto.rs) encoded `TypeProto` wrong.**
     It emitted `field 1 = value_type=1` (a varint) plus
     `field 2 = TensorTypeProto` (submessage). ONNX `TypeProto` is actually
     a `oneof value { Tensor tensor_type = 1; … }` — i.e., field 1 IS the
     `TensorTypeProto` submessage (length-delimited). The shape bytes were
     also wrongly wrapped in an extra `raw_bytes(1, ...)`, merging all
     `Dimension` submessages into one and collapsing e.g. `[1, 267]` to `[10]`.
     Fix: emit `raw_bytes(1, &tensor_type)` directly, with `tensor_shape` as
     the joined `Dimension` submessage bytes (no outer wrap).
  2. **`WarpGridBlock` grid + lens-shading LUT were ONNX initializers.**
     MNN saw constant grid/LUT data and folded the warp into identity. Both
     are now runtime `extra_inputs()` (graph input ports fed by
     `set_extra_inputs()` at frame time) — matching how CCM/tone/AWB already
     override initializers, and keeping data-dependent paths alive.
- **`synth_bayer` module added** — 5 synthetic Bayer pattern generators
  (constant, rainbow, maze, color-diffuse, stress) with high spatial
  frequency so MNN cannot mark part of the graph as constant.
- **cam-isp tests:** **732** lib tests pass with `--features rectifier`, 0 failures.
- **4K→4K MNN stress tests:**
  - HEAVY: **79.8 FPS** avg (2394 frames / 30 s, P99 129.1 ms).
  - UNIFIED (tiled 2×2): **141.2 FPS** avg (4236 frames / 30 s, P99 8.7 ms).
- **Profile defaults:** `HEAVY.use_saturation = false` (SaturationBlock
  has no MNN Extra runtime executor and raised `NOT_SUPPORT`).

## Status (2026-07-15)

- **cam-isp tests:** **813** lib tests pass with `--features mnn`, **0 failures, 0 warnings**
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
- **External-memory zero-copy surface (cam-isp): REMOVED 2026-11-15.** The FFI
  `mnn_run_external_zero_copy` (mnn_wrapper.cpp), the Rust extern +
  `MNN_MEMORY_AHARDWAREBUFFER` const (mnn_sys.rs), `MnnEngine::run_external_zero_copy`
  (mnnengine.rs) and `CameraIspService::process_raw_frame_external` (integration.rs)
  were removed because they could not work against the MNN build we ship, and had no
  callers outside that chain. `Tensor::setDevicePtr` only stores the raw handle
  (`mBuffer.device = (uint64_t)ptr; mBuffer.flags = memoryType;`), while the Vulkan
  backend unconditionally does `reinterpret_cast<VulkanBuffer*>(tensor->deviceId())`
  and dereferences it — so passing an fd/AHB pointer there is a type-confused
  dereference, not an import. Nothing in `source/backend/vulkan/` reads
  `buffer().flags`, and the shipped `libMNN_Vulkan.so` exports neither
  `VulkanBuffer::createExternal` nor any AHB import entry point. See P2b for what a
  working implementation actually requires.
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

### P2b — True Zero-Copy via External Memory (not implemented in shipped MNN)

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
- **cam-isp side — REMOVED (2026-11-15):** the FFI `mnn_run_external_zero_copy`,
  the Rust extern + `MNN_MEMORY_AHARDWAREBUFFER` const, `MnnEngine::run_external_zero_copy`
  and `CameraIspService::process_raw_frame_external` were deleted. They were inert at
  best and unsafe at worst (see the Status entry above); the surface should be rebuilt
  against the backend work below rather than kept as a stub.
- **MNN backend side — NOT PRESENT in the checked-out MNN tree (verified 2026-11-15):**
  `grep -rn 'createExternal' ~/MNN/source/backend/vulkan/` returns nothing, and nothing
  under `source/backend/vulkan/` reads `buffer().flags` or `MNN_MEMORY_AHARDWAREBUFFER`.
  The shipped `libMNN_Vulkan.so` correspondingly exports no such symbol. The plan below
  is therefore still a plan, not a revert-to-working:
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
- **Verification status:** nothing on the cam-isp side remains to verify — that surface
  was removed (2026-11-15). The MNN backend work **cannot be built or runtime-tested in
  Termux** (no `cmake`/`ninja`/`vulkan` in PATH; no GPU/V4L2 device); it must be built
  on the MNN NDK machine and validated on-device (V4L2 dma-buf → Vulkan import →
  inference).
- **Enhancement surface:** `cam-isp/mnn_sys/mnn_vulkan_stubs.cpp` holds weak no-op
  Vulkan extension stubs (e.g. `MNNVulkanHotSwapConstBuffer`) that our custom MNN
  build overrides — the natural place to add the import entry point.
- **Status:** not implemented end-to-end. The cam-isp Rust surface was removed rather
  than kept as a stub that cannot work; the MNN Vulkan backend import has to be written
  (item 1–4 above), the custom `.so` built and copied into `cam-rust/lib/aarch64-v8a/`,
  and the Rust surface reintroduced only once the backend honours the flag.
- **Correction (2026-11-25):** `ProcessParams::{input_ahb,output_ahb}` and
  `mnn_import_ahb_as_device_memory()` are still present in the tree (the 2026-11-15
  removal above applied to a different FFI surface, `mnn_run_external_zero_copy`).
  They are inert for the APK — no caller sets the fields, `zero_copy` is gated on
  `output_buffer`, the two imports both bind `output_name` (the input should use
  `self.frame_input_name`), and the FFI return codes are discarded. The
  Vulkan `onAcquire` limitation is unchanged. In-repo correctness bugs were fixed;
  the APK is deliberately left on the working copy-in/copy-out path.

### P2c — Staged prep / infer / post frame phases (TODO)

- **User idea recorded 2026-11-25.** Present the output frame as soon as
  inference finishes, then perform 3A stats readback on the worker; also do the
  next frame's controller/metadata tensor preparation while the camera is
  still filling its buffer. The previous controller values are already the ones
  the arriving frame consumes, so this is semantically the same input as today.
- **Plan:** add opt-in staged APIs on `MnnEngine` (`prepare`, `run_prepared`,
  `finish`) with opaque session tokens, keep the camera mutex limited to
  `next_frame()`, and fall back to today's inline `process()` on first frame or
  geometry change. Existing `IspAuxOutput::calibration_stats` callers must keep
  the current behaviour by default.
- **Buffer ownership refinement (implementation stage):** allocate and bind the
  inference input buffer/AHB before capture and hand the same pointer to camera2
  as the capture target. Camera fill then overlaps prep and the previous frame;
  "buffer ready → `runSession`" is the critical path, with no input copy. Exact
  AHB/ImageReader ownership, usage bits, fences, and lifetime are to be decided
  at implementation time and depend on P2b. Never reuse a buffer the GPU still
  reads.
- **Estimate:** can hide roughly 58 ms of pre-inference tensor work and 79 ms
  of GPU→CPU stats readback from the visible frame path; camera2 filling a
  pre-bound buffer also removes the input wait/copy from the inference path.
  `runSession` (~112 ms) remains the floor. It is a latency win, not an
  inference-speed change.
- **Status:** not implemented. This is a larger API-surface follow-up than the
  A-on/B-off stats split and is documented in
  `docs/isp/20261125-perf-phase-instrumentation.md`.

### P3 — Hardening / CI ✅ (workspace clean)
- **`cargo clippy --workspace --all-targets --features mnn -D warnings` passes** on all
  7 buildable crates (cam-types, cam-hal, cam-motion, cam-onnx, cam-core, cam-hal-linux,
  cam-app, cam-isp). Zero Rust warnings. Only 2 NDK crates excluded (cam-hal-android,
  cam-binder). ✅
- **19 bridge integration tests** in `cam-binder/tests/bridge_integration.rs` — covers
  `V4l2AidlBridge` (construction, sensor config, synthetic capture, capture loop, stats,
  error display), `HardwareBufferBridge` (pool, formats, acquire/release), and
  `CameraIspService` wiring (mnn-gated). All pass with and without `--features mnn`. ✅

### P3 — Follow-up: unified runtime tensor naming

- **Survey and normalize `extra_inputs()` tensor suffixes across all blocks.**
  The WB path currently has three names for one decision:
  `BayerWbBlock/gains` is the logical 3A input, `DemosaicCcmBlock/w` is the
  fused coefficient matrix that actually consumes it, and callers writing
  `ProcessParams::bayer_gains` use a third spelling. Keep the current names for
  this perf/WB work; do the rename as one atomic follow-up with compatibility
  aliases so existing MNN models and probes keep loading.
- **Use `docs/reference/TensorNames.java` as the canonical naming reference.**
  It already defines `BAYERWBBLOCK/BAYER_GAINS` for the logical `[4]` FLOAT WB
  gains and `DEMOSAICCCMBLOCK/W` / `DEMOSAICCCMBLOCK/B` for the fused matrix and
  bias. That supports the user's request for a semantic WB name, but points to
  `BAYER_GAINS` (or `AWB_GAINS` elsewhere) rather than inventing `wb_gain`, and
  it confirms that `W` is intentionally generic: it is the fused demosaic+CCM
  weight matrix, not a WB gain vector. A future Rust rename should converge on
  that registry, including `DemosaicBlock/w`, `CcmBlock_ccm/matrix`, shader
  bindings, `RUNTIME_PARAMS.md`, and the ignored `probe_wb_fused` comparison.
  The Java file is generated from `TensorNames.kt`; if the registry itself must
  change, update the Kotlin source and regenerate the Java wrapper.

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
| `docs/reference/TensorNames.java` | Canonical runtime tensor-name registry for future renames |
| `docs/overview/ROADMAP.md` | This document |
