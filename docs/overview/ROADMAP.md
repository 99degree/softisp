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
  stats switch. Both unrelated `cam-isp --lib` failures are now fixed: the
  forced-input16 head uses a Vulkan-safe FLOAT32 input, and packed-display
  decoding requires a one-channel output tensor. See
  `docs/isp/20261125-cam-isp-lib-failures.md`; the MNN lib suite is now
  856 passed, 0 failed, 2 ignored after rebasing onto the current `master`.
- **Staged frame phases** (this session): `MnnEngine` gained opt-in
  `prepare()` / `run_prepared()` / `finish()` with opaque session tokens, and
  both camera workers now prep before capture, present before deferred stats,
  and fall back on first frame or geometry change. `process()` keeps its old
  single-call contract. See `docs/isp/20261125-staged-frame-phases.md`.
- **External cancellation requested** (this session): a way for outside code
  to stop the pipeline and release camera/MNN/buffer resources cleanly is
  recorded as P2d below; it is not implemented yet.
- **APK build unblocked** (this session): the stale `jni` workspace member was
  removed because no such crate exists in the repository, and the inert
  per-package profile sections in `cli/Cargo.toml` were dropped. `./build-apk.sh`
  now completes cleanly and produces `target/debug/apk/softisp-camera.apk`.
  See `docs/build/20261125-build-apk-workspace-repair.md`.
- **Force-update recovery** (this session): `master` was rewritten upstream
  with a combined IPA/CLI history that no longer contained the earlier
  forced-input16, packed-output, and workspace/build fixes. The rebase restored
  those patches and the two graduated build/ISP analysis docs, so the staged
  work sits on top of a verified baseline rather than a regressed one.
- **Repo resynced and ANW repaired** (this session): all pushed commits were
  rebased onto the current `origin/master`, the missing
  `anw/src/presentation.rs` FFI wiring was repaired, and workspace rustfmt
  defaults were applied in a formatting-only commit. `cargo fmt --all -- --check`,
  the cam-isp/app/anw lint and unit suites, and the real-MNN staged test are
  clean. See `docs/build/20261125-anw-presentation-module.md`.
- **APK rebuilt** (this session): `./build-apk.sh` produced a signed
  `target/debug/apk/softisp-camera.apk`. A packaging audit found that step 3
  still ships the previous app library; see P2e and
  `docs/build/20261125-apk-step3-stale-lib-audit.md`.
- **Branch audit** (this session): all 62 local branches were inspected with a
  strict three-day window. Only five recent patches on `local-fixes` lack an
  upstream equivalent; see P2f and
  `docs/overview/20261125-local-branch-audit.md`.
- **Application code target changed** (user direction, 2026-11-25):
  `android-app/` will hold only non-code resources and packaging metadata; all
  Rust/native application code moves to `anw/`, and NativeActivity loads
  `softisp` (`libsoftisp.so`). Tracked as P2g; P2e remains only the interim
  diagnosis of the current `libcam_camera_app.so` packaging bug. See
  `docs/ipa/ANW_SPLIT_PLAN.md`.
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

### P2c — Staged prep / infer / post frame phases ✅ (implemented 2026-11-25)

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
- **Status:** implemented for the engine API and camera workers. Default
  `process()` remains unchanged; the real-MNN staged streaming test passes.
  Production preview latency has not yet been re-measured with a fresh APK, and
  the pre-bound camera input-buffer refinement remains dependent on P2b. See
  `docs/isp/20261125-staged-frame-phases.md`.

### P2d — External pipeline cancellation and clean shutdown (TODO)

- **User request recorded 2026-11-25.** Add a mechanism that lets code outside
  the camera/ISP workers issue a stop request to the running pipeline and have
  every worker wind down deterministically, releasing camera, MNN session,
  buffer-pool and staged-frame resources cleanly.
- **Surface:** an idempotent shared handle (e.g. `PipelineHandle::cancel()` or
  a `CancellationToken`) exposed through the app/service/JNI boundary. It must
  not depend on dropping the worker threads or killing the process.
- **Worker contract:** observe cancellation at safe points (before preparing a
  frame, immediately after capture, after handing a finished frame to the
  presenter and after deferred stats finish). Stop taking new frames, drain or
  finish in-flight work, and join workers without leaving poisoned mutexes.
- **Staged-path constraint:** a cancel request must not drop a live
  `PreparedFrame` or `PendingStats` session guard while MNN/Vulkan may still
  read its buffers. Either let the in-flight frame finish, or add an explicit
  await-release path before tearing down the engine.
- **Verification:** hermetic tests for idempotent cancel, worker exit, and no
  leaked session/buffer reservations; on-device check that a cancel followed
  by restart leaves no stale camera device or Vulkan buffers.

### P2e — APK step 3 ships the previous app library (TODO)

- **Status:** reproduced on the 2026-11-25 build. `build-apk.sh` patches and
  copies `libcam_camera_app.so` into `cam-rust/lib` before `cargo apk2 build`,
  and `cargo-apk2` packages that stale copy. The APK entry SHA-256 matched the
  25,763,641-byte pre-build library, not the freshly linked 113,719,008-byte
  one. Details and hashes: `docs/build/20261125-apk-step3-stale-lib-audit.md`.
- **Fix:** build the cdylib first, copy it into both ABI runtime dirs, run
  `patchelf` on the fresh copies, then `cargo apk2 build`; verify that the
  target, runtime copy, and APK entry hashes all match.
- **Impact:** current Rust/app changes may be absent from the shipped APK even
  though the build reports success. The second consecutive build would pick up
  the previous build's library.
- **Scope:** this is an interim fix for the current tree only. The intended
  architecture is P2g, where `libcam_camera_app.so` is retired and the APK loads
  a freshly built `libsoftisp.so`.

### P2f — Recent local branch patches without upstream equivalents (TODO)

- **Status:** five commits on `local-fixes` dated 2026-11-23/24 have no
  equivalent patch on `origin/master`; every other branch with commits in the
  strict three-day window is patch-equivalent upstream or is the current
  `master`. Details: `docs/overview/20261125-local-branch-audit.md`.
- **Patches:** `04e87a9e` (MNNConvert CLI build), `ea139531` (mnn-gated
  cleanups), `a796111e` (runtime_libs path), `72ad0292` (cam-app → cam-disp /
  android-app docs), `2b796345` (AppCompat theme / adaptive icons).
- **Action:** review each against the current renamed workspace before any
  cherry-pick; older-than-three-day changesets and all agent worktrees were
  explicitly excluded from triage.

### P2g — Move all app code into `anw/`; make `android-app/` resource-only (TODO)

- **User direction recorded 2026-11-25.** `android-app/` holds non-code APK
  resources and packaging metadata only. All Rust/native application code lives
  in `anw/`, whose `softisp` cdylib (`libsoftisp.so`) is loaded by
  NativeActivity.
- **Library name:** `libsoftisp.so` is the preferred filename. Existing
  `softisp_anw_*` C ABI symbol names remain unchanged for compatibility.
- **Target split:**
  - `android-app/`: `AndroidManifest.xml`, `Sources/res/**`, cargo-apk2 metadata
    pointing at those resources; no camera/ISP/pipeline/storage/crash-log code.
  - `anw/`: `android_main`, `camera.rs`, `crashlog.rs`, `pipeline.rs`,
    `pipeline_runner.rs`, `storage.rs`, `presentation.rs`, the existing raw-frame
    C ABI, and the Android linker build script.
- **Implementation steps:**
  1. Move the `android-app/src/**` modules into `anw/src/**` and adapt imports,
     crate paths, and feature gates from `cam-camera-app` to `softisp-anw`.
  2. Move `android-app/build.rs` linker setup to `anw/build.rs`; remove the Rust
     library target and `src/**` from `android-app`.
  3. Repoint `anw/Cargo.toml` packaging at `../android-app/AndroidManifest.xml`
     and `../android-app/Sources/res`, keep package identity
     `com.softisp.camera`, set `android.app.lib_name` to `softisp`, and keep
     `runtime_libs = "../cam-rust/lib"`.
  4. Change `build-apk.sh` to build/package `softisp-anw`; delete the stale
     `libcam_camera_app.so` copy/patch step and apply the libc++ `patchelf`
     rewrite to a freshly built `libsoftisp.so`.
  5. Retire `libcam_camera_app.so` references in docs, CI, and runtime dirs, and
     update the P4 crate table accordingly.
- **Behaviour constraints:** preserve `OutputFormat::PackedRgb`, final
  `isp.display_uint32_bgra`, dual-engine execution, staged frame phases, no
  float fallback, AHB only for large tensors, and the existing crash-log path.
- **Verification:** workspace format/clippy/tests for `anw`; successful
  `./build-apk.sh`; APK contains `lib/arm64-v8a/libsoftisp.so` and no
  `libcam_camera_app.so`; target/runtime/APK hashes match; device logs show
  camera startup, dual-engine ISP, and preview from `anw`. Assistant must not
  install or share the APK. Full plan: `docs/ipa/ANW_SPLIT_PLAN.md`.

### P4 — Multi-Frontend IPA Plugin Architecture ✅ (implemented 2026-11-25)

New crates added to the workspace:

| Crate | Type | Path | Purpose |
|-------|------|------|---------|
| `cam-ipa` | rlib | `cam-rust/cam-ipa/` | Stable Rust IPA API — `IpaContext`, `EngineManager`, `IpaController`, `IpaPipeline`, `BufferManager`, `EventEmitter` |
| `libcamera-ipa` | rlib+cdylib | `libcamera-ipa/` | libcamera C FFI plugin — exports `ipaCreate()` / `IPAModuleInfo`, implements `IPASoftIspInterface` vtable |
| `jni` | rlib+cdylib | `jni/` | Android JNI bridge — `JNI_OnLoad`, `IpaSession` native peer, `HardwareBuffer` → `AhwBuffer` |
| `anw` | rlib+cdylib | `anw/` | ANativeWindow / application code owner after P2g (currently presentation + cam-ipa consumer) |
| `cli` | rlib+cdylib+bin | `cli/` | 4K frame generator, 3A + neural processor, V4L2 capture, training-data prep |

**Architecture** (`docs/ipa/MULTI_FRONTEND_IPA_ARCHITECTURE.md`):

```
┌─────────────────────────────────────────────────────────────────┐
│                      cam-ipa (CORE, rlib)                       │
│  IpaContext (lifecycle) │ EngineManager (MnnEngine) │ IpaController │
│  IpaPipeline (blocks)   │ BufferManager (zero-copy) │ EventEmitter │
│                         │ NeuralController (isp-rectifier, opt-in)│
└──────────────────────────┬──────────────────────────────────────┘
         ┌─────────────────┼─────────────────┬─────────────────────┐
         ▼                 ▼                 ▼                     ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐ ┌─────────────┐
│libcamera-ipa  │ │     jni       │ │     anw       │ │     cli     │
│  (cdylib)     │ │   (cdylib)    │ │   (cdylib)    │ │ (cdylib+bin)│
│               │ │               │ │               │ │               │
│• IPAInterface │ │• JNI_OnLoad   │ │• ANativeWindow│ │• 4K frame   │
│• IPASoftIsp   │ │• Java API     │ │• BufferQueue  │ │• 3A + neural│
│• SharedFD→dmab│ │• AHardwareBuf │ │• EGL/GPU      │ │• V4L2 capt. │
│   uf→DmaBuf   │ │               │               │ │• training prep│
└───────────────┘ └───────────────┘ └───────────────┘ └─────────────┘
```

**cam-ipa public API:**
- `IpaConfig` / `IpaSettings` / `IpaSensorInfo` / `IpaFormat` — configuration types
- `IpaContext::new/init/start/stop/configure/process_frame/process_stats` — lifecycle + processing
- `Buffer` trait (zero-copy) with `DmaBufBuffer` (Linux dmabuf/memfd mmap + Android SharedFD), `AhwBuffer` (Android AHB), `MemBuffer` (testing)
- `ControlList` / `ControlInfoMap` — serializable control exchange
- `IpaCallbacks` — `metadata_ready`, `frame_done`, `set_sensor_controls`, `set_isp_params`

**Neural processor (opt-in):**
- `cam_isp::neural_controller::NeuralController` (isp-rectifier distilled ONNX + rule-based fallback)
- Enable via `IpaConfig.neural_enabled = true` + `SOFTISP_MODEL_DIR` env var
- Falls back to rule-based `IspController` if model unavailable

**libcamera-ipa FFI exports:**
- `IPA_MODULE_INFO` static (`moduleAPIVersion=1`, `pipelineName="softisp"`, `name="SoftISP"`)
- `ipa_create()` → `*mut IPAInterface`
- VTable implementing `IPASoftIspInterface` (init/start/stop/configure/configure_frame_backend/queue_request/compute_params/process_stats/process_frame)
- `SharedFD` → `RawFd` → `DmaBufBuffer` mapping for zero-copy buffer passing

**cli capabilities:**
- 4K (3840×2160) synthetic 16-bit Bayer frame generator (default)
- V4L2 capture via `cam-hal-linux` (feature `v4l2`, `rscam`) — real frames from `/dev/video*`
- Full ISP pipeline processing via `MnnEngine` Vulkan backend
- 3A (AWB/AE/CCM/tone) via `IspController`; neural via `NeuralController`
- Zero-copy: Linux `memfd_create` + mmap (`DmaBufBuffer`); Android `SharedFD` / `AhwBuffer`
- Pipeline profile selection: `--profile {lite,med,fused,heavy,pro,test,unified,hdr,yuv}` per subcommand
- Profile JSON dump/load: `profile-json --name fused --output p.json`, `generate --profile-json p.json`
- CMOS character dump: `cmos-dump --device /dev/video0 --output cmos.json` (libcamera-compatible JSON)
- Output BGRA frame to `output_frame.bgra`; per-frame metadata via callbacks
- Polyglot ELF: runs directly (`./libcam_ipa_test_cli.so`) and dlopen-able (exports `ipa_test_main`, `my_entry`)

**Integration points (reused from cam-isp):**
- `MnnEngine` / `DualGpuEngine` — Vulkan GPU inference backend
- `IspController` — 3A (AWB/AE/CCM/tone/scene)
- `NeuralController` — neural + fallback (isp-rectifier)
- `PipelineBuilder` — standard ISP block chain (unpack → demosaic → tone → denoise → display)
- `ProcessParams` — zero-copy input/output buffers + AHB import for Vulkan
- `cam-hal-linux` — V4L2 capture (rscam)

**Verification status:**
- `cargo build -p cam-ipa` — compiles (rlib)
- `cargo build -p libcamera-ipa` — compiles (rlib+cdylib)
- `cargo build -p cam-ipa-sk` — compiles (rlib+cdylib+bin)
- `cargo build -p softisp-jni` / `softisp-anw` — compile-stubbed (Android target only)
- Not yet runtime-tested against libcamera pipeline handler (requires libcamera-99degree fork build)

**Next steps:**
- Runtime test: load `libipa_softisp.so` in libcamera-99degree pipeline handler
- End-to-end frame processing: stats → params → frame output
- CLI end-to-end: V4L2 capture → ISP process → output frame + stats for training
- JNI frontend wired to android-app crate
- ANativeWindow frontend wired to display surface

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
