# Project Roadmap & Outstanding Work

> Last updated: 2026-11-26. See [`AGENTS.md`](../AGENTS.md) §14 for the canonical pointer.

## Status (2026-11-26) — packed-preview perf work

- **Output AHB zero-copy path landed, on by default** (2026-11-26). Each ISP
  worker owns one large, permanently mapped `AHardwareBuffer` bound to the
  session output tensor before resize; `mnn_run_ahb_output()` waits for the GPU
  write and returns with no `copyToHostTensor`. `IspFrame::new_external()`
  removes the per-frame `width * height * 4` host allocation. Disable without a
  rebuild with `SOFTISP_OUTPUT_AHB=0` (also `off`, `false`, `no`),
  `SOFTISP_OUTPUT_AHB_OFF`, or `/sdcard/download/softisp-output-ahb-off`;
  `SOFTISP_VERBOSE` / `/sdcard/download/softisp-verbose` adds per-frame slot
  traces. No cross-engine wait: a worker only waits for its own presenter
  handoff. The output slot is now sized for the fused graph's `H*W*16` worst
  case, the C wrapper verifies a real AHardwareBuffer and rejects any in-run
  resize that would re-acquire the output tensor, and `-10` logs the tensor
  flags/device, AHB capacity and host-pool status before falling back. Signed
  default-on APK rebuilt with the rotation and binding fixes; on-device
  relaunch/log evidence is still pending. Ownership analysis:
  `docs/mnn/20261126-mnn-output-tensor-buffer-ownership-ahb-feasibility.md`.
- **Output rotation fixed in the fused graph** (2026-11-26): the fused display
  shader now writes an intermediate tensor and the graph's `Transpose` / `Slice`
  nodes terminate at `DisplayBlock/frame`, so host-copy and AHB paths see the
  same upright pixels. No standalone SPIR-V shader was added. Regression tests
  cover the no-op and every rotate mode.
  `docs/isp/20261126-default-on-ahb-rotation-report.md`.
- **Hermetic test warnings cleaned** (2026-11-26): stale `cam-ipa` scaffolding
  and `upload_cache` switch helpers were fixed at the root instead of
  suppressed; `AhwBuffer::new()` now rejects geometry/format mismatches.
  `docs/isp/20261126-cargo-test-warning-cleanup.md`.
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
- **External cancellation implemented** (2026-11-26): `PipelineHandle` is a
  cloneable, idempotent stop signal exposed through the `anw` C ABI; the activity
  loop joins workers, clears camera/reorder state, releases the surface and
  exits. Hermetic `softisp-anw` tests are 49/49 and `cam-isp rectifier` is
  831 passed/2 ignored on the current tree. See P2d and
  `docs/isp/20261126-p2d-cancellation.md`.
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
- **P2f patches reviewed** (2026-11-26): three patches were already applied or
  superseded, one stale docs patch was adapted to the `anw/` layout, and one
  Android resource patch was skipped as unrelated churn. See P2f and
  `docs/overview/20261126-p2f-patch-review.md`.
- **Application code moved to `anw/`** (P2g, 2026-11-25): `android-app/` now holds
  only the manifest and `Sources/res`; `anw/` owns `android_main`, camera
  capture, pipeline, presentation, storage, crash logging, and the raw-frame C
  ABI. NativeActivity loads `softisp` (`libsoftisp.so`). The stale
  `libcam_camera_app.so` packaging path is retired. See
  `docs/ipa/ANW_SPLIT_PLAN.md`.
- **HEAVY MNN test repaired** (this session): `test_mnn_engine` now fills the
  new zero-copy `ProcessParams` fields via `ProcessParams::new()`, passes the
  profile's real head block for pre-converted models, uses the real sensor
  height, and derives expected frame bytes from `IspFrame` geometry/format.
  HEAVY converts 91 ops, runs, and returns a valid frame; a standalone ignored
  `test_mnn_heavy_profile` was added. PRO still fails conversion on unsupported
  ops and is tracked separately.
- **P2b re-verified against the current MNN tree:** the fork's Vulkan backend
  imports per-tensor AHBs via `setDevicePtr(..., MNN_MEMORY_AHARDWAREBUFFER)`
  and `VulkanBufferFromAHB`, so two workers can bind different buffers without
  serialization. Rebuilt via `scripts/build_mnn_libs.sh arm64-v8a`. The older
  claim that Vulkan ignores the flag applied to the vendored headers-only tree,
  not the linked fork. The APK is now wired to an **on-by-default** output AHB
  slot; opt out with `SOFTISP_OUTPUT_AHB=0` / `SOFTISP_OUTPUT_AHB_OFF` /
  `/sdcard/download/softisp-output-ahb-off`. The host-copy fallback remains for
  API < 26, failed import, and non-AHB callers. See
  `docs/isp/20261126-output-copy-zero-copy-audit.md`.
- **P2h overlap / allocation fast path recorded** (2026-11-26): extend the
  staged phases so camera DMA fills a pre-bound input AHB while the next frame's
  metadata tensors are prefilled, and hand the output AHB to Android for display
  while stats readback updates the next controller state. Also audit buffer
  zeroing as a warm-up cost and add explicit AHB fence tokens. Tracked as TODO
  below; depends on P2b and on-device fence/lifetime validation.
- **P2h fence foundation landed** (2026-11-26): generation-tagged fence tokens
  now travel through `ProcessParams`, `cam-ipa` owns sync fds and AHB slot
  generations, and the Android HAL passes acquire fds into
  `AHardwareBuffer_lock` and captures release fds from
  `AHardwareBuffer_unlock`. `run_prepared()` rejects cross-generation AHB
  bindings. The non-Android dmabuf/sync_fd equivalent is a separate TODO below.
  Verified with fmt, workspace check, canonical clippy, 2 `cam-types` / 3
  `cam-hal` / 2 `cam-onnx` / 6 `cam-motion` / 10 `cam-ipa` / 840 `cam-isp` /
  49 `softisp-anw` hermetic tests and 875 real MNN tests. On-device logs after
  the P2h APK showed no fatal/panic/kill; one recoverable camera error 4 was
  followed by session-ready and continued frames.
- **Critical-path phase instrumentation** (2026-11-26): `IspFrame::aux.timing`
  now carries setup, tensor-upload, inference, output and deferred-readback
  durations, and `probe_4k_perf` prints the visible split with a focused
  `perf_critical_path` case. See
  `docs/isp/20261126-critical-path-instrumentation.md`.
- **Upload path shortened** (2026-11-26): `20dbcfe1` caches the last successful
  value per session slot so unchanged tensor writes stop paying a host→device
  barrier, and `09c95a62` recycles per-slot host scratch buffers so fixed-shape
  coefficients (warp grid/LUT) are not reallocated each frame. The dedup is now
  **off by default** after a blue-frame report: `SOFTISP_CACHE_ON` /
  `/sdcard/download/softisp-cache-on` brings it back without a rebuild. Host
  scratch reuse stays active either way. A follow-up capture with the
  always-upload build stayed normal for 170 sampled frames while 3A and a
  recoverable camera error still occurred, so the default stays off. See the
  critical-duration backlog below.
- **Output metadata cached on the visible path** (2026-11-26): `prepare()` now
  stores the output shape/type-derived layout and the tensor-name `CString`s in
  `PreparedMeta`, so `run_prepared()` no longer repeats those FFI reads and
  allocations between `runSession` and presentation. Three pure layout tests
  pass with real MNN enabled.
- **P3 WB/CCM tensor names normalized** (2026-11-26): canonical registry names
  are emitted by the WB/CCM blocks, with legacy aliases accepted by `MnnEngine`.
  Verified with 851 rectifier / 886 real-MNN tests and three ignored MNN probes;
  see `docs/isp/20261126-p3-runtime-tensor-names.md`.

## Critical-duration optimization backlog (2026-11-26)

`perf_critical_path` defines the visible frame as
`camera fill → prepare/upload → runSession → output handoff`, with 3A readback
deferred onto the worker. `runSession` is the irreducible floor; everything
around it can be overlapped, deduplicated or removed. Ranked by expected payoff
at 4K:

| # | Change | Phase attacked | Status |
|---|--------|----------------|--------|
| 1 | Hand the finished frame to the compositor through `ProcessParams::output_buffer` / AHB import instead of `buf_bytes.to_vec()` | ~33 MB host copy plus present blit on the visible path | PARTIAL — engine-side copy removed by default and output AHB binding hardened; compositor-level handoff and blit removal still TODO (P2h) |
| 2 | Extend stats gating from "engine A only" to a 2nd/4th-frame cadence | deferred GPU `_finish()` sync | TODO — 3A does not need fresh stats every frame |
| 3 | Bake unchanged `default_inputs` into the graph as ONNX/MNN initializers | the single upload per slot on first use | TODO — needs fused-Vulkan conversion validation; never make the warp grid/LUT constant (see the 2026-07-21 DCE regression) |
| 4 | Cache `get_output()` shape/type/channel derivation and `CString` names in `PreparedMeta` | per-frame metadata reconstruction | DONE — `docs/isp/20261126-output-meta-cache.md` |
| 5 | Release the buffer-pool guard before `runSession`; reacquire only for the output copy | lock contention between workers | TODO — helps concurrency, not single-frame latency |
| 6 | Resize/warm every session slot once at startup | first-frame stalls after a geometry change | TODO |
| 7 | Bind the camera buffer directly instead of `FrameStager`'s CPU copy | input staging | TODO — P2b; the `RawInt` path is already zero-copy from `p.input_buffer` |
| 8 | Lower preview resolution, fp16 storage, or a lighter profile (`MED`/`LITE`) | `runSession` itself | TODO — algorithmic, not plumbing |
| 9 | Raise the session pool above two slots | `acquire()` blocking when more than two frames are in flight | Deferred — one-frame-ahead prep is already covered; extra slots cost Vulkan memory and a first-frame resize (four slots at 5 Mpx previously got the process killed) |
| 10 | Copy the stats output into the next metadata input device-to-device | host round-trip | Rejected as stated — MNN tensor roles are fixed by the graph; output→host→input costs two copies plus a sync, and Vulkan `onCopyBuffer` still calls `_finish()` |
| 11 | Ping-pong the output AHB of frame N into the input of frame N+2 behind fences | both copies on GPU | Blocked — needs the P2b/P2h fence lifecycle; the acquire fd is not yet waited by the Vulkan path |

Landed in this window:

- **Per-slot last-value upload dedup** (`20dbcfe1`): unchanged scalars, arrays
  and raw bytes are not re-uploaded when enabled; pooled sessions stay
  independent and `resizeSession` clears the cache. Default off — see the
  blue-frame investigation linked below.
- **Per-slot host scratch pool** (`09c95a62`): `WarpGrid/grid` and the other
  fixed-shape coefficients are built in a recycled `Vec`, so steady-state
  frames allocate no host buffer for them.
- **Phase instrumentation** (`6dd6cf4e`, `fb8cd119`) and one-frame-ahead
  `prepare()` already hide upload work while the sensor fills the next buffer.

Analysis lives in `docs/isp/20261126-upload-dedup-critical-path.md`,
`docs/isp/20261126-output-meta-cache.md` and
`docs/isp/20261126-critical-path-instrumentation.md`. Validate every change with
the ignored `perf_critical_path` probe (`PERF_FRAMES=40`, `--features mnn`) and
compare the prefill, inference and output columns. Hermetic gate at this point:
fmt, canonical clippy, 854 passed / 2 ignored
`cam-isp --lib --features rectifier` tests, 892 passed / 2 ignored with real
MNN, `cargo check --features "rectifier mnn"`, and
`./build-apk.sh` (signed APK with the default-on AHB output path).

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

### P2d — External pipeline cancellation and clean shutdown ✅

- **User request recorded 2026-11-25.** Add a mechanism that lets code outside
  the camera/ISP workers issue a stop request to the running pipeline and have
  every worker wind down deterministically, releasing camera, MNN session,
  buffer-pool and staged-frame resources cleanly.
- **Implemented 2026-11-26.** `PipelineHandle` is a cloneable `Arc<AtomicBool>`
  signal with idempotent `cancel()` and `is_cancelled()`. `PipelineRunner`
  exposes `handle()` and `cancel()`; cancellation sets the flag, joins both
  workers exactly once, drains the reorder queue, and clears the camera session.
  `Drop` routes through the same path.
- **Surface:** registered in an `anw` crate-level handle registry with C ABI
  `softisp_anw_cancel_pipeline()` and `softisp_anw_pipeline_cancelled()`. The
  `android_main` loop polls this registry (10 ms timeout), so cancellation is
  handled on the activity thread and never touches `ANativeWindow` from a worker.
- **Worker contract:** safe points now sit after prepare, immediately after
  capture (before staging/ISP), after handing a finished frame to the presenter
  and `finish()` consumes `PendingStats`, and before idle sleeps. A frame
  already in flight finishes its stats readback; no new work starts.
- **Staged-path constraint:** a cancel request never drops a live
  `PreparedFrame` or `PendingStats` guard. The worker owns both guards, calls
  `finish()` before checking the stop flag, and its engine drops only after the
  join. See `docs/isp/20261126-p2d-cancellation.md`.
- **Verification:** hermetic tests cover idempotent cancel, worker exit, engine
  `Drop` after join, camera/reorder cleanup, and the C ABI registry state. The
  on-device cancel/restart check remains a user-side APK check; the assistant
  only reads logs.

### P2e — APK step 3 shipped the previous app library (resolved by P2g)

- **Status:** reproduced on the 2026-11-25 build. `build-apk.sh` patched and
  copied `libcam_camera_app.so` into `cam-rust/lib` before `cargo apk2 build`,
  and cargo-apk2 packaged that stale copy. Details and hashes:
  `docs/build/20261125-apk-step3-stale-lib-audit.md`.
- **Resolution:** P2g removes the copy/patch-to-`cam-rust/lib` path entirely.
  cargo-apk2 builds `softisp-anw` and packages the cdylib it finds in `target/`;
  a `RUSTC_WRAPPER` applies the libc++ `patchelf` rewrite before packaging. The
  APK entry hash is verified against the target cdylib, and
  `libcam_camera_app.so` is rejected explicitly.
- **Scope:** this was an interim diagnosis. The shipped application now loads
  `libsoftisp.so` from `anw/`.

### P2f — Recent local branch patches without upstream equivalents ✅

- **Status:** all five commits on `local-fixes` dated 2026-11-23/24 were
  compatibility-reviewed against the current `anw/` workspace on 2026-11-26.
  No blind cherry-pick was performed; every other branch with commits in the
  strict three-day window is patch-equivalent upstream or is the current
  `master`. Details: `docs/overview/20261125-local-branch-audit.md` and
  `docs/overview/20261126-p2f-patch-review.md`.
- **Patches:** `04e87a9e` (MNNConvert CLI build) skipped — runtime conversion
  uses the in-process `libMNNConvertDeps.so` API and the added CLI path is not
  in the fallback search list. `ea139531` (mnn-gated cleanups) superseded by
  `SessionPool` and explicit zero-copy metadata. `a796111e` (runtime_libs path)
  already applied in `anw/Cargo.toml`. `72ad0292` (cam-app → cam-disp /
  android-app docs) applied where still stale in `cam-rust/README.md` and
  `docs/isp/PROFILE_BLOCK_COMPARISON.md`. `2b796345` (AppCompat theme /
  adaptive icons) skipped — the current platform theme and working icon set
  are already part of the verified APK.
- **Action:** closed; older-than-three-day changesets and all agent worktrees
  were explicitly excluded from triage.

### P2g — Move all app code into `anw/`; make `android-app/` resource-only ✅

- **User direction recorded 2026-11-25.** `android-app/` now holds non-code APK
  resources only. All Rust/native application code lives in `anw/`, whose
  `softisp` cdylib (`libsoftisp.so`) is loaded by NativeActivity.
- **Library name:** `libsoftisp.so`; the existing `softisp_anw_*` C ABI symbols
  remain unchanged for compatibility.
- **Target split:**
  - `android-app/`: `AndroidManifest.xml`, `Sources/res/**`; no `Cargo.toml`,
    `src/**`, or camera/ISP/pipeline/storage/crash-log code.
  - `anw/`: `android_main` in `app.rs`, `camera.rs`, `crashlog.rs`, `pipeline.rs`,
    `pipeline_runner.rs`, `storage.rs`, `presentation.rs`, the raw-frame C ABI,
    and the Android linker build script.
- **Packaging:** `anw/Cargo.toml` uses package `com.softisp.camera`, APK name
  `softisp-camera`, `manifest_override = "../android-app/AndroidManifest.xml"`,
  `resources = "../android-app/Sources/res"`, and
  `runtime_libs = "../cam-rust/lib"`. `android.app.lib_name` is `softisp`.
- **Build:** `build-apk.sh` runs `scripts/build_mnn_libs.sh arm64-v8a`, then
  `cargo apk2 build -p softisp-anw`. A `RUSTC_WRAPPER` applies the libc++
  `patchelf` rewrite to the freshly linked cdylib in `target/` before
  cargo-apk2 copies it into the APK; no app library is copied into
  `cam-rust/lib`.
- **Behaviour constraints preserved:** `OutputFormat::PackedRgb`, final
  `isp.display_uint32_bgra`, dual-engine execution, staged frame phases, no
  float fallback, AHB only for large tensors, and the existing crash-log path.
- **Verification:** workspace format/check/clippy/tests for `anw`; successful
  `./build-apk.sh`; APK contains `lib/arm64-v8a/libsoftisp.so` and no
  `libcam_camera_app.so`; target and APK entry hashes match. Device logs remain
  a user-side check (the assistant does not install or share the APK). Full
  plan: `docs/ipa/ANW_SPLIT_PLAN.md`.

### P2g.1 — APK launcher shim experiment (device check pending)

- **User direction recorded 2026-11-26.** Test whether ISP/MNN edits can skip
  APK repacking by adding a separate tiny `NativeActivity` launcher crate in
  `android-shim-app/`; `android-app/` and `build-apk.sh` stay untouched.
- **Change:** new `softisp-shim` crate builds `libsoftisp_shim.so`;
  `android.app.lib_name` now selects it. It stages and `dlopen`s the real
  `libsoftisp.so` from `anw/` and forwards `android_main`. All searched roots
  are logged, with a `FATAL [shim]` line on failure.
- **Build:** `./build-apk.sh` stays the production APK path. The new
  `./build-test-apk.sh` builds the core with `cargo apk2 -- build -p softisp-anw`
  and packages a shim-only APK (`softisp-test.apk`); no MNN or `libsoftisp.so`
  is bundled. MNN libs are pushed to `/sdcard/download` alongside the core.
- **Verification:** format/clippy/6 shim unit tests (including MD5 vectors) and
  a successful `./build-test-apk.sh` pass locally. On-device startup is a
  user-side check. Details: `docs/build/20261126-apk-shim-hotload-experiment.md`.
- **MD5 assurance + intent control (2026-11-26):** every staged `.so` is
  MD5-fingerprinted into the log, and `./auto-debug.sh` compares it with local
  `md5sum`. The rare action `com.softisp.camera.action.CONTROL` with extra
  `softisp_cmd` accepts `ping`, `cancel` (calls the core's
  `softisp_anw_cancel_pipeline`) and `stop` (self-terminates the activity).
- **Intent delivery fix:** stock `NativeActivity` never calls `setIntent()` on
  `onNewIntent()`, so the native watcher could not see control intents. The test
  APK now compiles `com.softisp.camera.ShimActivity extends NativeActivity` via
  `java_sources = "java"` and calls `setIntent()`; the activity class in the
  manifest and in all launch commands changed accordingly. Window flags for
  locked-screen wake are set, but their reliability is still under study.
- **User-owned install:** the assistant must never run `install-apk.sh` or any
  `pm`/`am` package/process lifecycle command (see `AGENTS.md` §1.7); the user
  installs each rebuilt APK. A stale process predating the Java subclass needs
  one user-side force-stop.

### P2h — Overlap AHB framing with metadata prefill/readback (TODO)

- **User concern recorded 2026-11-26.** Reduce the visible per-frame critical
  path to inference itself: while the sensor fills an input AHB, upload the
  metadata tensors that frame will consume; once the buffer is ready, run only
  inference. On the output side, give the finished output AHB to Android for
  display and read back the metadata tensors for the next neural/ISP controller
  at the same time. Skip unnecessary zeroing/memset during AHB allocation to
  shorten warm-up.
- **Current state:** P2c already splits `MnnEngine` into `prepare()` /
  `run_prepared()` / `finish()`, and the workers prep before capture and call
  `finish()` after presentation. However capture still goes through
  `AImageReader_acquireLatestImage` plus a CPU copy, presentation still uses
  `ANativeWindow_lock` and a CPU blit. The output AHB field is live by default
  (the fork imports it in `onAcquire`), but the input AHB field is not wired for
  the APK. The two overlap opportunities are therefore not yet live.
- **Proposed input path:** allocate/bind the input AHB before capture and hand
  it to camera2 as the capture target. While the sensor writes that buffer,
  `prepare()` snapshots the already-current controller state and uploads the
  metadata tensors for the arriving frame. A buffer-ready fence/generation
  token releases `run_prepared()`; the visible path then contains only
  `runSession` plus any unavoidable output handoff. Never read the AHB before
  the capture fence and never reuse a slot the GPU still reads.
- **Proposed output path:** import/queue the output AHB to the Android
  compositor immediately after inference; in parallel call `finish()` on the
  worker to read 3A/histogram/zone tensors into the next controller state.
  The output AHB may not be recycled until both the display release fence and
  `finish()` have completed; track this with the existing staged token plus an
  explicit display-release token.
- **AHB fence plumbing (foundation landed, full handoff TODO):** Android
  `AHardwareBuffer` has no fence field of its own; the sync fd travels with the
  BufferQueue/capture slot. `cam_types::{SyncFenceToken, AhbFenceTokens,
  AhbBinding}` now carry non-owning fd/generation tokens, `cam_ipa::AhbFenceState`
  owns the fds, and `ProcessParams::{input_ahb, output_ahb}` carry them instead
  of bare tuples. The Android HAL `AHardwareBuffer_lock`/`unlock` wrappers pass
  the acquire fd and return the release fd. `MnnEngine::run_prepared()` rejects
  a binding from the wrong generation. Still TODO: camera acquire fd → Vulkan
  wait semaphore before `run_prepared()`, GPU completion → display release fd
  after inference, and wait for both display release and `finish()` before
  recycling. Never assume implicit driver ordering.
- **Non-Android dmabuf/sync_fd TODO:** AHB is Android-specific. V4L2/dmabuf
  platforms need the same generation-tagged contract using `dma-buf` fds and
  `sync_file`/fence fds, with Vulkan external-fence import and
  `BufferManager::allocate_dmabuf()` slots. Do not let the Android AHB wrappers
  become a proxy for host dmabuf ordering; add the Linux path separately with
  hermetic fd-ownership tests and an ignored device test.
- **Zeroing audit:** measure before changing. `MemBuffer::new()` zeroes with
  `vec![0u8; size]`, scratch vectors use `resize(..., 0)`, and
  `AHardwareBuffer_allocate` zeroing is allocator/driver-dependent. Skip the
  clear only for buffers the camera or GPU fully overwrites before any read;
  keep zeroing for partial stats/readback buffers and for any buffer that can
  be observed before the producer write completes. An explicit `ZeroPolicy`
  flag is now available; the host `MemBuffer` still zeroes both policies because
  skipping safely needs a producer callback that initialises every byte. TODO:
  wire the policy into an allocating producer path, report warm-up time with and
  without the clear, and confirm `AHardwareBuffer_allocate` assumptions on-device.
- **Constraints:** `OutputFormat::PackedRgb` and `isp.display_uint32_bgra`
  stay unchanged; no float fallback; AHB only for large tensors, small tensors
  stay non-AHB; dual-engine ordering and P2d cancellation must drain both the
  AHB slots and `PreparedFrame`/`PendingStats` guards. No APK install or launch
  by the assistant — device checks are user-side log evidence.
- **Dependencies:** P2b for input AHB ownership and camera-buffer handoff; P2c
  for the staged API; on-device fence/coherency validation for the default-on
  output surface. Do not bypass P2b with a host-pointer bind the Vulkan backend
  ignores.
- **Verification:** hermetic tests now cover metadata/frame generation matching
  in `PrepareParams`, fence ownership/close-on-consumer, no reuse before the
  release fence is taken, the zeroing policy and full-overwrite host allocation.
  Idempotent cancel/drain with both token shapes is now covered by hermetic
  `cam-types` and `cam-ipa` tests. The engine-side probe split is done
  (`FramePhaseTimings`, `StagedFinish::readback_ns`, `perf_critical_path`; see
  `docs/isp/20261126-critical-path-instrumentation.md`): prefill, inference,
  output and deferred readback are reported separately. An ignored on-device
  test now reports warm-up allocation time and checks that the first frame after
  cancel + restart is non-blank with sequence 0 (`softisp-anw`, `--ignored`);
  user-side log confirmation is still required. Still TODO: app-level
  timestamps for capture-fill and display handoff.

### P4 — Multi-Frontend IPA Plugin Architecture ✅ (implemented 2026-11-25)

New crates added to the workspace:

| Crate | Type | Path | Purpose |
|-------|------|------|---------|
| `cam-ipa` | rlib | `cam-rust/cam-ipa/` | Stable Rust IPA API — `IpaContext`, `EngineManager`, `IpaController`, `IpaPipeline`, `BufferManager`, `EventEmitter` |
| `libcamera-ipa` | rlib+cdylib | `libcamera-ipa/` | libcamera C FFI plugin — exports `ipaCreate()` / `IPAModuleInfo`, implements `IPASoftIspInterface` vtable |
| `jni` | rlib+cdylib | `jni/` | Android JNI bridge — `JNI_OnLoad`, `IpaSession` native peer, `HardwareBuffer` → `AhwBuffer` |
| `anw` | rlib+cdylib | `anw/` | ANativeWindow / application code owner (`libsoftisp.so`, NativeActivity + cam-IPA consumer) |
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

### P3 — Follow-up: unified runtime tensor naming ✅ (landed 2026-11-26)

- **WB/CCM runtime sinks converge on `docs/reference/TensorNames.java`.**
  `cam_isp::tensor_names` is the single Rust mirror of the registry subset.
  Blocks now emit `BAYERWBBLOCK/BAYER_GAINS`, `DEMOSAICCCMBLOCK/W`,
  `DEMOSAICCCMBLOCK/B`, `CCMBLOCK/CCM_MATRIX`, and `CCMBLOCK/CCM_BIAS`; the
  engine detects sink presence and writes values under both canonical and
  legacy names. Existing MNN models and probes therefore keep loading. Details
  and explicit deferrals are in `docs/isp/20261126-p3-runtime-tensor-names.md`.
- **Intentionally deferred:** `DemosaicBlock/w` / `b` have no registry entry;
  `bayer_proc` and controller-model `awb.gains*` are separate namespaces;
  GLSL/C++ uniforms are positional bindings; `ProcessParams` field names and a
  repo-wide suffix normalization across unrelated blocks are separate changes.
  The Java registry is generated from `TensorNames.kt`; if the registry itself
  must change, update the Kotlin source and regenerate the Java wrapper.
- **Verification:** `cargo fmt --all -- --check`, canonical clippy for `cam-isp`
  with `rectifier` and `rectifier mnn`, 851 rectifier / 886 real-MNN hermetic
  tests, and the ignored `probe_isp_ccm_bindings`, `probe_wb_fused`, and
  `probe_isp_contract` MNN probes are clean. `RUNTIME_PARAMS.md`,
  `BAYER_WB_DESIGN.md`, and the perf-phase findings were updated with aliases.

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
