# Repo-wide AHB / zero-copy analysis (2026-11-26)

Date: 2026-11-26  
Scope: current tree at `f3484653`, Android/NDK code, Rust HAL/IPA/ISP code, C++ MNN FFI, build scripts, project docs, and the read-only MNN fork at `/data/data/com.termux/files/home/MNN`. Search terms included `AHardwareBuffer`, `AHardWareBuffer` (no matches), `hardware_buffer`, `gralloc`, `MNN_MEMORY_AHARDWAREBUFFER`, `setDevicePtr`, `import_ahb`, `input_ahb`, `output_ahb`, `AhbBinding`, `AHARDWAREBUFFER_`, `dma-buf`/`dma_buf`, `memfd`, and `AhbFenceTokens`. This document is the only file written by the study.

## Executive summary / verdict

1. **The only real producer-side AHB owner in the APK is `anw/src/ahb_output.rs`.** It allocates a `BLOB` AHB, locks it once, keeps the mapping for the worker lifetime, builds `AhbBinding` for MNN, and unlocks/releases on drop (`anw/src/ahb_output.rs:179-238`, `:247-255`, `:304-319`). Nothing else in the tree supplies a comparable slot to the app.

2. **No existing helper can replace that slot as-is.**
   - `cam-rust/cam-ipa/src/buffer.rs` owns fds/mmap for `DmaBufBuffer`, but `AhwBuffer` is only a wrapper around a caller-supplied AHB pointer: null CPU pointers, no fd, no release (`cam-rust/cam-ipa/src/buffer.rs:11-16`, `:105-193`, `:291-303`).
   - `cam-rust/cam-hal-android/src/gralloc.rs` can allocate a real AHB, but its `AHardwareBufferBacked` wrapper has per-call lock/unlock helpers, no AHB accessor, and no release/Drop (`cam-rust/cam-hal-android/src/gralloc.rs:82-119`, `cam-rust/cam-hal-android/src/adapter.rs:182-299`). It also links non-NDK HAL libraries (`cam-rust/cam-hal-android/build.rs:12-24`) that `anw` does not depend on (`anw/Cargo.toml:8-15`).
   - `HardwareBufferPool` and `cam-binder::HardwareBufferBridge` are accounting stubs with opaque ids/null pointers (`cam-rust/cam-hal-android/src/buffer_pool.rs:85-92`, `:146-166`; `cam-rust/cam-binder/src/hal_bridge.rs:53-70`, `:108-172`).
   - Linux `dmabuf`/`memfd`/CMA paths are host/V4L2-style fd+mmap buffers, not Android AHB slots (`cam-rust/cam-hal-linux/src/dmabuf.rs:17-40`, `:42-119`; `cam-rust/cam-isp/src/mnn_buffer/memfd_buffer.rs:26-61`; `cam-rust/cam-isp/src/mnn_buffer/cma_buffer.rs:86-105`).

3. **The MNN fork has real per-tensor Vulkan AHB import, but only because the caller owns the AHB.** `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` stores flags/device (`/data/data/com.termux/files/home/MNN/source/core/Tensor.cpp:498-503`); the import happens later in `VulkanBackend::onAcquire()` during resize (`VulkanBackend.cpp:351-383`) through `VulkanBufferFromAHB::create()` (`VulkanBuffer.cpp:126-225`). Public APIs can bind and wait but cannot allocate/export an AHB; `getDeviceInfo()` returns a `VkBuffer`, and `TensorUtils::getSharedMem()` is source-only (`TensorUtils.hpp:216-218`).

4. **The landed output path is narrow and opt-in.** It removes the engine's per-frame output allocation and `to_vec()`/`copyToHostTensor` readback when enabled, but capture still copies `AImage` pixels into a `Vec` (`anw/src/camera.rs:552-574`, `:582-659`) and presentation still performs an `ANativeWindow_lock` CPU blit (`anw/src/presentation.rs:197-222`, `:242-264`). It is not end-to-end compositor zero-copy.

5. **Direct answer:** neither `cam-ipa::buffer.rs` nor `cam-hal-android::gralloc.rs` can replace `anw/src/ahb_output.rs` today. At most, the gralloc file's `AHardwareBuffer_allocate` call is a lower-level primitive that could be factored into an NDK helper; it lacks ownership, persistent mapping, per-worker handoff, generation/fence handling, and release. Keep `anw/src/ahb_output.rs` as the producer-side owner.

## Inventory table

| Path / area | What the code actually does | Real AHB? | Replacement candidate? | Evidence |
|---|---|---:|---:|---|
| `anw/src/ahb_output.rs` | Allocates `AHARDWAREBUFFER_FORMAT_BLOB` with `CPU_READ_OFTEN | GPU_DATA_BUFFER`, capacity `2 * w * h * 4` rounded to power of two; locks once; exposes mapping + binding; busy flag handoff; unlock/release on drop. | Yes | **This is the slot** | `:36-41`, `:63-72`, `:179-238`, `:247-255`, `:269-301`, `:304-319` |
| `anw/src/pipeline_runner.rs` | One slot per MNN worker; recreate on geometry change; bind in `prepare()`; wait/mark busy per frame; send `AhbView`; mark free on send failure/error. | Uses real slot | No, lifecycle only | `:236-258`, `:263-271`, `:331-340`, `:359-390` |
| `anw/src/app.rs`, `anw/src/presentation.rs` | Presenter reads mapped bytes, marks slot free after blit, skips repaint while busy; `ANativeWindow_lock` + CPU scale/BGRA→RGBA copy remains. | Consumer of slot | No | `app.rs:620-629`, `:703-714`; `presentation.rs:197-222`, `:242-264` |
| `anw/src/camera.rs` | `AImageReader_acquireLatestImage`; plane copied into `Vec<u16>`; no camera AHB capture target. | No | No | `:552-574`, `:582-659` |
| `cam-rust/cam-ipa/src/buffer.rs` — `DmaBufBuffer` | Owns `RawFd` via `OwnedFd`, `mmap`s it, unmaps on drop. | fd/dma, not AHB | No for AHB | `:11-16`, `:25-57`, `:77-103` |
| `cam-rust/cam-ipa/src/buffer.rs` — `AhwBuffer` | Wraps caller `AHardwareBuffer*`, describes it, stores fences; CPU pointers and fd are null/none; no allocation/release. | Wrapper only | No | `:105-147`, `:160-193`; manager `allocate_ahb` only wraps at `:291-303` |
| `cam-rust/cam-ipa/src/fence.rs` | Owning `SyncFence`/`AhbFenceState`; generation, acquire/release fd ownership and validation. | Fence utility only | No | `:7-15`, `:50-61`, `:80-142` |
| `cam-rust/cam-types/src/lib.rs` | Non-owning `SyncFenceToken`, `AhbFenceTokens`, `AhbBinding` view. | Types only | No | `:340-427` |
| `cam-rust/cam-ipa/src/context.rs` | Populates `ProcessParams` from a `Buffer`; AHB fields built from `ahb_ptr`/fences. With `AhwBuffer`, `output_buffer` is a null pointer. | Plumbing only | No | `:290-305`; null pointer from `buffer.rs:160-176` |
| `cam-rust/cam-hal-android/src/gralloc.rs` | `AHardwareBuffer_fromHardwareBuffer` import and real `AHardwareBuffer_allocate`; `RawFdInterop` mmaps native_handle fd. | Yes (allocator/import) | Partial primitive only | `:30-44`, `:82-119`, `:126-155` |
| `cam-rust/cam-hal-android/src/adapter.rs` | `AHardwareBufferBacked` wraps raw handle with lock/unlock helpers; `AHardwareBufferAllocator` falls back to heap; `lock_and_copy_buffer` copies whole AHB to `Vec`. | Wrapper/copy | No | `:182-259`, `:301-333`, `:349-393` |
| `cam-rust/cam-hal-android/src/buffer_pool.rs` | Opaque handle id and accounting; real allocation only a comment. | Mock | No | `:85-92`, `:146-166` |
| `cam-rust/cam-binder/src/hal_bridge.rs` | Null `host_ptr`, `fd = id as i32`, release increments `zero_copy_count`. | Mock | No | `:53-70`, `:108-172` |
| `cam-rust/cam-isp/src/engine.rs` | `ProcessParams::{input_buffer, output_buffer, input_ahb, output_ahb}`; AHB fields are non-owning views; `PrepareParams` records generations. | API plumbing | No | `:177-218`, `:268-301` |
| `cam-rust/cam-isp/src/mnnengine.rs` | Import-before-resize, generation/pointer/capacity validation, AHB output run and fallback copy, external `IspFrame`. | Runtime binding | No | `:2175-2185`, `:2234-2251`, `:2360-2423`, `:2580-2671`, `:2764-2766`, `:2836-2847` |
| `cam-rust/cam-isp/mnn_sys/*` | Rust FFI plus C++ `setDevicePtr`, no-copy run, wait, release; header still describes direct fd/VkDeviceMemory import that the implementation does not do. | FFI to real AHB | No | `src/mnn_sys.rs:148-186`; `mnn_wrapper.cpp:749-878`; `mnn_wrapper.h:338-352` |
| MNN fork | Public `MNN_MEMORY_AHARDWAREBUFFER`, public `setDevicePtr`/`wait`, source-only AHB MemObj and real `VkImportAndroidHardwareBufferInfoANDROID` path. | Backend import | No allocator | `MNNForwardType.h:53-55`; `Tensor.hpp:282`, `:306`, `:310`; `VulkanBackend.cpp:290-383`; `VulkanBuffer.cpp:126-225` |
| `cam-rust/cam-hal-linux/src/dmabuf.rs` | dma-heap open, memfd fallback, mmap/munmap/close. | Linux dma/memfd | No | `:17-40`, `:42-119` |
| `cam-rust/cam-isp/src/mnn_buffer/{memfd_buffer,cma_buffer}.rs` | memfd+mmap and CMA/ION/dma-buf/shm/malloc buffer managers for MNN host pointers. | Not AHB | No | `memfd_buffer.rs:26-61`, `:101-120`, `:214-220`; `cma_buffer.rs:86-105`, `:229-245` |
| `cam-rust/cam-isp/vendor/mnn/` | Checked-in fallback headers/converter sources; `build.rs` prefers `$HOME/MNN/include` when present. | Headers only | No | `build.rs:14-23`, `:625-635`; `vendor/mnn/include/MNN/MNNSharedContext.h:27-34` |

## Deep dives

### 1. Producer-side slot: `anw/src/ahb_output.rs`

`AhbOutputSlot::new()` computes capacity, describes a one-dimensional `BLOB` AHB, calls `AHardwareBuffer_allocate`, then locks it once with `CPU_READ_OFTEN` and a `-1` acquire fence (`anw/src/ahb_output.rs:179-224`). The slot stores handle, map, capacity, geometry, generation and a busy flag (`:148-166`). `buffers()` returns both the `AhbBinding` used by `setDevicePtr` and the `(map, capacity)` tuple used as `ProcessParams::output_buffer` (`:247-255`). `Drop` unlocks and releases the handle (`:304-319`).

The one-slot-per-worker rule is deliberate: MNN imports the AHB in `onAcquire`, so swapping buffers under an already-bound tensor would keep the old `VkBuffer`; geometry changes therefore drop and recreate the slot (`anw/src/pipeline_runner.rs:239-257`; `anw/src/ahb_output.rs:22-26`). Busy/wait is a CPU `AtomicBool` handoff, not a sync fd (`anw/src/ahb_output.rs:139-145`, `:269-301`).

### 2. `cam-ipa` buffers and fences

`DmaBufBuffer` is a genuine fd owner: it takes an `OwnedFd`, mmaps it, exposes the pointer/fd, and unmaps on drop (`cam-rust/cam-ipa/src/buffer.rs:11-16`, `:25-57`, `:77-103`). `create_memfd_dmabuf` is explicitly `#[cfg(not(target_os = "android"))]` and is a test helper (`:195-222`).

`AhwBuffer` is different: `new()` only calls `AHardwareBuffer_describe` on a pointer supplied by the caller and estimates size as `width * height * 4` (`cam-rust/cam-ipa/src/buffer.rs:123-147`). Its `Buffer` impl returns null CPU pointers, no fd, and the AHB pointer/fence tokens (`:160-193`). `BufferManager::allocate_ahb` merely constructs and stores this wrapper (`:291-303`). There is no `AHardwareBuffer_allocate`, lock, map, unlock, or release in this path.

`cam-ipa::fence.rs` does provide real ownership: `SyncFence` owns an fd, `AhbFenceState` tracks generation/acquire/release, refuses reuse before release is taken, and validates tokens (`cam-rust/cam-ipa/src/fence.rs:7-15`, `:50-61`, `:80-142`). These types can support a future producer slot, but they do not allocate pixels today. `cam-ipa/src/context.rs:290-305` shows how they flow into `ProcessParams`; with `AhwBuffer` the host pointer remains null.

### 3. Android gralloc/adapter/pool

`gralloc.rs` declares only `AHardwareBuffer_fromHardwareBuffer` and `AHardwareBuffer_allocate` as externs (`cam-rust/cam-hal-android/src/gralloc.rs:30-44`). `AHardwareBufferInterop::import` wraps a framework handle; `allocate` really calls `AHardwareBuffer_allocate` and wraps the result (`:82-119`). The wrapper, `AHardwareBufferBacked`, describes the buffer and offers `map_with_fence`/`unmap_with_fence`, but it has no Drop/release and no `ahb_ptr()` accessor (`cam-rust/cam-hal-android/src/adapter.rs:182-259`). Its normal `map()` passes `-1`, and `unmap()` discards the release fd (`:261-269`). `AHardwareBufferAllocator` falls back to heap for both allocation methods (`:301-333`), and `lock_and_copy_buffer()` performs a full AHB→`Vec<u8>` copy (`:349-393`).

`HardwareBufferPool` is explicitly an opaque stub; allocation is only a comment (`:85-92`, `:146-166`). `cam-binder` is even more explicitly an accounting mock: null host pointer, id-as-fd, and release increments a counter (`cam-rust/cam-binder/src/hal_bridge.rs:53-70`, `:108-172`).

Linking this HAL into `anw` would also add `cutils`, `utils`, `camera_metadata`, `camera3`, `hardware`, `hidl`, `base`, and `gralloc` (`cam-rust/cam-hal-android/build.rs:12-24`), while `anw` currently declares only `cam-ipa`, `cam-isp`, `cam-types`, `ndk-sys`, `android-activity`, `thiserror`, and `log` (`anw/Cargo.toml:8-15`). That is a second reason not to route the app slot through it.

### 4. Engine and FFI boundary

`ProcessParams` stores host pointers plus non-owning `AhbBinding`s; the caller must keep the buffer and fds alive (`cam-rust/cam-isp/src/engine.rs:177-218`). `PrepareParams` captures AHB generations so staged runs can reject stale bindings (`:268-301`).

`MnnEngine::prepare()` sets `zero_copy = p.output_buffer.is_some()` and `output_ahb = zero_copy && p.output_ahb.is_some()` (`cam-rust/cam-isp/src/mnnengine.rs:2175-2185`). If output AHB is active, it calls `mnn_import_ahb_as_device_memory` only when the session needs resize, before `resize()` (`:2234-2251`, `:2264`). `run_prepared()` validates geometry, AHB generations, output pointer and capacity (`:2360-2423`), then chooses `mnn_run_ahb_output` (`:2622-2635`). A `-10` result falls back to `mnn_run_true_zero_copy` into the same AHB mapping (`:2636-2656`). For zero-copy frames, `data_out` stays empty (`:2764-2766`) and `IspFrame::new_external()` carries the caller pointer (`:2836-2847`).

The C++ wrapper's `mnn_import_ahb_as_device_memory()` calls `AHardwareBuffer_getFd` only diagnostically and immediately closes the fd; the real operation is `tensor->setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` (`cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp:811-854`). `mnn_run_ahb_output()` rejects outputs that are not AHB-backed, runs the session, checks capacity, waits with `output->wait(MAP_TENSOR_READ, true)`, and never calls `copyToHostTensor` (`:749-781`).

## MNN capability findings

- Public constant: `MNN_MEMORY_AHARDWAREBUFFER = 14` (`/data/data/com.termux/files/home/MNN/include/MNN/MNNForwardType.h:53-55`). It is a memory-type flag, not a forward backend type.
- Public tensor APIs: `Tensor::getDeviceInfo`, `map`/`unmap`, `wait`, and `setDevicePtr` are declared in `include/MNN/Tensor.hpp:282`, `:299-300`, `:306`, `:310`.
- `setDevicePtr()` only writes `mBuffer.flags` and `mBuffer.device`; it allocates nothing and imports nothing (`source/core/Tensor.cpp:498-503`).
- `getDeviceInfo()` dispatches to backend `onGetTensorInfo` (`source/core/Tensor.cpp:510-519`). Vulkan fills `MNNVulkanTensorContent`, which exposes `VkBuffer`, size and offset, not an AHB (`include/MNN/MNNSharedContext.h:27-34`; `source/backend/vulkan/buffer/backend/VulkanBackend.cpp:840-849`).
- `TensorUtils::getSharedMem()` is declared in `source/core/TensorUtils.hpp:216-218` and implemented at `source/core/TensorUtils.cpp:1001-1004`; it is not part of `include/MNN`. The AHB-owning `VulkanAhbMemObj` is file-local to `VulkanBackend.cpp:290-306`. External wrappers cannot safely downcast it.
- Real import path: `VulkanBackend::onAcquire()` checks `MNN_MEMORY_AHARDWAREBUFFER`, calls `VulkanBufferFromAHB::create()`, stores a per-tensor MemObj, and returns it (`:351-383`). `getBuffer()` returns the imported AHB `VkBuffer` (`:308-336`). `VulkanBufferFromAHB::create()` creates an external-memory buffer, queries AHB properties, imports with `VkImportAndroidHardwareBufferInfoANDROID`, allocates and binds device memory (`:126-225`). This is per tensor, so two workers can bind different buffers.
- Import timing is lazy: `setDevicePtr` before resize is required. The app's bind-before-resize order is at `cam-rust/cam-isp/src/mnnengine.rs:2234-2264`; swapping an AHB without re-acquire is not supported (`VulkanBackend.cpp:319-323`, `:365-382`).
- `Tensor::map()` is not a zero-copy AHB export. If the backend does not override `onMapTensor`, it `malloc`s and calls `onCopyBuffer` for read (`source/core/Tensor.cpp:427-460`). The landed output path deliberately uses `wait`, not `map`.
- Build mode matters: the AHB path lives in the buffer Vulkan backend. Softisp's MNN build script invokes the fork script (`scripts/build_mnn_libs.sh:61-64`), which sets `MNN_VULKAN=ON`, `MNN_VULKAN_IMAGE=OFF`, and `MNN_SEP_BUILD=ON` (`/data/data/com.termux/files/home/MNN/build_vulkan_android.sh:95-98`). `cam-isp/build.rs` prefers `$HOME/MNN/include` when present and copies `libMNN.so`, `libMNN_Vulkan.so`, and converter libs into the APK runtime dir (`cam-rust/cam-isp/build.rs:14-23`, `:337-355`, `:447-449`).

## Current landed implementation and switches

| Commit | Landed change |
|---|---|
| `69fb7a61` | Added `mnn_run_ahb_output()` plus Rust FFI declarations; no `copyToHostTensor` on the AHB path (`cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp:664-782`; `src/mnn_sys.rs:169-186`). |
| `bab1c1c5` | Added bind-before-resize, staged generation/pointer/capacity validation, and `IspFrame::new_external()` use (`cam-rust/cam-isp/src/mnnengine.rs:2175-2185`, `:2360-2423`, `:2836-2847`). |
| `0d0a712d` | Added `anw/src/ahb_output.rs`, worker slot lifecycle, `SOFTISP_OUTPUT_AHB` / `SOFTISP_VERBOSE`, and app/presenter handoff (`anw/src/ahb_output.rs:36-41`; `anw/src/pipeline_runner.rs:236-258`, `:331-340`; `anw/src/app.rs:620-629`). |
| `f3484653` | Recorded the opt-in output path and updated CI counts in the roadmap (`docs/overview/ROADMAP.md:7-15`). |

Runtime switches are off by default: `SOFTISP_OUTPUT_AHB` or `/sdcard/download/softisp-output-ahb` enables output AHB, and `SOFTISP_VERBOSE` or `/sdcard/download/softisp-verbose` enables per-slot logs (`anw/src/ahb_output.rs:36-41`, `:47-97`). The APK still declares `min_sdk_version = 24` (`anw/Cargo.toml:47-49`), so AHB is ignored below API 26 with a warning (`anw/src/ahb_output.rs:63-72`). `anw` enables MNN by default (`anw/Cargo.toml:25-31`) and links `camera2ndk` on Android (`anw/build.rs:8`).

Per frame the worker waits only for its own presenter handoff, marks the slot busy, publishes binding+mapping, runs, and sends an `AhbView` (`anw/src/pipeline_runner.rs:331-380`). The presenter reads the mapping, blits it into the surface, then marks the view free (`anw/src/app.rs:620-629`). There is no cross-engine lock or wait. On-device validation is still pending (`docs/overview/ROADMAP.md:7-15`).

## Risks, gaps, and documentation contradictions

### Runtime gaps

- **Not end-to-end zero-copy.** Camera capture still de-strides/copies into `Vec<u16>` (`anw/src/camera.rs:582-659`), and presentation still locks `ANativeWindow` and copies/scales pixels (`anw/src/presentation.rs:219-264`). The AHB is a BLOB GPU data buffer, not a queued compositor image (`anw/src/ahb_output.rs:195-201`).
- **Fences are placeholders.** The app publishes `AhbFenceTokens::new(-1, -1, generation)` (`anw/src/ahb_output.rs:247-253`). Ordering is the CPU busy flag, not a sync fd. The engine explicitly warns that input AHB acquire fds are not waited by the MNN Vulkan path (`cam-rust/cam-isp/src/mnnengine.rs:2389-2395`). `cam-ipa` has the owning fence machinery, but the app does not use it for this slot.
- **Output import failure is non-fatal.** `prepare()` warns and continues; `mnn_run_ahb_output()` returns `-10`; the engine falls back to a host copy into the AHB mapping (`cam-rust/cam-isp/src/mnnengine.rs:2245-2250`, `:2636-2656`). This is correctness-safe but can silently leave the intended optimization disabled unless logs are checked.
- **One slot, fixed geometry.** A geometry change drops the slot and invalidates staged prepare (`anw/src/pipeline_runner.rs:239-257`); capacity is `2×` the display bytes rounded to a power of two (`anw/src/ahb_output.rs:180-190`), trading memory for stable MNN binding.
- **API baseline mismatch.** AHB needs API 26 while the manifest allows 24; the runtime fail-closed check prevents symbol resolution on old devices but means the optimization is unavailable there (`anw/Cargo.toml:47-49`; `anw/src/ahb_output.rs:63-72`).
- **Release/binding hazard.** `mnn_release_ahb_device_memory()` clears flags/device (`cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp:856-878`); the prepared output binding is intentionally kept for the worker lifetime. Any future release/rebind must also recreate the slot or force a fresh `onAcquire`, otherwise `getBuffer()` can fall through to a stale/dummy buffer (`VulkanBackend.cpp:319-335`).

### Documentation contradictions / stale claims

| Document | Claim | Current code/fork |
|---|---|---|
| `docs/mnn/ARCH_ISP_OUTPUT_ZERO_COPY.md:5`, `:168-172` | Vulkan still pays device→host readback and the missing piece is binding imported device memory. | The fork now imports per tensor in `onAcquire` (`VulkanBackend.cpp:351-383`), and the app path uses `mnn_run_ahb_output` with no `copyToHostTensor` (`mnn_wrapper.cpp:749-781`; `mnnengine.rs:2622-2635`). Stale. |
| `docs/mnn/ARCH_ISP_OUTPUT_ZERO_COPY.md:115`, `:145`, `:158` | Wrapper exports AHB→dma-buf fd and binds `VkDeviceMemory`; binding is still to be implemented in `mnn_wrapper.cpp`. | The fd is diagnostic and closed; the wrapper only calls `setDevicePtr`; binding is inside the MNN fork (`mnn_wrapper.cpp:834-847`; `VulkanBuffer.cpp:197-222`). Stale/inaccurate. |
| `cam-rust/cam-isp/mnn_sys/mnn_wrapper.h:346-349` | Header states explicit steps `AHardwareBuffer_getFd`, `vkGetMemoryAndroidHardwareBufferANDROID`/fd import, then tensor binding. | Implementation does not perform those wrapper-side steps (`mnn_wrapper.cpp:834-847`). Fix the header comment. |
| `docs/overview/ROADMAP.md:523-525` | Says `ProcessParams::{input_ahb, output_ahb}` are inert for the APK and Vulkan AHB import is not implemented. | The current output path sets both fields and binds output AHB before resize (`anw/src/pipeline_runner.rs:266-270`, `:331-340`; `mnnengine.rs:2234-2251`). Stale; the same roadmap records the landed path at `:7-15`. |
| `docs/isp/20261125-ahb-buffer-pool-and-zero-copy-analysis.md:32-49`, `:309-312` | Says AHB import never runs in the APK and `process_raw_frame_zc` has no callers. | True for that older build; superseded by `0d0a712d` (`anw/src/ahb_output.rs`; `anw/src/pipeline_runner.rs:266-270`). Keep as historical, not current design. |
| `docs/isp/20261125-ahb-buffer-pool-and-zero-copy-analysis.md:443-448` | Says only the OpenCL backend consumes `setDevicePtr`. | False for the current fork; Vulkan consumes it in `onAcquire` (`VulkanBackend.cpp:357-382`). |
| `docs/isp/20261126-p2h-ahb-fences.md:8-30` | Symptom section says AHB bindings are bare `(ptr, len)` with no fence/generation. | Superseded by `AhbFenceTokens`/`AhbBinding` and generation validation (`cam-types/src/lib.rs:340-427`; `engine.rs:268-301`; `mnnengine.rs:2381-2405`). Its stated limits about Vulkan fence waits remain valid (`:61-77`). |
| `docs/mnn/VULKAN_ZEROCOPY_ARCHITECTURE.md:48` in the fork | Says CI default image mode excludes the zero-copy path. | The fork's Android build script used by Softisp sets `MNN_VULKAN_IMAGE=OFF` (`build_vulkan_android.sh:95-98`). The warning is version/build dependent, not a property of the current APK build. |

## Recommendations

1. **Do not replace `anw/src/ahb_output.rs`** with `cam-ipa::buffer.rs` or `cam-hal-android::gralloc.rs`. If code reuse is desired, extract only the NDK allocate/describe/release primitives into a small helper with explicit ownership and a persistent mapping; the current gralloc wrapper does not provide that contract.
2. **Update stale docs in a separate change:** `docs/mnn/ARCH_ISP_OUTPUT_ZERO_COPY.md`, `mnn_wrapper.h:338-352`, `docs/overview/ROADMAP.md:523-525`, and the historical sections of `docs/isp/20261125-ahb-buffer-pool-and-zero-copy-analysis.md` / `docs/isp/20261126-p2h-ahb-fences.md`. Mark them historical rather than deleting evidence.
3. **Validate on device with the switches on** (`SOFTISP_OUTPUT_AHB`, optionally `SOFTISP_VERBOSE`). Confirm the switch log reports `output AHB=on`, no `AHB output import ... failed` / `-10` fallback, no black frames, and stable frame stats. Until then keep the host-copy fallback and the roadmap's "validation pending" note.
4. **Add real fence plumbing before claiming P2h completion:** camera acquire fd → Vulkan wait, GPU completion → display release fd, and wait for both display release and `finish()` before recycling. Replace the busy spin with the owning `SyncFence`/`AhbFenceState` types where a real fd exists.
5. **To remove the final blit**, move from a `BLOB` CPU-mapped buffer to a SurfaceQueue-compatible AHB format/usage and queue it to the compositor after `wait`; presentation should not lock `ANativeWindow` and copy pixels.
6. **Record MNN provenance.** The AHB import behavior comes from the modified fork, not checked-in vendored headers. Pin the fork commit and keep `MNN_VULKAN_IMAGE=OFF`; otherwise the same source can silently lose the AHB path.
