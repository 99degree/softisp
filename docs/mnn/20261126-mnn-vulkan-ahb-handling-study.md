# MNN Vulkan AHardwareBuffer (AHB) handling — evidence-backed study

- **Date:** 2026-11-26
- **Scope:** read-only study of the MNN fork at `~/MNN` plus the softisp integration. Every claim below is anchored to `file:line` in the fork (`MNN/…`) or this repo. No source file was modified; no device run was performed.
- **Question answered:** can softisp keep a display frame on the GPU into a caller-owned AHB, and what are the exact preconditions, guarantees and gaps?

## Executive summary / verdict

1. The fork has a working **per-tensor** Vulkan AHB import, but it is **lazy**: the actual `VkImportAndroidHardwareBufferInfoANDROID` happens in `VulkanBackend::onAcquire()` during **resize**, not in `Tensor::setDevicePtr` and not in softisp's FFI (`MNN/source/backend/vulkan/buffer/backend/VulkanBackend.cpp:351-383`; `MNN/source/core/Tensor.cpp:498-503`; `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp:811-854`).
2. `setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` only writes `buffer().flags`/`device`. Calling it **after** resize does not bind the AHB; swapping the AHB on an already-bound tensor is ignored until another `onAcquire` (re-)runs (`MNN/source/core/Tensor.cpp:498-503`; `VulkanBackend.cpp:319-323,365-382`; `mnn_wrapper.cpp:718-731`).
3. The AHB must be sized for `VulkanBackend::getTensorSize` = `UP_DIV(elementSize,4)*4 * (float && FP16 ? 2 : 4)` bytes. For softisp's packed BGRA display tensor (one 4-byte word per pixel) that is `H*W*4`: 7.68 MB @1600×1200, 8.29 MB @1920×1080, 33.18 MB @3840×2160. softisp's `16×` power-of-two slot covers the fused graph's worst case without a resize (`MNN/source/backend/vulkan/buffer/backend/VulkanBackend.cpp`; `anw/src/ahb_output.rs`).
4. Synchronization is an **internal Vulkan fence wait only**: `Tensor::wait → onSync → _finish → mFence->wait()`. There is **no AHB acquire/release fence import, no cache invalidation, and no cross-API (camera/surface) fence handoff**. softisp keeps the AHB CPU-mapped for its lifetime and uses CPU atomics for the worker↔presenter handoff; that is a software protocol, not a memory-fence contract (`Tensor.cpp:489-496`; `VulkanBackend.cpp:829-832,553-573`; `VulkanFence.cpp:25-35`; `anw/src/ahb_output.rs:211-224,269-301`; `mnnengine.rs:2389-2395`).
5. **No public API** exists to retrieve the AHB or its fd from a tensor. `getDeviceInfo` returns `VkBuffer`, `TensorUtils::getSharedMem/setSharedMem` are in `source/core`, and `VulkanAhbMemObj` is file-local. A wrapper compiled with only `include/MNN` can **set** the binding but cannot **read it back** (`MNN/include/MNN/Tensor.hpp:282,299-310`; `MNN/source/core/Tensor.cpp:510-519`; `MNN/source/core/TensorUtils.hpp:138-140,216-218`; `VulkanBackend.cpp:295-313`).
6. softisp's output-AHB path is **on by default** (opt out with `SOFTISP_OUTPUT_AHB=0` / `SOFTISP_OUTPUT_AHB_OFF` / `/sdcard/download/softisp-output-ahb-off`, disabled below API 26). The host-copy fallback remains because the binding can be lost on resize, because extensions/coherency are unvalidated on device, and because the non-AHB path must still work (`anw/src/ahb_output.rs`, `cam-rust/cam-isp/src/mnnengine.rs`, `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp`).

## Where the Vulkan AHB path lives (call chain)

| Step | Site | Effect |
|---|---|---|
| 1. Caller allocates/locks one AHB per worker | `anw/src/ahb_output.rs:179-238` | `AHARDWAREBUFFER_FORMAT_BLOB`, `width=capacity, height=1`, usage `CPU_READ_OFTEN \| GPU_DATA_BUFFER`; `AHardwareBuffer_lock` once, never unlocked until `Drop` (`:304-319`) |
| 2. Worker sets busy, publishes binding + mapping | `anw/src/pipeline_runner.rs:331-350`, `ahb_output.rs:248-255` | `AhbBinding(ptr, capacity, AhbFenceTokens::new(-1,-1,generation))`, `output_buffer = (map, capacity)` |
| 3. softisp binds AHB to output tensor **before first resize** | `mnnengine.rs:2178-2251` | `mnn_import_ahb_as_device_memory(...)` at `:2237-2244`, then `slot.sess.resize()` at `:2264` |
| 4. FFI "import" | `mnn_wrapper.cpp:811-854` | `AHardwareBuffer_getFd` is **diagnostic only**, fd `close()`d at `:840`; real action is `tensor->setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` at `:847` |
| 5. Public setter | `MNN/source/core/Tensor.cpp:498-503` | `mBuffer.flags = memoryType; mBuffer.device = (uint64_t)devicePtr;` — unconditional `true`, no import |
| 6. Resize acquires storage | `MNN/source/core/Backend.cpp:138-150` → `VulkanBackend.cpp:351-383` | AHB branch at `:357-383`: rejects `device==0` (`:361-364`); re-imports if `getAhbMem` null / id changed / size grew (`:365-367`); usage flags `STORAGE_BUFFER \| TRANSFER_DST \| TRANSFER_SRC` (`:370-371`); stores `VulkanAhbMemObj` in tensor shared memory (`:377-379`) |
| 7. Real Vulkan import | `VulkanBuffer.cpp:126-225` | `vkGetDeviceProcAddr("vkGetAndroidHardwareBufferPropertiesANDROID")` (`:139-146`); `VkExternalMemoryBufferCreateInfo` with `ANDROID_HARDWARE_BUFFER` handle type (`:162-170`); properties → `allocationSize` + memory type (`:176-195`); **`VkImportAndroidHardwareBufferInfoANDROID`** (`:197-200`); dedicated allocation (`:202-211`); `vkAllocateMemory` / `vkBindBufferMemory` (`:212-222`). No dma-buf fd is used anywhere in the fork's import |
| 8. GPU resolution | `VulkanBackend.cpp:308-336` | `getAhbMem()` gates on `flags == MNN_MEMORY_AHARDWAREBUFFER` (`:309-312`); `getBuffer()` returns the imported `VkBuffer` with `getTensorSize(tensor)` and offset `0` (`:319-322`). Non-AHB/zero-device tensors get the normal or dummy buffer (`:324-335`) |
| 9. Run + wait | `mnn_wrapper.cpp:757-781` | `runSession`, check `out_total <= max_out` (`:763-773`), `output->wait(MAP_TENSOR_READ, true)` (`:776`), return element count — **no `copyToHostTensor`** |

`VulkanBufferFromAHB::~VulkanBufferFromAHB` destroys the buffer and frees the imported memory (`VulkanBuffer.cpp:227-234`). There is no explicit `AHardwareBuffer_acquire`/`AHardwareBuffer_release` in this path: the Vulkan import owns its own reference while allocated.

### Timing traps (exact answers)

- **Import trigger:** only `onAcquire`, i.e. resize/prepare (`Backend.cpp:138-146`; `mnnengine.rs:2229-2264`). `setDevicePtr` alone starts nothing.
- **`setDevicePtr` after resize, previously not AHB-bound:** the tensor's `deviceId` still points at MNN's normal `VulkanBuffer` (`VulkanBackend.cpp:391-402`), shared memory is null (`TensorUtils.cpp:1001-1004`), so `getBuffer()` falls through to `getTensorBuffer()` (`:334-335`). The GPU writes into MNN's buffer; the caller's CPU mapping stays black. `mnn_run_ahb_output`'s check only inspects `flags`/`device` (`mnn_wrapper.cpp:751-755`), so this failure passes validation.
- **`setDevicePtr` after resize with a different AHB on an already-bound tensor:** `getAhbMem()` still returns the old `VulkanAhbMemObj`, so `getBuffer()` keeps the old `VkBuffer` until a later resize compares `existing->ahbId()` with `buffer().device` and re-imports (`VulkanBackend.cpp:319-323,365-382`). `mnn_run_ahb_output` only resizes when the input element count changed (`mnn_wrapper.cpp:718-731`), so a same-geometry "swap" is silently ignored. This is why softisp recreates the whole slot and drops the staged prepare on a geometry change (`pipeline_runner.rs:239-257`) and never swaps buffers (`ahb_output.rs:22-26`).

## Tensor sizing and allocation contract

`VulkanBackend::getTensorSize` (`VulkanBackend.cpp:344-349`):

```
alignElementSize = UP_DIV(elementSize, 4) * 4
bytes            = (float && mUseFP16) ? 2 : 4
size             = alignElementSize * bytes
```

- Only float tensors under FP16 precision shrink to 2 bytes; `halide_type_int` (the packed BGRA word path, `mnnengine.rs:410-417`) is always 4 bytes.
- Element count is padded **up to a multiple of four elements**, not to a byte/cache-line boundary. softisp's fused display tensor has `graph_channels == 1` and `shape [1,1,H,W]` (`mnnengine.rs:394-417`), so `elementSize = H*W` and the padding is a no-op for all three target sizes (`H*W` divisible by 4).

| Display | Pixels `H*W` | AHB tensor bytes (1 word/pixel, 4 B) | FP16-float variant | softisp slot capacity (`16×` → next pow2, `anw/src/ahb_output.rs`) |
|---|---:|---:|---:|---:|
| 1600×1200 | 1,920,000 | 7,680,000 (≈7.32 MiB) | 3,840,000 | 134,217,728 (128 MiB) |
| 1920×1080 | 2,073,600 | 8,294,400 (≈7.91 MiB) | 4,147,200 | 268,435,456 (256 MiB) |
| 3840×2160 | 8,294,400 | 33,177,600 (≈33 MB / 31.64 MiB) | 16,588,800 | 536,870,912 (512 MiB) |

Caveats:

- If a profile produces a **non-packed** `[1,4,H,W]` float display tensor, `elementSize = 4*H*W` and `getTensorSize` becomes `16*H*W` (4K ≈ 132,710,400 B), which **does not fit** the slot formula above. The capacity contract is valid only for the packed `graph_channels == 1` layout (`mnnengine.rs:394-425`).
- `VulkanBufferFromAHB::create` sets `bufInfo.size = alignSize` (`VulkanBuffer.cpp:168`) but allocates `ahbProps.allocationSize` (`:210`). The code never checks `ahbProps.allocationSize >= alignSize`; a too-small caller AHB can fail at `vkAllocateMemory`/`vkBindBufferMemory` (`:212-222`) or produce an out-of-range binding.
- `onAcquire` re-imports when `existing->size() < alignSize` (`VulkanBackend.cpp:367`), so a geometry change also needs a new slot; softisp handles this by recreating the slot (`pipeline_runner.rs:245-255`).

## Synchronization and fences: guarantees vs gaps

Call chain: `mnn_wrapper.cpp:776` → `Tensor::wait` (`Tensor.cpp:489-496`) → `VulkanBackend::onSync` (`:829-832`) → `_finish` (`:553-573`) → `VulkanFence::wait`/`rawWait` (`VulkanFence.cpp:25-35`).

- `_finish()` submits every queued command buffer with **zero wait/signal semaphores**, one internal `VkFence`, then `mFence->wait()` and clears the queue (`VulkanBackend.cpp:557-572`). `rawWait` polls `waitForFence(..., 5 s)` in a loop (`VulkanFence.cpp:25-31`). `onSync` ignores `mtype`, `toCpu`, `finish` and `dstTensor` (`:829-831`).
- In the buffer backend, `Pipeline::_exitExecute()` already calls `onExecuteEnd()` (`MNN/source/core/Pipeline.cpp:1297-1302`), and `VulkanBackend::onExecuteEnd` calls `_finish()` (`:506-546`). The explicit `wait` in `mnn_run_ahb_output` is therefore normally a second, no-op `_finish()` (it early-returns when `mCmdBuffers.empty()`, `:554-556`); it remains the only sync point if commands were left queued.
- `Tensor::map(MAP_TENSOR_READ)` would call `onSync` then `onMapTensor`, which Vulkan does not override, then `malloc` + `onCopyBuffer` (`Tensor.cpp:427-460`) — a **copy**. softisp deliberately never calls `map()` on the output; it reads its own AHB mapping (`anw/src/ahb_output.rs:135-137`).

**Guaranteed:** submitted GPU work for that queue has completed (the internal fence signaled) before `mnn_run_ahb_output` returns.

**Not guaranteed:**

- No AHB/acquire or release fence is imported into Vulkan or exported back. `AhbFenceTokens::new(-1, -1, generation)` carries placeholder fds (`anw/src/ahb_output.rs:248-253`), and softisp itself warns that an input AHB acquire fd "is not waited by the MNN Vulkan path yet" (`mnnengine.rs:2389-2395`). The only ordering between GPU write and surface read is the CPU `busy` flag (`ahb_output.rs:139-145,269-301`; `pipeline_runner.rs:335-336,368-390`).
- No Vulkan memory barrier, layout transition or cache-domain operation exists in the AHB import/resolve path (`VulkanBuffer.cpp:126-225`; `VulkanBackend.cpp:319-336,351-383`). `AHardwareBuffer_lock` is called **once** and the mapping stays live across GPU writes (`ahb_output.rs:211-224,304-319`). A permanently CPU-mapped BLOB/`GPU_DATA_BUFFER` is not, by itself, a coherency guarantee for GPU-written pixels; stale reads are possible until validated on real hardware.
- `wait` does not refresh a CPU mapping, does not perform `copyToHostTensor`, and does not synchronise the producer (camera) or consumer (surface) side of the AHB.

## Public vs internal API surface (what softisp can/cannot call)

| Surface | Site | Contents |
|---|---|---|
| `Tensor::setDevicePtr(const void*, int)` | `MNN/include/MNN/Tensor.hpp:307-310`; impl `Tensor.cpp:498-503` | Setter only; stores flags + pointer |
| `Tensor::getDeviceInfo(void*, int)` | `Tensor.hpp:278-282`; impl `Tensor.cpp:510-519` | Requires `backend()->type() == type`, then calls `onGetTensorInfo`; Vulkan fills `MNNVulkanTensorContent` = `{VkBuffer, size, offset, realType, mask}` (`VulkanBackend.cpp:840-850`; struct at `MNN/include/MNN/MNNSharedContext.h:27-35`) |
| `map` / `unmap` / `wait` | `Tensor.hpp:295-306` | Host pointer via copy, or fence wait |
| `MNN_MEMORY_AHARDWAREBUFFER = 14` | `MNN/include/MNN/MNNForwardType.h:53-55` | Public constant used as the `memoryType` argument; note it is **not** a backend type (Vulkan is `MNN_FORWARD_VULKAN = 7`, `MNNForwardType.h:35`), so `getDeviceInfo(dst, MNN_MEMORY_AHARDWAREBUFFER)` returns false |
| `MNNSharedContext.h` | `:19-35,38-55,57-70` | Vulkan context/tensor-content structs, Metal, device context — **no AHB struct, no AHB getter** |
| `TensorUtils::getDescribe/getDescribeOrigin/setSharedMem/getSharedMem` | `MNN/source/core/TensorUtils.hpp:138-140,216-218`; impl `TensorUtils.cpp:996-1004` | Internal `source/core`, not installed under `include/MNN` |
| `VulkanAhbMemObj`, `getAhbMem` | `VulkanBackend.cpp:295-313` (`static` helper, class defined in the `.cpp`) | File-local; `mAhbId`, `VkBuffer`, `shared_ptr<VulkanBufferFromAHB>` cannot be reached from a public-header consumer |

**Answer:** No. An external wrapper compiled with only `include/MNN` can **bind** an AHB with `setDevicePtr`, but cannot retrieve the bound `AHardwareBuffer*`, its fd, or the imported `VkBuffer` as an AHB — `getDeviceInfo` yields a `VkBuffer` (and only for `MNN_FORWARD_VULKAN`), and the AHB-owning object is private to the Vulkan backend. The caller must own, allocate, lock, bind and read the AHB itself, exactly as documented in `anw/src/ahb_output.rs:10-14` and `docs/isp/20261126-output-copy-zero-copy-audit.md:44-57`.

## softisp call flow and switches

**Gates:** `mnn` feature (`cam-rust/cam-isp/Cargo.toml`), enabled by default in `anw` (`anw/Cargo.toml`); `ahb_output.rs` is a normal module (`anw/src/lib.rs`) whose marker checks are Android-only (`anw/src/ahb_output.rs`). AHardwareBuffer entered the NDK in **API 26**; the default-on path is forced off below 26 (`anw/src/ahb_output.rs`). Output AHB defaults to **on**; opt out with `SOFTISP_OUTPUT_AHB=0|off|false|no`, `SOFTISP_OUTPUT_AHB_OFF`, or `/sdcard/download/softisp-output-ahb-off`. Verbose defaults to **off** and is enabled by `SOFTISP_VERBOSE` / `/sdcard/download/softisp-verbose` (`anw/src/ahb_output.rs`).

**Required call order for the zero-copy output path:**

1. Allocate one `AhbOutputSlot` per worker, `2 × display_bytes` rounded to a power of two, `BLOB` format, `CPU_READ_OFTEN | GPU_DATA_BUFFER`, locked once (`ahb_output.rs:179-238`, called from `pipeline_runner.rs:243-257`).
2. Before inference, wait for **this worker's** presenter handoff, mark busy, and put `AhbBinding` + CPU mapping into `ProcessParams` (`pipeline_runner.rs:331-340`; `ahb_output.rs:248-255,269-301`).
3. In `prepare()`: if geometry changed, bind the AHB to the output tensor **before** `resize()` (`mnnengine.rs:2229-2251,2264`); reuse that binding for subsequent frames on the same worker (`:2178-2185`). `run_prepared()` validates pointer, capacity and AHB generation (`mnnengine.rs:2375-2423`).
4. `run_prepared()` takes the `meta.output_ahb` branch (`mnnengine.rs:2622-2635`) → `mnn_run_ahb_output`: verify AHB flags on the output (`:751-755`), upload input via `copyFromHostTensor` (`:733-738`), `runSession` (`:757`), capacity check (`:763-773`), `wait(MAP_TENSOR_READ, true)` (`:776`), return element count. **No `copyToHostTensor`.**
5. Presenter consumes `Done.ahb: AhbView` (`pipeline_runner.rs:359-372`; `ahb_output.rs:125-145`) and marks the slot free after the blit; worker-side error paths also release the busy flag (`pipeline_runner.rs:377-390`).
6. Per-frame release is used only for inputs (and the non-prepared output AHB path): `mnnengine.rs:2580-2621,2672-2694`. `mnn_release_ahb_device_memory` merely clears `flags`/`device` (`mnn_wrapper.cpp:856-879`) — it does not destroy the imported `VkBuffer`/`VkDeviceMemory` (those live until the tensor describe is torn down, `VulkanBuffer.cpp:227-234`).

**Why the host-copy fallback remains:**

- If the AHB binding did not survive a resize, `mnn_run_ahb_output` returns `-10` (`mnn_wrapper.cpp:749-755`); softisp logs the loss and calls `mnn_run_true_zero_copy` **into the same AHB CPU mapping**, which still ends in `copyToHostTensor` (`mnnengine.rs:2636-2656`; `mnn_wrapper.cpp:659`). This keeps the frame visible rather than presenting black pixels.
- Non-AHB callers and the CPU backend still use `mnn_run_true_zero_copy` (`mnnengine.rs:2657-2671`), whose name is aspirational on Vulkan: `copyFromHostTensor`/`copyToHostTensor` are the real transfers (`mnn_wrapper.cpp:625-631,659,666-671`).
- Until device validation of fences, cache coherency and display handoff, the host-copy fallback stays available, but the app default is now the AHB path (`docs/isp/20261126-output-copy-zero-copy-audit.md`, `docs/overview/ROADMAP.md`).

## Risks / open questions / on-device validation checklist

1. **Extensions may never be enabled.** `VulkanDevice.cpp:126-148` only *probes* `VK_KHR_external_memory`, `VK_ANDROID_external_memory_android_hardware_buffer`, `VK_KHR_dedicated_allocation`, `VK_KHR_get_memory_requirements2`, `VK_KHR_bind_memory2` into a trace file; `deviceExtensions` passed to `vkCreateDevice` (`:225-239`) contains only portability/FP16/int8/cooperative-matrix entries. `VulkanBuffer.cpp:139-146` only checks that `vkGetDeviceProcAddr` resolves the properties function. If the driver requires explicit extension enablement, `vkAllocateMemory` with `VkImportAndroidHardwareBufferInfoANDROID` (`:207-216`) can fail and `onAcquire` returns `nullptr` (`VulkanBackend.cpp:372-376`).
2. **Capacity validation.** No check that `ahbProps.allocationSize >= alignSize` (`VulkanBuffer.cpp:176-183,207-212`); the AHB is described as a 1-D BLOB of `capacity` bytes (`ahb_output.rs:195-201`), which must match the tensor's padded word count.
3. **Coherency / cross-API fences.** The imported buffer has no producer/consumer fence chain; CPU mapping stays live; cache behaviour of GPU writes into a permanently mapped AHB is unverified. The 5 s internal `VkFence` (`VulkanFence.cpp:25-31`) is not a display sync token.
4. **Stale device state after release.** The AHB branch of `onAcquire` does not set `tensor->deviceId()` (`VulkanBackend.cpp:357-383`), unlike the normal branches (`:391,402`). Clearing the binding with `setDevicePtr(nullptr,0)` (`mnn_wrapper.cpp:877`) leaves a stale/zero `deviceId`; `getBuffer` will use the dummy buffer or `getTensorBuffer` can assert (`VulkanBackend.cpp:328-342`).
5. **Shape assumptions.** Slot sizing assumes the packed `graph_channels == 1` display layout (`mnnengine.rs:394-425`); a 4-channel float output would exceed capacity by 4×.
6. **On-device checklist:** capture the `[AHB-PROBE]` trace (`MNN/source/backend/vulkan/component/VulkanDevice.cpp`) and `[ZERO-COPY] AHB imported ok` line (`MNN/source/backend/vulkan/component/VulkanBuffer.cpp`, gated by `MNN_ISP_TRACE`); confirm `mnn_last_error()` is empty and no `-10` fallback in the log; run the default-on path with `SOFTISP_VERBOSE` at 1600×1200, 1920×1080 and 3840×2160 and check for non-black frames, Vulkan validation errors, tearing (slot busy warnings, `anw/src/ahb_output.rs`), and geometry-change slot recreation (`anw/src/pipeline_runner.rs`); verify API-26 guard on an older device (`anw/src/ahb_output.rs`).

## Doc contradictions to reconcile

| Doc | Claim | Status vs source |
|---|---|---|
| `docs/isp/20261125-ahb-buffer-pool-and-zero-copy-analysis.md:441-471` | "AHB surface is a stub that cannot move pixels on Vulkan"; `onAcquire` "ignores the flag and allocates from its own pool"; "Only the OpenCL backend consumes `setDevicePtr`"; "do not wire the APK" | Stale. The **same doc** retracts it at `:473-494` (fork now imports per-tensor in `onAcquire`), and `docs/overview/ROADMAP.md:95-104` confirms. Do not use `:441-471` as the current design description. |
| `docs/mnn/ARCH_ISP_OUTPUT_ZERO_COPY.md` | External-memory import is "the remaining step", "partial"/"done (partial)", "the missing piece is binding imported `VkDeviceMemory`" | Stale: the binding exists in the fork's `VulkanBackend.cpp` / `VulkanBuffer.cpp`. The doc also says the wrapper exports a dma-buf fd for binding; in fact the fd is diagnostic-only and closed (`cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp`). Its "no `copyToHostTensor`" claim is true only for the AHB branch with an intact binding. |
| `docs/isp/20261126-output-copy-zero-copy-audit.md` | "`Tensor::wait(MAP_TENSOR_READ, true)` makes the GPU write visible to the CPU mapping" | Overstates the guarantee: only an internal fence wait, no AHB fence or cache invalidation (see above). Its slot and fallback description (1 slot/worker, no swap) still matches the code; capacity is now 16× the display bytes and the opt-out switches are default-on. |
| `docs/overview/ROADMAP.md` | Path landed, on by default, no cross-engine wait, on-device validation pending | Consistent with code, but "explicit AHB fence tokens" should be read as bookkeeping only — `cam-rust/cam-isp/src/mnnengine.rs` still warns the acquire fd is not waited by MNN's Vulkan path. |
| `cam-rust/cam-isp/mnn_sys/mnn_wrapper.h` | Historically claimed the wrapper imports a dma-buf fd into `VkDeviceMemory` | Reconciled: the header now states that it only calls `setDevicePtr`, the fd is diagnostic and closed, and the real import is in the MNN fork's `onAcquire` (`cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp`). |
