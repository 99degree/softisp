# MNN Intermediate Buffer Lifecycle & Inference-Perf Analysis

> **Scope**: how MNN (Vulkan backend, `MNN_VULKAN_IMAGE=ON` buffer backend as built here)
> allocates, shares and frees *intermediate* tensors; whether an op graph
> `A → B` where B consumes A's output copies or aliases; and which memory /
> host-overhead levers actually shorten per-frame inference time for the softisp
> pipeline.
>
> **Sources read** (MNN at `~/MNN`):
> - `source/core/Pipeline.cpp` — tensor alloc/release/refcount, `execute()`
> - `source/core/Session.cpp` — encode / allocMemory / resize
> - `source/backend/vulkan/buffer/backend/VulkanBackend.cpp` — `onAcquire`, `onCopyBuffer`, `_finish`, `onExecuteEnd`
> - `source/backend/vulkan/component/VulkanMemoryPool.{hpp,cpp}`, `VulkanBuffer.cpp`, `VulkanPipeline.cpp`
> - `source/core/BufferAllocator.hpp`, `source/core/WrapExecution.cpp`, `include/MNN/Tensor.hpp`, `include/MNN/Interpreter.hpp`
> - softisp side: `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp`, `src/mnnengine.rs`, `src/mnn_session_pool.rs`
>
> **Last updated**: 2026-06-21

---

## 1. The question: does `A → B` copy or alias?

Case: a graph with two ops, `op1` produces `t1`, `op2` consumes `t1` and
produces `t2`. Does MNN memcpy `t1` into `t2`, or assign it?

**Answer: `t1` and `t2` are two distinct `Tensor` objects with two distinct
`deviceId()` allocations. There is no memcpy between them — and there is no
aliasing either.** The handoff is a *pointer*: `op2`'s command records
`workInputs = {t1}` and `workOutputs = {t2}`, and the GPU shader of `op2`
reads `t1`'s storage-buffer binding and writes `t2`'s. Copy cost is zero;
memory cost is 2× the frame size while both are live.

Why it cannot be free of the second allocation:

| Mechanism | Location | Behaviour |
|---|---|---|
| Refcount-based liveness | `Pipeline::_allocForTensor` (`Pipeline.cpp:958-1078`) | `useCount` per tensor; `_releaseTensor` (`:101-113`) drops `mem` only when `useCount` hits 0 |
| Storage pooling | `EagerBufferAllocator` (`BufferAllocator.hpp:79+`) | freed `MemChunk` goes back to a size-indexed free list and is reused for the **next** tensor of that size — so `t3` in `A → B → C` typically lands on `t1`'s old bytes |
| In-place aliasing | **not implemented** | MNN has no "output tensor = input tensor" rewrite for elementwise ops; the converter's `RemoveInplace` pass (`tools/converter/source/optimizer/PostConverter.cpp:296`) explicitly *separates* tensors for in-place ops |

So the memory profile of a chain is `2 × frame` steady-state (current op output
+ next op output), not `N × frame`, because of pool recycling — but it is never
`1 × frame` for an elementwise chain.

### 1.1 When a *copy* does get inserted

`WrapExecution::needWrap` (`source/core/WrapExecution.cpp:20-57`) inserts an
explicit `OpType_Copy` command when source and destination backends differ —
e.g. a CPU-fallback op feeding a Vulkan op. In an all-Vulkan chain this is
never triggered. **Any CPU-fallback op in the graph silently costs two extra
full-frame copies (GPU→host→GPU) plus a pipeline stall.**

### 1.2 The one aliasing path that does exist

`Pipeline::_InsertCopy` (`Pipeline.cpp:833-838`) rewires
`iterP->workOutputs[v] = iter.workInputs[i]` when an output tensor is also one
of the op's inputs. That is input-in-place (a Raster reading and writing the
same buffer), not output→next-op aliasing. It only fires for graph loops, not
for the linear ISP chain.

---

## 2. Where reallocation actually costs us

The steady-state path is fine (`resizeSession` is only re-entered when the
extent changes — `Session::resize` is guarded by `mNeedResize`/`mNeedMalloc`,
`Session.cpp:309-415`). The expensive moment is **`Pipeline::allocMemory` →
`onClearBuffer()` → `mCurrentDynamicBufferPool->release(false)`**
(`VulkanBackend.cpp:422-426`), which throws away the whole eager free list.
That runs on:

- every `resizeSession` when the shape changed,
- and `VulkanRuntime::onGabageCollect()` (`VulkanRuntime.cpp`, end of file),
  which additionally does `vkDeviceWaitIdle` + `mMemoryPool->clear()` +
  **`mPipelineFactory->reset()`** — destroying every compiled `VkPipeline`.

Note `VulkanPipelineFactory::reset()` (in `VulkanPipeline.cpp`) currently only
nulls `mCache` and constructs a fresh `VulkanPipelineCache` — compiled
pipelines are *dropped*, and the next resize re-compiles shader modules
(≈150-300 ms on Adreno, per `docs/guides/MNN_VULKAN_GUIDE.md`).

**Consequence**: a session rebuild is not just a memory event, it is a
shader-recompile event. Anything that avoids a rebuild is worth more than any
micro-optimisation below.

---

## 3. Host-side overhead found in the frame loop

These are real, measurable, and outside the GPU work.

### 3.1 Unconditional per-command `fopen` in `Pipeline::execute`

`source/core/Pipeline.cpp:1227-1237` — **every command, every frame**:

```cpp
auto code = cmd.execution->onExecute(cmd.workInputs, cmd.workOutputs);
...
{ FILE* tf = fopen("/sdcard/Download/isp_vlog_trace", "a");
  if (tf) { fprintf(tf, "[CMD] %s backend=%d exec=%p\n", ...); fclose(tf); } }
```

Plus a second `fopen` per *frame* at `Pipeline.cpp:1189-1193` that walks the
whole command list to count Vulkan vs CPU commands.

The trace file on this device is **119 MB** and still growing. At ~14 ISP
stages that is ~30 open/close/append/fsync-ish sequences per frame through
f2fs → roughly 0.5-2 ms of pure host latency on a phone, *and* it serialises
because the GPU fence wait is already synchronous.

This is guarded nowhere — unlike the `VulkanFuse.cpp` path which is correctly
gated behind `ispVlog()` (`ISP_DEBUG_VLOG` env / `debug.isp_vlog` prop).

### 3.2 The remaining ISP trace sites

| File | Trigger | Frequency |
|---|---|---|
| `core/Pipeline.cpp:1189`, `:1229` | **none** | per frame + per command |
| `backend/vulkan/buffer/execution/VulkanFuse.hpp:84,109,117` | `VulkanFuseCreator::onCreate` | per session build (acceptable) |
| `component/VulkanBuffer.cpp:21` (`mnnIspZeroCopyLog`) | called by zero-copy path | per AHB import |
| `backend/vulkan/buffer/backend/VulkanBackend.cpp:463` | `mnnIspTraceSink()` non-null | gated by trace arming, OK |

### 3.3 `_finish()` is a full device-wide sync — and it is called 2-3× per frame

`VulkanBackend::_finish()` submits and then `mFence->wait()`. It is called from:

- `onCopyBuffer` (host→gpu) — once for the upload, and again *after*
  (`VulkanBackend.cpp:695-745`: `_finish()` at the start of the branch and
  again after `pushCommand`),
- `onCopyBuffer` (gpu→host) — twice more,
- `onExecuteEnd`.

So a frame pays: upload-sync, execute-sync, download-sync. Each is a full
queue drain. The `[HOST2GPU]` / `[GPU2HOST-STAGE]` debug blocks also
`mHostBuffer->map()` + `fprintf(stderr)` under `ISP_DEBUG_VLOG` only — those
are fine.

**This is inherent to the `copyFromHostTensor` / `copyToHostTensor` usage in
softisp's `mnn_run_with_output`** (`mnn_wrapper.cpp:694-782`) and is the price
of the non-zero-copy input/output contract documented in
`docs/guides/MNN_VULKAN_GUIDE.md` §"Buffer Management for Vulkan".

---

## 4. Ranked opportunities

Ordered by expected wall-clock win per unit of risk. Measured against the
current 4K path (HEAVY 79.8 FPS / UNIFIED 141.2 FPS per `CLAUDE.md`).

### P0 — delete the unconditional trace (`Pipeline.cpp`)

**Win**: ~0.5-2 ms/frame, removes O(commands) syscalls and a 119 MB log.
**Risk**: none (the data is already available under `ISP_DEBUG_VLOG`).
**Change**: gate both blocks behind the existing `ispVlog()`-style check, or
compile them out with `#ifdef MNN_ISP_TRACE`. Reuse
`MNNIspTrace.hpp`'s `mnnIspTraceSink()` rather than opening per call.

### P1 — stop rebuilding the session

Every `resizeSession` that actually changes shape costs a shader recompile
(§2). In softisp the resize decision is already per-slot
(`mnnengine.rs:1798`, `slot.resized_for != Some((w,h))`), so this only bites on
a genuine geometry change — but a profile switch or a preview→capture
resolution change pays 150-300 ms.

**Win**: 150-300 ms of hitch, not steady-state FPS.
**Change**: cache the compiled pipeline across resizes instead of
`mPipelineFactory->reset()` in `onGabageCollect`. `VulkanPipelineFactory::reset`
already keeps old pipelines alive via `SharedPtr`; making the cache
*persistent* across the reset (keyed by shader hash, which `getPipeline`
already does) removes the recompile.

### P2 — `Session_Memory_Cache` instead of `Session_Memory_Collect`

`Session::ModeGroup` defaults to `Interpreter::Session_Memory_Collect`
(`source/core/Session.hpp:34`), which in `Session::resize`
(`Session.cpp:391-396`) calls `onGabageCollect(0)` on **every** runtime after
every realloc. On Vulkan `onGabageCollect` is the
`vkDeviceWaitIdle + clear + pipelineFactory.reset` path from §2.

```cpp
// mnn_wrapper.cpp — before createSession
net->setSessionMode(MNN::Interpreter::Session_Memory_Cache);
```

**Win**: removes a device-wide idle + memory teardown + shader recompile on
every resize. Effectively the upstream-supported form of P1.
**Risk**: higher peak memory (the free list is retained). With
`fp16_storage = true` that is affordable; with fp32 it is not — so pair it
with P3.

### P3 — fp16 tensor storage must stay the default

`VulkanBackend.cpp:176`:
`mUseFP16 = (precision != Precision_High) && device supports fp16`, and
`:368` halves the per-element bytes for every float tensor. Every intermediate
in the ISP chain is float, so at 2592×1940 this is the single largest lever on
peak GPU memory (~38 MB instead of 77 MB for the RGBA display tensor alone).

softisp already defaults `fp16_storage = true` (`mnnengine.rs:224`) and
*forces* it off when a fused `isp.*` op is present
(`mnnengine.rs:1416-1423`). That forcing is the documented correctness
constraint — the `isp.*` shaders declare `float[]` storage buffers and read
halves as floats. **Do not "optimise" it away**; instead shrink the fused-op
chain so more of the graph can run fp16.

### P4 — eliminate CPU-fallback ops (the hidden 2-copy tax)

Per §1.1, any op that lands on the CPU backup backend costs a full
frame GPU→host→GPU. `Pipeline.cpp:1176-1194`'s counter (once the tracing in P0
is gone, keep the count and log it at `debug` level once per frame) tells you
whether any exist. If `cpu>0`, that op is worth more than everything else here.

Target: `vk == total` and `cpu == 0`.

### P5 — fewer, wider dispatches

`kIndirectSegmentOpLimit = 10` (`VulkanBackend.hpp:190`) batches indirect
command buffers 10 ops at a time; `mDirect` is on when
`MNN_GPU_RECORD_BATCH` is not set (`:175`). Each `isp.*` stage is its own
dispatch with its own descriptor set. This is where the existing fusion work
(`IspChainFusion`, the `isp.demosaic_g2_ccm` / `isp.demosaic_edge_ccm` fused
shaders in `~/MNN/source/backend/vulkan/buffer/execution/isp_spv/`) pays:
one dispatch whose shader does demosaic+CCM beats two dispatches.

**Win**: proportional to dispatch count; measured GPU-side, not host-side.
**Already in progress** — see `docs/architecture/PIPELINE_BLOCKS.md`
"MNN Fusion Rules".

### P6 — argument-buffer / tensor sharing between ops

The structural answer to the original `A → B` question: MNN does **not**
support it today (§1). Adding it would mean teaching `Pipeline::_allocForTensor`
to detect "op B's output is never read after this point AND sizes match" and
aliasing `t2` onto `t1`. That is a genuine frame-size memory saving (halves
steady-state peak) but **zero** latency saving — the GPU still does the same
work, and the pool already recycles. Not worth it for FPS; worth it only if
peak GPU memory is the binding constraint.

---

## 5. What to measure first

Before touching anything, confirm the split between host and GPU:

1. Remove the P0 trace, rebuild, re-run the 4K HEAVY benchmark
   (`CLAUDE.md`: 79.8 FPS baseline). If FPS jumps, host overhead was real.
2. Log `total / vk / cpu` command counts at `debug` once per frame (from the
   existing counter at `Pipeline.cpp:1176`). `cpu > 0` → P4 is the priority.
3. `getSessionInfo(MEMORY)` (`mnn_wrapper.cpp:786-830`, exposed as
   `MnnSessionSafe::get_model_info`) before/after a resize: a large drop means
   the allocator is being torn down (P2).
4. `mRuntime->onGetLastGpuTimeMs()` / `ENABLE_VULKAN_TIME_PROFILE`
   (`VulkanBackend.cpp:502-518`) for the per-op GPU breakdown. Compare against
   total `process()` time from `mnnengine.rs` — the delta *is* the host cost
   you are chasing.

---

## 6. Summary

| Question | Answer |
|---|---|
| Does `B` copy `A`'s output? | **No.** Pointer handoff via `workInputs`, zero memcpy. |
| Does it alias it? | **No.** Two separate `deviceId()` buffers; 2× peak for the pair. |
| Is memory reused across the chain? | **Yes**, via `EagerBufferAllocator`'s size-indexed free list and `useCount`-driven release. Steady state is 2× frame, not N×. |
| Biggest available speedup | **Delete the unconditional per-command `fopen`** in `Pipeline.cpp:1229`. |
| Biggest available memory/throughput lever | fp16 storage (**keep it**), then `Session_Memory_Cache` to stop the resize teardown. |
| Biggest structural lever | Fewer `isp.*` dispatches via fusion; no CPU-fallback ops. |
| Buffer aliasing between ops | Genuine memory win, no latency win — only worth it under memory pressure. |
