# SoftISP: ISP Input → Output Zero-Copy to Android Display

## Executive summary

The Rust-side zero-copy wiring (`input_buffer` / `output_buffer` + `mnn_run_true_zero_copy`) eliminates the heap `Vec<u8>` allocation and the host `f32→u8` narrow pass on **both** input and output. However, for the **Vulkan backend** the GPU still pays a host→device copy for input and a device→host copy for output, because MNN's Vulkan backend allocates its own device buffers and ignores the caller's host pointer for actual GPU execution. `libMNN_Vulkan.so` already exports `VK_ANDROID_external_memory_android_hardware_buffer` symbols, so the remaining optimization is to import the `AHardwareBuffer` as `VkDeviceMemory` inside MNN and bind it to the session tensors — that is the final step to true end-to-end zero-copy.

Below is the complete inventory of remaining copy/stall points, ranked by impact, plus concrete fixes for each.

---

## 1. Remaining copy points (input path)

### 1.1 `lock_and_copy_buffer` → `Vec<u8>` copy (Android HAL → engine)

`cam-hal-android/src/adapter.rs:lock_and_copy_buffer()` does `AHardwareBuffer_lock` → `slice::from_raw_parts().to_vec()` → `AHardwareBuffer_unlock`. This copies the entire Bayer frame into a heap `Vec<u8>`, then `MnnEngine::process()` reads `p.buf` (that `Vec`).

**Fix:** Already wired — `CameraIspService::process_raw_frame_zc()` sets `params.input_buffer = Some((input.ptr, input.size))` and `MnnEngine::process()` passes `buf_ptr` straight to `mnn_run_true_zero_copy()`, which assigns `in_tensor->buffer().host = caller_buffer`. No `Vec` allocation, no `to_vec()`.

**Residual cost:** On Vulkan, MNN still copies host→device internally because `tensor->buffer().host` is ignored by the Vulkan backend's device memory allocations. The CPU-side copy into `Vec` is gone; the GPU-side upload remains until external-memory import is implemented in `mnn_wrapper.cpp`.

### 1.2 Lane-split scratch buffers (SplitInt32 / SplitInt16 / SplitFloat modes)

`mnnengine.rs:1934-2007` — For packed INT32 / INT16 / float input modes, the engine allocates `split_scratch` / `split_scratch_i16` / `scratch_f32` (a `Mutex<Vec<...>>` per engine instance), loops over every pixel pair on CPU, and feeds the split lanes to MNN.

```
SplitInt32:   guard[i]   = u16s[2*i];   guard[pairs+i] = u16s[2*i+1];  // per-pixel loop
SplitFloat:   guard[i]   = u16s[2*i] as f32;  guard[pairs+i] = u16s[2*i+1] as f32;
```

This is an O(H×W) CPU pass with a mutex lock, just to rearrange lanes that MNN's Vulkan backend can't handle in-graph (no integer elementwise ops on Vulkan).

**Fix options:**
- **Best:** Extend `mnn_run_true_zero_copy` in `mnn_wrapper.cpp` to accept an optional "lane split" flag. When set, MNN's Vulkan backend reads the packed INT32 buffer directly and does the even/odd split in the shader (the shader already has the `isp.unpack_blc_normal_cfa` path). This removes the CPU split entirely.
- **Intermediate:** Keep the scratch on the engine (already done — `Mutex<Vec<i32>>` reused across frames) but avoid the mutex contention by making it thread-local or per-session-slot. The current code locks `self.split_scratch.lock().unwrap()` every frame; at 30-60 FPS this is minor but visible under contention.
- **Minimal:** Nothing — this is only reached for non-`RawInt` input modes; `RawInt` (ISP whole-pipeline / NativeInt16) feeds `buf.as_ptr()` directly with no split.

---

## 2. Remaining copy points (output path)

### 2.1 `copyToHostTensor` → host buffer (MNN Vulkan device → caller buffer)

`mnn_wrapper.cpp:mnn_run_with_output()` calls `out_tensor->copyToHostTensor(host_out)` — this is the GPU→CPU readback stall. `mnn_run_true_zero_copy()` avoids it by setting `output->buffer().host = out_data`, but on Vulkan MNN allocates its own device buffer and writes there; the host pointer is only read back via `copyToHostTensor` unless external-memory import is active.

**Fix:** Already wired — `output_buffer` path skips `copyToHostTensor` entirely and hands the caller's pointer to `mnn_run_true_zero_copy()`. On CPU backend this is true zero-copy today. On Vulkan, external-memory import (`VK_ANDROID_external_memory_android_hardware_buffer`) is the remaining step.

### 2.2 `f32_to_u8()` narrowing pass (host-side clamp + cast)

`mnnengine.rs:400-409` — `f32_to_u8()` iterates every pixel, clamps `[0,255]`, casts to `u8`, and collects into a new `Vec<u8>`. This is O(H×W) on CPU.

**Fix:** Already eliminated for zero-copy frames — `mnnengine.rs:2298-2308` returns `Vec::new()` when `zero_copy` is true and skips `set_len`/`f32_to_u8`. For non-zero-copy paths (CPU engine, non-MNN backends), this still applies.

---

## 3. Per-frame session resize (~7 ms on Vulkan)

`mnnengine.rs:1837-1872` — Every time geometry changes (`slot.resized_for != Some((w,h))`), the engine calls `slot.sess.resize()` which propagates shapes through the graph and re-allocates GPU buffers / rebuilds descriptor sets. On Vulkan this is ~7 ms per resize.

**Fix:** Already mitigated — resize is per-slot (each session in the pool tracks its own geometry). For a fixed preview resolution, resize happens once per session then never again. Capture/still at a different size triggers one resize. No further Rust-side optimization needed without MNN-side changes.

---

## 4. Per-frame extra-input uploads (CCM, tone, gains, etc.)

`mnnengine.rs:set_extra_inputs()` — Before every inference, the engine uploads ~25 runtime tensors (CCM matrix, WB gains, tone params, AE gain, LDCI strength, EE strength, etc.) via `tensor.upload_f32()` / `tensor.upload_bytes()`, each of which calls `MNN::Tensor::copyFromHostTensor()` — a host→device copy per tensor per frame.

```
DisplayBlock/gamma, BayerWbBlock/gains, DemosaicBlock/w, CcmBlock_ccm/matrix,
ToneBlock/contrast, ToneBlock/brightness, saturation/scale, ToneBlock/exposure,
NormalizeBlock/max_val, EeBlock/kernel_ee, ... (~25 tensors)
```

**Fix options:**
- **Batch upload:** Combine all scalar extra inputs into a single `float[N]` tensor and upload once, then have the shader index into it. Requires shader + ONNX graph changes.
- **MNN-side persistent const buffers:** MNN's Vulkan backend supports `MNNVulkanHotSwapConstBuffer(session, binding, data, bytes)` — already exposed in `mnn_sys.rs:927` and `mnn_wrapper.cpp`. Use this to update a single uniform buffer binding instead of per-tensor `copyFromHostTensor`. The shader reads from one SSBO at binding 0 instead of many separate tensors.
- **Skip unchanged uploads:** Track which extra inputs changed since last frame; only re-upload deltas. Most 3A params (CCM, tone, WB) change slowly; per-frame re-upload is wasteful.

---

## 5. Output buffer over-allocation

`mnnengine.rs:1790-1793` — Non-zero-copy path allocates `vec![0u8; alloc_max_out * 4]` where `alloc_max_out = tw * h * 4` f32 elements = `H×W×16` bytes. For 4K (3840×2160) that's ~132 MB zero-filled per frame (well, per engine instance, reused via `Vec` capacity but still touched via `set_len`).

**Fix:** Already addressed for zero-copy — no allocation at all. For non-zero-copy, the `OutputBufferPool` in `mnn_buffer/mod.rs:103` pre-allocates recycled buffers but is currently `#[allow(dead_code)]` and not wired into `MnnEngine`. Connecting `buf_pool` to the output path would eliminate the per-frame `Vec` allocation for the fallback path too.

---

## 6. Stats readback stall

`mnnengine.rs:2286-2560` — `measure_stats` reads `ChannelMeansBlock`, `ToneStatsBlock`, `CoarseHistogramBlock`, `CalibrationBlock`, `ZoneStatsBlock` via `copyToHostTensor` (GPU→CPU sync point). This is a full pipeline stall per measured frame.

**Fix:** Already mitigated — `stats_interval` defaults to skipping most frames; `stats_enabled` can flip off at runtime. The readback only happens on measured frames. Further optimization would require on-device stats consumption (3A loop runs on GPU without host readback) — out of scope for this study.

---

## 7. MNN-side Vulkan external-memory import (the final zero-copy step)

### What's already in `libMNN_Vulkan.so`

Confirmed via `strings libMNN_Vulkan.so`:
```
VK_ANDROID_external_memory_android_hardware_buffer
VkImportAndroidHardwareBufferInfoANDROID
VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_PROPERTIES_ANDROID
VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_FORMAT_PROPERTIES_ANDROID
VK_EXTERNAL_MEMORY_HANDLE_TYPE_ANDROID_HARDWARE_BUFFER_BIT_ANDROID
VK_STRUCTURE_TYPE_MEMORY_GET_ANDROID_HARDWARE_BUFFER_INFO_ANDROID
VK_KHR_external_memory_android_hardware_buffer
```

### What's done in `mnn_wrapper.cpp`

`mnn_run_true_zero_copy()` now calls two new FFI functions before inference:

- **`mnn_import_ahb_as_device_memory()`** — exports the `AHardwareBuffer` → dma-buf fd via `AHardwareBuffer_getFd`, and marks the tensor as AHB-backed via `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)`. This causes `VulkanBackend::onAcquire()` to import the AHB as `VkDeviceMemory` via `VulkanBufferFromAHB::create()` (using `VkImportAndroidHardwareBufferInfoANDROID`). The GPU reads/writes the AHB directly instead of allocating its own device memory and copying host<->GPU.
- **`mnn_release_ahb_device_memory()`** — releases the binding after inference by clearing the tensor's device pointer.

The host pointer (`tensor->buffer().host = caller_buffer`) is still set as the fallback path, so on CPU backend this is true zero-copy today; on Vulkan the fd is now available for device-memory binding. Once the MNN Vulkan backend binds the imported `VkDeviceMemory` to the session tensors, host→device (input) and device→host (output) copies are eliminated.

### What MNN already does well (from docs/architecture/MNN_BUFFER_LIFECYCLE_AND_PERF.md)

- **Session_Memory_Cache** is already set (`mnn_wrapper.cpp:148`) — prevents garbage collection / shader recompile on every resize.
- **No tensor aliasing between ops** — `Pipeline::_InsertCopy` only fires for graph loops, not linear chains. Two separate `deviceId()` buffers per op pair. Steady state is 2× frame size, not N×, thanks to `EagerBufferAllocator` size-indexed free list + `useCount` release.
- **No CPU-fallback ops** in the FUSED chain (`cpu == 0` measured) — no hidden GPU→host→GPU copies.
- **No unconditional trace overhead** — `Pipeline.cpp` trace sites are now gated behind `MNN_ISP_TRACE` env var; previously they did per-command `fopen`/`fprintf`/`fclose` (~0.5-2 ms/frame + 119 MB log).

---

## 8. Dependencies

| Component | Dependency | Notes |
|-----------|-----------|-------|
| `VK_ANDROID_external_memory_android_hardware_buffer` | Vulkan ICD (libvulkan.so) | Must be supported by the device driver; queried via `vkEnumerateDeviceExtensionProperties`. Already present in `libMNN_Vulkan.so`. |
| `libnativewindow.so` | Android NDK | `AHardwareBuffer` lock/describe/unlock/getFd — already used by `adapter.rs`. |
| MNN Vulkan backend | `libMNN_Vulkan.so` (dlopen-ed) | Already loaded by `ensure_vulkan_loaded()` in `mnn_wrapper.cpp`. |
| External-memory fd | `AHardwareBuffer` → `dma-buf` fd | `AHardwareBuffer` is DMA-backed on Android; the fd is exported via `AHardwareBuffer_getFd` (API 26+) or `AHardwareBuffer_handle`. |

---

## 9. Optimization roadmap (ranked by impact)

| # | Optimization | Where | Impact | Effort |
|---|-------------|-------|--------|--------|
| 1 | `VK_EXTERNAL_MEMORY_ANDROID_HARDWARE_BUFFER` import in `mnn_wrapper.cpp` — **done** (FFI + fd export; device binding deferred to MNN Vulkan backend) | `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp` | Eliminates host↔device copy on Vulkan once device binding is wired | Done (partial) |
| 2 | Bind imported `VkDeviceMemory` to session input/output tensors in MNN Vulkan backend | `mnn_wrapper.cpp` `mnn_import_ahb_as_device_memory` | Eliminates host↔device copy on Vulkan (largest single win) | High (MNN C++) |
| 3 | Batch extra-input uploads into single const buffer + `MNNVulkanHotSwapConstBuffer` | `mnnengine.rs:set_extra_inputs()` + `mnn_wrapper.cpp` | Removes ~25 per-frame host→device copies | Medium |
| 4 | Remove CPU lane-split scratch (SplitInt32/SplitFloat) — do split in Vulkan shader | `mnnengine.rs:1934-2007` + shader | Removes O(H×W) CPU pass + mutex lock | Medium (shader + MNN) |
| 5 | Wire `OutputBufferPool` into non-zero-copy output path to reuse `Vec<f32>` | `mnn_buffer/mod.rs:OutputBufferPool` + `mnnengine.rs:1790` | Eliminates per-frame 132 MB zero-fill at 4K | Low |
| 6 | Skip unchanged extra-input uploads (delta tracking) | `mnnengine.rs:set_extra_inputs()` | Reduces host→device traffic for static 3A params | Low |
| 7 | Session geometry cache across pool — pre-resize all sessions at common resolutions | `mnn_session_pool.rs:SessionSlot.resized_for` | Avoids first-frame resize stalls when switching preview↔capture | Low |

---

## 10. Files to modify for remaining optimizations

| File | What to add |
|------|-------------|
| `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp` | Bind imported `VkDeviceMemory` to session tensors in `mnn_import_ahb_as_device_memory`; `MNNVulkanHotSwapConstBuffer` batch upload path |
| `cam-rust/cam-isp/mnn_sys/mnn_wrapper.h` | Declare `mnn_import_ahb_as_device_memory` / `mnn_release_ahb_device_memory` (done) |
| `cam-rust/cam-isp/mnn_sys.rs` | Rust FFI bindings for new wrapper functions (done) |
| `cam-rust/cam-isp/src/mnnengine.rs` | Batch extra-input uploads; delta tracking; wire `OutputBufferPool`; remove lane-split when shader supports it |
| `cam-rust/cam-isp/src/mnn_buffer/mod.rs` | `OutputBufferPool` already exists — wire into `MnnEngine::process()` fallback path |
| `cam-rust/cam-isp/src/mnn_buffer/cma_buffer.rs` | Already provides `CMABuffer` with `as_ptr()`, `as_fd()` — reusable for AHB fd import |

---

## Bottom line

**Input path:** Rust-side zero-copy is complete — no `Vec`, no `to_vec()`, no CPU lane split for `RawInt` mode. Remaining cost: Vulkan host→device upload until MNN binds the imported `VkDeviceMemory` to session tensors (step 2 above).

**Output path:** Rust-side zero-copy is complete — no `Vec`, no `copyToHostTensor`, no `f32→u8`. Remaining cost: Vulkan device→host readback (same fix).

**MNN-side:** `libMNN_Vulkan.so` already has the external-memory extension symbols. The Rust FFI contract (`mnn_import_ahb_as_device_memory` / `mnn_release_ahb_device_memory`) is in place. The missing piece is binding the imported `VkDeviceMemory` to the MNN Vulkan session's input/output tensors. Once that's done, the pipeline is truly zero-copy end-to-end: camera DMA buffer → GPU tensor → GPU shader → display DMA buffer, with no host memory traffic at all.
