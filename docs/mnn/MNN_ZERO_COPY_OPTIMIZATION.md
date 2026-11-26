# SoftISP: ISP Input → Output Path — MNN-side Optimization Study

## Executive summary

The Rust-side zero-copy wiring (`input_buffer` / `output_buffer` + `input_ahb` / `output_ahb` + `mnn_run_true_zero_copy` / `mnn_run_ahb_output`) eliminates the heap `Vec<u8>` allocation and the host `f32→u8` narrow pass on **both** input and output. For the **Vulkan backend**, input and output `AHardwareBuffer`s are now imported as Vulkan device memory via `Tensor::setDevicePtr(..., MNN_MEMORY_AHARDWAREBUFFER)` **before** `resizeSession()`, so MNN's `VulkanBackend::onAcquire()` imports them through `VulkanBufferFromAHB::create()` (`VkImportAndroidHardwareBufferInfoANDROID`). Host→device (input) and device→host (output) copies are skipped when the tensor carries `MNN_MEMORY_AHARDWAREBUFFER`. `libMNN_Vulkan.so` already exports `VK_ANDROID_external_memory_android_hardware_buffer`, so this path works on Vulkan-capable devices without a shared Rust-side Vulkan context — MNN's fork owns the VkDevice and performs the external-memory import itself.

Below is the complete inventory of remaining copy/stall points, ranked by impact, plus concrete fixes for each.

---

## 1. Remaining copy points (input path)

### 1.1 Camera de-stride copy → `Vec<u16>` (media NDK → engine)

`anw/src/camera.rs:copy_pixels()` reads plane 0 via `AImage_getPlaneData` and de-strides every pixel into a heap `Vec<u16>`, then `MnnEngine::process()` reads `p.buf` (that `Vec`). For RAW16 this is a per-pixel byte swap; for YUV it expands 8-bit luma into `u16`.

**Status:** Not yet fixed. The media-NDK `AImage` carries an `AHardwareBuffer` (`sys::AImage_getHardwareBuffer`), but the app path currently copies pixels out instead of passing the `AHardwareBuffer*` directly as `ProcessParams::input_ahb`.

**Fix:** In `anw/src/pipeline_runner.rs`, after `AImageReader_acquireLatestImage`, call `sys::AImage_getHardwareBuffer(image, &ahb)` to get the producer-owned `AHardwareBuffer*`, then pass it via `params.input_ahb = Some(AhbBinding::new(ahb as *mut c_void, capacity, fences))`. The caller must keep the `AImage` (and thus its AHB) alive for the duration of inference. `AHardwareBuffer_acquire` may be needed if the binding outlives the `AImage` (ncnn-style ref-counting). Skip the `copy_pixels()` de-stride entirely when `input_ahb` is set, feeding the AHB pointer straight to `mnn_run_true_zero_copy` / `mnn_run_ahb_output`.

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

**Status:** Fixed. `mnn_run_with_output()` calls `out_tensor->copyToHostTensor(host_out)` — that path is only used for non-zero-copy frames. `mnn_run_true_zero_copy()` and `mnn_run_ahb_output()` skip `copyToHostTensor` entirely: `mnn_run_ahb_output()` runs the session and waits on `output->wait(MAP_TENSOR_READ, true)` so the GPU has already written into the caller's AHB mapping; `mnn_run_true_zero_copy()` hands the caller's host pointer directly for CPU-backed sessions.

### 2.2 `f32_to_u8()` narrowing pass (host-side clamp + cast)

**Status:** Fixed for zero-copy frames. `mnnengine.rs:400-409` — `f32_to_u8()` iterates every pixel, clamps `[0,255]`, casts to `u8`, and collects into a new `Vec<u8>`. This is O(H×W) on CPU. Zero-copy frames return `Vec::new()` and skip `f32_to_u8` (`mnnengine.rs:2799-2808`). For non-zero-copy paths (CPU engine, non-MNN backends), this still applies.

---

## 3. Per-frame session resize (~7 ms on Vulkan)

**Status:** Mitigated. `mnnengine.rs:2295-2313` — `slot.resized_for != Some((w,h))` gates `resize()` per session slot. For a fixed preview resolution, resize happens once per session then never again. Capture/still at a different size triggers one resize. No further Rust-side optimization needed without MNN-side changes.

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

**Status:** Addressed for zero-copy — no allocation at all (`zero_copy` path returns `Vec::new()` and uses `IspFrame::new_external`). For non-zero-copy, the `OutputBufferPool` in `mnn_buffer/mod.rs:103` pre-allocates recycled buffers but is currently `#[allow(dead_code)]` and not wired into `MnnEngine`. Connecting `buf_pool` to the output path would eliminate the per-frame `Vec` allocation for the fallback path too.

---

## 6. Stats readback stall

`mnnengine.rs:2286-2560` — `measure_stats` reads `ChannelMeansBlock`, `ToneStatsBlock`, `CoarseHistogramBlock`, `CalibrationBlock`, `ZoneStatsBlock` via `copyToHostTensor` (GPU→CPU sync point). This is a full pipeline stall per measured frame.

**Fix:** Already mitigated — `stats_interval` defaults to skipping most frames; `stats_enabled` can flip off at runtime. The readback only happens on measured frames. Further optimization would require on-device stats consumption (3A loop runs on GPU without host readback) — out of scope for this study.

---

## 7. MNN-side Vulkan external-memory import — landed

### What's already in `libMNN_Vulkan.so`

Confirmed via `strings`:
```
VK_ANDROID_external_memory_android_hardware_buffer
VkImportAndroidHardwareBufferInfoANDROID
VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_PROPERTIES_ANDROID
VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_FORMAT_PROPERTIES_ANDROID
VK_EXTERNAL_MEMORY_HANDLE_TYPE_ANDROID_HARDWARE_BUFFER_BIT_ANDROID
VK_STRUCTURE_TYPE_MEMORY_GET_ANDROID_HARDWARE_BUFFER_INFO_ANDROID
VK_KHR_external_memory_android_hardware_buffer
```

### What's landed in `mnn_wrapper.cpp`

**`mnn_import_ahb_as_device_memory()`** — validates the caller's `AHardwareBuffer*` via `AHardwareBuffer_getFd` (diagnostic only, fd closed immediately) and `AHardwareBuffer_describe` (capacity check), then calls `tensor->setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)`. This marks the tensor so MNN's `VulkanBackend::onAcquire()` imports it via `VulkanBufferFromAHB::create()` (`VkImportAndroidHardwareBufferInfoANDROID` + dedicated allocation + `vkBindBufferMemory`) at resize time. The wrapper does not allocate or release the AHB — the caller owns it.

**`mnn_run_true_zero_copy()`** — input `copyFromHostTensor` is now guarded by `in_tensor->buffer().flags != MNN_MEMORY_AHARDWAREBUFFER`. When the input tensor was bound to an AHB before resize, MNN's Vulkan backend already imported the AHB as device memory in `onAcquire()`; the caller's buffer points at the AHB's CPU mapping (same physical memory), so the host→device upload is skipped entirely. Same guard applied in `mnn_run_ahb_output()` for the input path.

**`mnn_run_ahb_output()`** — output path already skips `copyToHostTensor`; `output->wait(MAP_TENSOR_READ, true)` makes GPU writes visible to the caller's CPU mapping.

### Shared Vulkan context

No shared Vulkan context is required from the Rust glue layer. MNN's fork owns the VkDevice (created internally by `mnn_session_create` → `ensure_vulkan_loaded()` → `dlopen libMNN_Vulkan.so`) and performs the external-memory import itself. The Rust side only needs to:
1. Allocate or obtain the `AHardwareBuffer*` (caller-owned).
2. Pass the pointer via `AhbBinding.ptr`.
3. Call `mnn_import_ahb_as_device_memory` before `resizeSession`.

If the app wants to share its *own* VkDevice with MNN (e.g. for a custom pre/post-process shader reading the same AHB), that would require MNN's `sharedContext` mechanism (`BackendConfig` + `MNNVulkanSetSessionWorkgroup` etc.) — not currently wired, and not needed for the AHB zero-copy path itself.

---

## 8. Optimization roadmap (ranked by impact)

| # | Optimization | Where | Impact | Effort |
|---|-------------|-------|--------|--------|
| 1 | Camera: pass `AImage_getHardwareBuffer` → `input_ahb`, skip `copy_pixels()` de-stride | `anw/src/camera.rs`, `anw/src/pipeline_runner.rs` | Eliminates full-frame input de-stride copy | Medium (media NDK) |
| 2 | Batch extra-input uploads into single const buffer + `MNNVulkanHotSwapConstBuffer` | `mnnengine.rs:set_extra_inputs()` + `mnn_wrapper.cpp` | Removes ~25 per-frame host→device copies | Medium |
| 3 | Remove CPU lane-split scratch (SplitInt32/SplitFloat) — do split in Vulkan shader | `mnnengine.rs:1934-2007` + shader | Removes O(H×W) CPU pass + mutex lock | Medium (shader + MNN) |
| 4 | Wire `OutputBufferPool` into non-zero-copy output path to reuse `Vec<f32>` | `mnn_buffer/mod.rs:OutputBufferPool` + `mnnengine.rs` | Eliminates per-frame 132 MB zero-fill at 4K | Low |
| 5 | Skip unchanged extra-input uploads (delta tracking) | `mnnengine.rs:set_extra_inputs()` | Reduces host→device traffic for static 3A params | Low |
| 6 | Session geometry cache across pool — pre-resize all sessions at common resolutions | `mnn_session_pool.rs:SessionSlot.resized_for` | Avoids first-frame resize stalls when switching preview↔capture | Low |
| 7 | Presentation: avoid per-pixel CPU BGRA→RGBA blit; present from `AhbView` mapping directly | `anw/src/presentation.rs` | Eliminates final host-side copy before display | Medium |
| 8 | Real sync fences instead of CPU `AtomicBool` busy flags | `anw/src/ahb_output.rs`, `cam-ipa::fence.rs` | Correct GPU/CPU coherency ordering | Medium |

---

## 9. Files modified for landed zero-copy

| File | Change |
|------|--------|
| `cam-rust/cam-isp/src/mnnengine.rs` | `input_ahb` bound before `resize()` in `prepare()`; per-frame input import/release removed from `run_prepared()`; `copyFromHostTensor` skipped for AHB-bound input tensors |
| `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp` | `mnn_run_true_zero_copy()` and `mnn_run_ahb_output()` guard `copyFromHostTensor` on `in_tensor->buffer().flags != MNN_MEMORY_AHARDWAREBUFFER` |
| `docs/isp/ZERO_COPY_NCNN_COMPARISON.md` | ncnn wiki comparison + SoftISP state after changes |

---

## Bottom line

**Input path:** Rust-side zero-copy is complete for `RawInt` mode — no `Vec`, no `to_vec()`, no CPU lane split. Vulkan host→device upload is eliminated when `input_ahb` is bound before resize (tensor flags == `MNN_MEMORY_AHARDWAREBUFFER`). Remaining cost: camera de-stride copy into `Vec<u16>` (fix via media NDK `AImage_getHardwareBuffer` → `input_ahb`).

**Output path:** Rust-side zero-copy is complete — no `Vec`, no `copyToHostTensor`, no `f32→u8`. Vulkan device→host readback is eliminated when `output_ahb` is bound before resize (output AHB path uses `mnn_run_ahb_output` with no readback).

**MNN-side:** `libMNN_Vulkan.so` already has the external-memory extension symbols, and `mnn_wrapper.cpp` now wires `setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` before resize so `VulkanBackend::onAcquire()` performs the `VkImportAndroidHardwareBufferInfoANDROID` import. The pipeline is zero-copy from the MNN FFI boundary onward: camera DMA buffer → GPU tensor → GPU shader → display DMA buffer, with no host memory traffic at the MNN boundary. The remaining host-side copies are the camera de-stride and the presentation blit (roadmap items #1 and #7).
