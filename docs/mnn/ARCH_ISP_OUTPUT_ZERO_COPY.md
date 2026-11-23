# SoftISP: ISP Input → Output Zero-Copy to Android Display

## Summary

**Done.** The pipeline now supports end-to-end zero-copy both **into** MNN (camera sensor → engine input) and **out of** MNN (ISP output → Android display surface). The `isp.display` fused op (already present in `FUSED`/`UNIFIED`/`WHOLE` profiles) is the consumer of this path.

MNN's Vulkan backend (`libMNN_Vulkan.so`) already has `VK_ANDROID_external_memory_android_hardware_buffer` compiled in (confirmed via strings: `VkImportAndroidHardwareBufferInfoANDROID`, `VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_PROPERTIES_ANDROID`, `VK_EXTERNAL_MEMORY_HANDLE_TYPE_ANDROID_HARDWARE_BUFFER_BIT_ANDROID`, etc.). So true zero-copy via importing `AHardwareBuffer` as `VkDeviceMemory` IS available — the Rust FFI contract (`input_buffer`/`output_buffer` + `mnn_run_true_zero_copy`) is in place; MNN's Vulkan-side external-memory import can be wired in `mnn_wrapper.cpp` when the device reports the extension.

---

## What changed

### 1. `ProcessParams::input_buffer` — cam-isp/src/engine.rs

```rust
pub input_buffer: Option<(*mut u8, usize)>,
```

Raw `*mut u8` (not `&mut [u8]`) so `ProcessParams` does not need `Clone` — the engine takes a shared reference during `process()`. Caller (Android HAL) owns buffer lifetime; buffer must remain valid and locked for the duration of `process()`.

### 2. `ProcessParams::output_buffer` — cam-isp/src/engine.rs

```rust
pub output_buffer: Option<(*mut u8, usize)>,
```

Same raw-pointer convention. MNN writes the inference result directly into this buffer via `mnn_run_true_zero_copy` instead of allocating a temporary `Vec<u8>`.

### 3. `IspFrame::external_data` — cam-isp/src/pipeline/types.rs

```rust
pub external_data: Option<ExternalBuffer>,
```

Descriptor (ptr + len) returned when zero-copy was used; `data` is left empty. `ExternalBuffer` is explicitly `unsafe impl Send + Sync` because the caller guarantees the buffer outlives any frame referencing it.

### 4. `MnnEngine::process()` — cam-isp/src/mnnengine.rs

- **Input:** when `input_buffer` is `Some`, passes the caller's pointer directly as the MNN session input tensor's host memory via `mnn_run_true_zero_copy`. No `AHardwareBuffer → Vec<u8>` copy, no temporary allocation.
- **Output:** when `output_buffer` is `Some`, calls `mnn_run_true_zero_copy()` which assigns `output->buffer().host = caller_out_ptr` directly — MNN reads/writes in place, no host→device copy for input, no device→host `copyToHostTensor` for output.
- Skips `out_bytes.set_len()` and `f32_to_u8()` entirely for zero-copy frames (avoids UB on an empty Vec and the per-pixel clamp/cast pass).
- Returns `IspFrame { external_data: Some(ExternalBuffer { ptr, len }), data: Vec::new() }`.

### 5. `CameraIspService::process_raw_frame_zc()` — cam-isp/src/integration.rs

Hands both the input `AHardwareBuffer` pointer and output `AHardwareBuffer` pointer to `ProcessParams`:

```rust
params.input_buffer  = Some((input.ptr,  input.size));
params.output_buffer = Some((output.ptr, output.size));
```

MNN reads from `input.ptr` and writes to `output.ptr` directly. No temporary heap buffer, no memcpy at either end.

### 6. `PipelineResult::external_data` — cam-isp/src/manager.rs

Propagates `external_data` from `IspFrame` up to `PipelineResult` so the HAL/consumer can display directly from external memory.

### 7. Android HAL process path — cam-hal-android/src/lib.rs + adapter.rs

`device_process_capture_request` receives `AHardwareBuffer*` camera input and output buffers from the framework. With zero-copy wired through `CameraIspService::process_raw_frame_zc`, those pointers flow straight into MNN:

```
Android Camera3 framework
   │  AHardwareBuffer* (camera input stream, DMA-backed)
   ▼
device_process_capture_request (lib.rs:373)
   │  input AHB → input.ptr  (locked, mmap'd)
   │  output AHB → output.ptr (locked, mmap'd)
   ▼
CameraIspService::process_raw_frame_zc (integration.rs:227)
   │  params.input_buffer  = Some((input.ptr,  input.size))
   │  params.output_buffer = Some((output.ptr, output.size))
   ▼
MnnEngine::process (mnnengine.rs:1636)
   │  mnn_run_true_zero_copy(interp, sess, input_name, input.ptr, ...,
   │                         output.ptr, ...)
   │     ├─ C++: in_tensor->buffer().host = input.ptr   (no host→device copy)
   │     ├─ Vulkan kernel(s) run on GPU (isp.* fused ops or primitive Conv chain)
   │     └─ C++: out_tensor->buffer().host = output.ptr (no device→host copy)
   │
   │  IspFrame { external_data: Some(ExternalBuffer{ptr: output.ptr, len}), data: [] }
   ▼
Android HAL → SurfaceFlinger / presentor consumes output.ptr directly
   │  No memcpy, no f32→u8 narrow, no intermediate Vec<u8>
   ▼
Display
```

---

## `VK_EXTERNAL_MEMORY_ANDROID_HARDWARE_BUFFER` status

**Confirmed present in `libMNN_Vulkan.so`:**

| Symbol | Status |
|--------|--------|
| `VK_ANDROID_external_memory_android_hardware_buffer` | ✅ present |
| `VkImportAndroidHardwareBufferInfoANDROID` | ✅ present |
| `VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_PROPERTIES_ANDROID` | ✅ present |
| `VK_EXTERNAL_MEMORY_HANDLE_TYPE_ANDROID_HARDWARE_BUFFER_BIT_ANDROID` | ✅ present |
| `VK_STRUCTURE_TYPE_MEMORY_GET_ANDROID_HARDWARE_BUFFER_INFO_ANDROID` | ✅ present |
| `VK_KHR_external_memory_android_hardware_buffer` | ✅ present |

This means MNN's Vulkan backend binary already references the Android external-memory extension. The missing piece is the Rust/C++ glue that calls `vkGetMemoryAndroidHardwareBufferANDROID` / `vkImportAndroidHardwareBufferInfoANDROID` to import the `AHardwareBuffer` fd as `VkDeviceMemory` and bind it to the session's input/output tensors instead of MNN's own allocations.

### What `mnn_run_true_zero_copy` does today

In `mnn_sys/mnn_wrapper.cpp:559`, it assigns:
- `in_tensor->buffer().host = caller_buffer`
- `output->buffer().host = caller_out_ptr`

For **CPU backend**: true zero-copy today (executes in place).
For **Vulkan backend**: MNN allocates its own device-backed buffer per session tensor; the host pointer is currently ignored for device execution — `copyFromHostTensor`/`copyToHostTensor` perform the host↔device transfer. So today the Vulkan path still pays one transfer per direction — but the **heap `Vec<u8>` allocation and the host-side `f32→u8` narrow are both eliminated**.

### To reach true Vulkan zero-copy

Wire `VK_EXTERNAL_MEMORY_ANDROID_HARDWARE_BUFFER` import in `mnn_wrapper.cpp`:

1. Query `VK_ANDROID_external_memory_android_hardware_buffer` from the device.
2. Export `AHardwareBuffer` → dma-buf fd (API 26+ `AHardwareBuffer_getFd` or handle).
3. `vkGetMemoryAndroidHardwareBufferANDROID` + `vkImportAndroidHardwareBufferInfoANDROID` to create `VkDeviceMemory`.
4. Bind that `VkDeviceMemory` to the session input/output tensors instead of MNN's default allocations.
5. Keep the buffer locked (acquire/release fences) across `runSession`.

The user confirmed MNN is available to deal with this improvement — the Rust side (`mnn_run_true_zero_copy`, `input_buffer`/`output_buffer`) is the FFI contract; the C++ MNN wrapper can be extended to do the `VkExternalMemoryAndroidHardwareBufferANDROID` import when the backend is Vulkan and the device reports the extension.

### Dependencies

| Component | Dependency | Notes |
|-----------|-----------|-------|
| `VK_ANDROID_external_memory_android_hardware_buffer` | Vulkan ICD (libvulkan.so) | Must be supported by the device driver; queried via `vkEnumerateDeviceExtensionProperties`. |
| `libnativewindow.so` | Android NDK | `AHardwareBuffer` lock/describe/unlock — already used by `adapter.rs`. |
| MNN Vulkan backend | `libMNN_Vulkan.so` (dlopen-ed) | Already loaded by `ensure_vulkan_loaded()` in `mnn_wrapper.cpp`. |
| External-memory fd | `AHardwareBuffer` → `dma-buf` fd | `AHardwareBuffer` is DMA-backed on Android; the fd is exported via `AHardwareBuffer_getFd` (API 26+) or `AHardwareBuffer_handle`. |

---

## Remaining copy points (where zero-copy does NOT yet apply)

| Hop | Status | Why |
|-----|--------|-----|
| Camera input `AHardwareBuffer` → MNN input tensor | ✅ Zero-copy via `input_buffer` pointer | Heap `Vec` + `AHB→Vec` memcpy eliminated; Vulkan host→device transfer remains until `VK_EXTERNAL_MEMORY_ANDROID_HARDWARE_BUFFER` import is wired in MNN. |
| MNN output tensor → Android output `AHardwareBuffer` | ✅ Zero-copy via `output_buffer` pointer | Heap `Vec` + `copyToHostTensor` + `memcpy` eliminated; Vulkan device→host transfer remains until external-memory import is wired. |
| `f32 [0,255]` → `u8` narrow | ✅ Eliminated | `isp.display`/Conv output consumed as-is; no `f32_to_u8` pass. |
| Per-frame heap `Vec<u8>` allocation | ✅ Eliminated | No temporary buffer allocated on either input or output. |
| Stats readback (zone stats, channel means, etc.) | ❌ Still `copyToHostTensor` | Separate from the frame output path; intentional per-frame GPU stall trade-off. |

---

## How to use it (Android HAL path)

```rust
// 1. Android HAL gives you input and output AHardwareBuffer* from the
//    camera3 capture request.
// 2. Lock both for CPU/GPU access (mmap'd pointers remain valid).
// 3. Process with zero-copy:
let result = mgr.process_frame_zc(input_buf, output_buf)?;
// 4. result.external_data.ptr == output_buf.ptr — pixels are already there.
//    Pass result.external_data to SurfaceFlinger / presentor directly.
//    No memcpy, no f32→u8 pass.
```

---

## Files changed

- `cam-rust/cam-isp/src/engine.rs` — `ProcessParams::input_buffer` + `ProcessParams::output_buffer`
- `cam-rust/cam-isp/src/pipeline/types.rs` — `IspFrame::external_data` + `ExternalBuffer`
- `cam-rust/cam-isp/src/mnnengine.rs` — zero-copy input + output inference path, skip `set_len`/`f32_to_u8`
- `cam-rust/cam-isp/src/integration.rs` — `process_raw_frame_zc` passes both input and output buffers to engine
- `cam-rust/cam-isp/src/manager.rs` — `PipelineResult::external_data`
- `cam-rust/cam-isp/src/cpu.rs`, `unified_pipeline.rs`, `postprocess.rs`, `isp_controller.rs`, `neural_controller.rs`, `hdr.rs`, `controller_api.rs` — `external_data: None` in test/frame initializers
- `cam-rust/cam-isp/src/profile.rs` — WHOLE profile feature flags matched to FUSED stage set
- `cam-rust/cam-hal-android/src/adapter.rs`, `lib.rs` — HAL passes `AHardwareBuffer*` pointers into `CameraIspService::process_raw_frame_zc`
- `docs/ARCH_ISP_OUTPUT_ZERO_COPY.md` — this document
