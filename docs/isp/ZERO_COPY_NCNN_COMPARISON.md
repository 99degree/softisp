# SoftISP Zero-Copy: ncnn Comparison, Analysis, and Changes

## 1. How ncnn does zero-copy (from wiki)

The ncnn wiki ("use ncnn with Android hardware buffer") documents importing a camera `AHardwareBuffer` directly into Vulkan as a `VkImageMat`, avoiding the host→device copy that `Mat::from_pixels` would incur.

### ncnn flow
```
AImageReader (GPU_SAMPLED_IMAGE usage)
  → AHardwareBuffer (YCbCr 420)
  → VkAndroidHardwareBufferImageAllocator (wraps AHB pointer)
  → VkImageMat::from_android_hardware_buffer (VkImage from AHB external memory)
  → ImportAndroidHardwareBufferPipeline (VkImportAndroidHardwareBufferInfoANDROID
     + samplerYcbcrConversion from AHB externalFormat)
  → VkCompute::record_import_android_hardware_buffer (convert_ycbcr.comp:
     YCbCr→RGB [0,255], elemsize=4 elempack=1)
  → VkMat dst → ex.input(dst)
```

### Key ncnn details
- Android API 26+, `VK_ANDROID_external_memory_android_hardware_buffer`, ncnn built with `-DANDROID_PLATFORM=android-26` + `-DNCNN_VULKAN=ON`.
- `AImageReader_newWithUsage` with `AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE` — `AImageReader_new` does not set it.
- Read-only input import; output uses a separate exportable buffer + `vkGetMemoryAndroidHardwareBufferANDROID`.
- Per-frame `pipe.create` is expensive (~24 ms) — ncnn caches allocator+pipeline+`VkImageMat` per `AHardwareBuffer*` pointer.
- `ex.input(VkMat)` does NOT auto-convert format/elempack — pre-cast via `convert_packing` before `ex.input`.
- Ref-counting: caller must `AHardwareBuffer_acquire` if allocator outlives the `AImage`.

---

## 2. SoftISP current zero-copy state (before changes)

### What already landed
| Item | Location |
|------|----------|
| `ProcessParams::{input_buffer, output_buffer, input_ahb, output_ahb}` non-owning views | `cam-rust/cam-isp/src/engine.rs` |
| `AhbBinding` + `AhbFenceTokens` (generation tracking) | `cam-rust/cam-types/src/lib.rs` |
| `mnn_import_ahb_as_device_memory()` / `mnn_release_ahb_device_memory()` / `mnn_run_ahb_output()` | `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp` + `mnn_sys.rs` |
| `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` → MNN Vulkan `onAcquire` → `VulkanBufferFromAHB` | MNN fork |
| `AhbOutputSlot` — one per worker, BLOB AHB allocated+locked once, mapping kept for worker lifetime | `anw/src/ahb_output.rs` |
| Output AHB bound before `resizeSession` (import-before-resize) | `cam-rust/cam-isp/src/mnnengine.rs:prepare()` |
| `mnn_run_ahb_output` (no `copyToHostTensor`) on AHB output path | `mnnengine.rs:run_prepared()` |
| Zero-copy frames skip `f32_to_u8` narrowing and `copyToHostTensor`; `IspFrame::new_external` carries caller pointer | `mnnengine.rs` |

### What was NOT real zero-copy (gaps)
1. **Input AHB imported per-frame in `run_prepared()`**, not cached (ncnn caches per-AHB).
2. **Input AHB bound after resize** — too late for Vulkan `onAcquire` import; host→device copy still happened.
3. **`mnn_run_true_zero_copy` always called `copyFromHostTensor`** even when the input tensor was AHB-bound — redundant host→device upload.
4. Camera still de-strides RAW16 into `Vec<u16>` (`anw/src/camera.rs:copy_pixels()`).
5. Presentation does per-pixel CPU BGRA→RGBA blit (`anw/src/presentation.rs:present()`).
6. Fences are CPU `AtomicBool` busy flags, not real sync fds.
7. `mnn_wrapper.h` claimed wrapper-side fd/VkDeviceMemory import steps that the implementation did not do (only `setDevicePtr`).

---

## 3. Comparison: ncnn vs SoftISP

| Aspect | ncnn (wiki) | SoftISP (before) | SoftISP (after changes) |
|--------|-------------|------------------|-------------------------|
| Import mechanism | `VkAndroidHardwareBufferImageAllocator` + pipeline + `VkImageMat` | `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` → MNN `onAcquire` | Same (MNN fork path) |
| Import timing | Cached per-AHB, created once, reused per frame | Output: before resize; Input: per-frame in `run_prepared` | **Both bound before resize in `prepare()`; input not re-imported per frame** |
| AImageReader usage | `newWithUsage` + `GPU_SAMPLED_IMAGE` | Not in SoftISP path | N/A |
| Color conversion | Shader `convert_ycbcr.comp` during import | ISP graph handles color | Same |
| Output path | Separate exportable buffer + `vkGetMemoryAndroidHardwareBufferANDROID` | `AhbOutputSlot` BLOB AHB mapped once, `mnn_run_ahb_output` no readback | Same |
| Host→device input copy | AHB mapped directly as device memory | `copyFromHostTensor` always | **Skipped when tensor flags == `MNN_MEMORY_AHARDWAREBUFFER`** |
| `ex.input` auto-convert | No — pre-cast via `convert_packing` | N/A (MNN tensor binding) | N/A |
| Ref-counting | Caller `AHardwareBuffer_acquire` if allocator outlives AImage | Slot owns AHB for worker lifetime; release on Drop | Same |
| Fences | N/A | CPU busy flag | CPU busy flag (real fds future work) |

---

## 4. Changes made

### 4.1 `cam-rust/cam-isp/src/mnnengine.rs` — Bind input AHB before resize

**New:** `input_ahb` validity check alongside `output_ahb`. The input binding is validated for capacity before reuse.

**Changed in `prepare()`:** input AHB is now imported via `mnn_import_ahb_as_device_memory` **before** `slot.sess.resize()` (same import-before-resize ordering as output). Uses `self.frame_input_c` (built at `build()`) as the tensor name. If the frame input name is unknown, import is skipped with a warning and the host-upload fallback applies.

**Changed in `run_prepared()`:** removed the per-frame input AHB import and the corresponding `mnn_release_ahb_device_memory` call. The input AHB is bound once in `prepare()` and stays bound for the worker lifetime (matching the output binding pattern). Only the output AHB (when not prepared-bound) is imported/released per call.

### 4.2 `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp` — Skip host→device copy for AHB-bound input tensors

**`mnn_run_true_zero_copy`:** the input `copyFromHostTensor` call is now guarded by `in_tensor->buffer().flags != MNN_MEMORY_AHARDWAREBUFFER`. When the input tensor was bound to an AHB before resize, MNN's Vulkan backend already imported the AHB as device memory in `onAcquire()`; the caller's buffer points at the AHB's CPU mapping (same physical memory), so the host→device upload is skipped entirely.

**`mnn_run_ahb_output`:** same guard added for the input path — `copyFromHostTensor` skipped when input tensor is AHB-backed.

### 4.3 Documentation — `docs/isp/ZERO_COPY_NCNN_COMPARISON.md`

New doc recording:
- How ncnn does AHB zero-copy (from the wiki URL).
- SoftISP current state and gaps.
- Side-by-side comparison table.
- The changes made and remaining work.

---

## 5. What remains for true end-to-end zero-copy

These are the remaining copy/stall points (ranked by impact):

| # | Gap | Location | Fix |
|---|-----|----------|-----|
| 1 | Camera de-strides RAW16 into `Vec<u16>` (full-frame copy) | `anw/src/camera.rs:copy_pixels()` | Expose `AHardwareBuffer*`/plane pointer + stride directly; set `ProcessParams::input_buffer`/`input_ahb` from it |
| 2 | Presentation does per-pixel CPU BGRA→RGBA blit | `anw/src/presentation.rs:present()` | Present from `AhbView` mapping directly, or queue AHB to compositor instead of `ANativeWindow_lock` + copy |
| 3 | Fences are CPU `AtomicBool` busy flags (no real sync fd ordering) | `anw/src/ahb_output.rs` / `mnnengine.rs` | Wire `cam-ipa::fence.rs` `SyncFence`/`AhbFenceState` into `AhbBinding`/`AhbOutputSlot`: camera acquire fd → Vulkan wait; GPU completion → display release fd |
| 4 | `mnn_wrapper.h` header claims wrapper-side fd/VkDeviceMemory import steps that the implementation doesn't do | `mnn_wrapper.h:346-349` | Update header comment to match reality (only `setDevicePtr`; fd diagnostic only), or implement wrapper-side fd import |

### Shared Vulkan context question

The user asked whether a shared Vulkan context needs to be held in the MNN glue layer. For SoftISP's current architecture:

- **MNN Vulkan backend creates its own device/context internally** when `mnn_session_create` is called with `MNN_BACKEND_VULKAN` (via `ensure_vulkan_loaded()` → `dlopen libMNN_Vulkan.so`).
- The AHB import path goes through `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)`, which MNN's `VulkanBackend::onAcquire()` handles internally via `VulkanBufferFromAHB::create()` (`VkImportAndroidHardwareBufferInfoANDROID`).
- **No shared Vulkan context is required from the Rust glue layer** — MNN's fork owns the Vulkan device and does the external-memory import itself. The Rust side only needs to:
  1. Allocate the AHB (`AHardwareBuffer_allocate` with `GPU_SAMPLED_IMAGE`/`GPU_DATA_BUFFER` usage).
  2. Pass the `AHardwareBuffer*` pointer via `AhbBinding.ptr`.
  3. Call `mnn_import_ahb_as_device_memory` before `resizeSession`.
- If the app wants to share its *own* VkDevice with MNN (e.g. for a custom pre/post-process shader reading the same AHB), that would require MNN's `sharedContext` mechanism (`BackendConfig` + `MNNVulkanSetSessionWorkgroup` etc.) — not currently wired, and not needed for the AHB zero-copy path itself.
