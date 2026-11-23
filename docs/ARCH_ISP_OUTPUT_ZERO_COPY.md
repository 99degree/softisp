# SoftISP: ISP Output Tensor → Android Display — Zero-Copy Implementation

## Summary

**Done.** The pipeline now supports end-to-end zero-copy from MNN/Vulkan inference output directly into a caller-supplied buffer (e.g. Android `AHardwareBuffer`, memfd, or CMA), eliminating the per-frame heap allocation + `copyToHostTensor` + `f32→u8` narrow + `memcpy` into the display buffer. The `isp.display` fused op (already present in `FUSED`/`UNIFIED`/`WHOLE` profiles) is the consumer of this path.

---

## What changed

### 1. `ProcessParams::output_buffer` — cam-isp/src/engine.rs

Added an optional caller-owned output buffer field:

```rust
pub output_buffer: Option<(*mut u8, usize)>,
```

The pointer is a raw `*mut u8` (not `&mut [u8]`) so `ProcessParams` does **not** need `Clone` — the engine takes a shared reference to `ProcessParams` during `process()`, and reborrowing a `&mut [u8]` out of a shared reference would violate aliasing rules. A raw pointer sidesteps this entirely; the caller (HAL) owns the buffer lifetime.

### 2. `IspFrame::external_data` — cam-isp/src/pipeline/types.rs

Added an `ExternalBuffer` descriptor (ptr + len) that the engine returns when zero-copy was used. `data` is left empty; the consumer reads from `external_data.ptr` instead. `ExternalBuffer` is explicitly `unsafe impl Send + Sync` because the caller guarantees the buffer outlives any frame referencing it.

### 3. `MnnEngine::process()` — cam-isp/src/mnnengine.rs

- Allocates no temporary `Vec<u8>` when `output_buffer` is set.
- Calls `mnn_run_true_zero_copy()` (already implemented in `mnn_sys/mnn_wrapper.cpp:559`) which assigns `in_tensor->buffer().host = caller_buffer` and `output->buffer().host = caller_out_ptr` directly — MNN reads/writes in place, no host→device or device→host copy.
- Skips `out_bytes.set_len()` and `f32_to_u8()` entirely for zero-copy frames (avoids UB on an empty Vec and the per-pixel clamp/cast pass).
- Returns `IspFrame { external_data: Some(ExternalBuffer { ptr, len }), data: Vec::new() }`.

### 4. `CameraIspService::process_raw_frame_zc()` — cam-isp/src/integration.rs

Hands the output `AHardwareBuffer` pointer directly to `ProcessParams::output_buffer` before calling `engine.process()`. If MNN wrote into it, no copy happens — the HAL output buffer already contains the final pixels. Falls back to the heap-allocated path when zero-copy is not requested.

### 5. `PipelineResult::external_data` — cam-isp/src/manager.rs

Propagates `external_data` from `IspFrame` up to `PipelineResult` so the HAL/consumer can display directly from external memory.

---

## Remaining copy points (where zero-copy does NOT yet apply)

| Hop | Status | Why |
|-----|--------|-----|
| ISP output tensor → external buffer | ✅ Zero-copy via `mnn_run_true_zero_copy` | MNN writes directly into caller buffer |
| External buffer → Android `AHardwareBuffer` | ✅ Zero-copy | Same buffer; HAL writes `output.ptr` directly |
| `f32 [0,255]` → `u8` narrow | ✅ Eliminated | `isp.display` (or Conv) output is consumed as-is |
| Per-frame heap `Vec<u8>` allocation | ✅ Eliminated | No temporary buffer allocated |
| Camera input `AHardwareBuffer` → engine input | ❌ Still a copy | `lock_and_copy_buffer()` reads AHB into `Vec<u8>`; MNN then uploads to GPU. Zero-copy input would require MNN Vulkan to read `AHardwareBuffer` memory directly — not yet wired. |
| Stats readback (zone stats, channel means, etc.) | ❌ Still `copyToHostTensor` | Separate from the frame output path; intentional per-frame GPU stall trade-off. |

---

## How to use it (Android HAL path)

```rust
// 1. Allocate the ISP output buffer as an AHardwareBuffer (or memfd/CMA)
let output = zero_copy_mgr.allocate_isp_output("disp_out", w, h, 4)?;

// 2. Lock it for CPU write (mmap'd pointer)
let (ptr, size) = AndroidCameraAdapter::lock_for_write(output.buffer)?;

// 3. Process with zero-copy
let mut result = mgr.process_frame_zc(input, output)?;

// 4. result.external_data points at output.ptr — pixels are already there.
//    Pass result.external_data to SurfaceFlinger / presentor directly.
//    No memcpy, no f32→u8 pass.
```

---

## Files changed

- `cam-rust/cam-isp/src/engine.rs` — `ProcessParams::output_buffer` field
- `cam-rust/cam-isp/src/pipeline/types.rs` — `IspFrame::external_data` + `ExternalBuffer`
- `cam-rust/cam-isp/src/mnnengine.rs` — zero-copy inference path, skip `set_len`/`f32_to_u8`
- `cam-rust/cam-isp/src/integration.rs` — `process_raw_frame_zc` passes output buffer to engine
- `cam-rust/cam-isp/src/manager.rs` — `PipelineResult::external_data`
- `cam-rust/cam-isp/src/cpu.rs`, `unified_pipeline.rs`, `postprocess.rs`, `isp_controller.rs`, `neural_controller.rs`, `hdr.rs`, `controller_api.rs` — `external_data: None` in test/frame initializers
