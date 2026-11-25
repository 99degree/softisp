# Output readback copy vs AHB zero-copy (2026-11-26)

## Symptom

The user rejects "reuse the `Vec` for output readback" as an optimization:
`buf_bytes.to_vec()` copies the whole display tensor (~33 MB at 4K) every
frame. With an AHB-backed output tensor the copy and the u8 cast should not be
needed at all. If a large host copy still appears in the trace, either the
zero-copy path is not applied to the real app frame, or it is implemented
incorrectly.

The user later tightened the requirement: **AHB support on the output tensor is
a must**, and because MNN may not expose an API that makes an arbitrary output
tensor AHB-compatible, the accepted workaround is to allocate the output AHB
large once, validate it when the output size is known, and reuse it.

## Reproduction / evidence to collect

- App logs report `raw_zero_copy` on the **input** path, but the visible frame
  timing still includes `output_copy`; verify which branch `run_prepared()`
  takes for the real camera frames.
- Identify what `ProcessParams` carries for output AHB / `output_buffer` and
  whether the Android app (`anw/`) ever provides one.
- Trace `meta.zero_copy` from `prepare()` into the output branch of
  `run_prepared()` and into `Frame::external_data` / presentation.

## Root cause

The app never set `ProcessParams::output_buffer` or `output_ahb`, so
`prepare()` always took the internal `buf_pool` branch and `run_prepared()`
always copied the pool buffer into a fresh `Vec` (`buf_bytes.to_vec()`) before
building `IspFrame`. Even when a caller did set `output_buffer`, `IspFrame::new()`
allocated a second `width * height * 4` byte `Vec` and then discarded it in the
zero-copy branch.

On the C++ side, `mnn_import_ahb_as_device_memory()` could bind an AHB to an
output tensor, but `mnn_run_true_zero_copy()` still finished with
`copyToHostTensor(host_out)`. That readback is exactly the device→host copy the
user wants removed. No caller in `anw/` supplied an output AHB anyway, so the
whole AHB output path was dead code in the shipped app.

## MNN capability study

`TensorUtils::getSharedMem()` / `setSharedMem()` live in MNN's internal
`source/core/TensorUtils.hpp`, not in `include/MNN`. `getDeviceInfo()` returns a
`VkBuffer`, not the `AHardwareBuffer*`, and `Tensor::map()` falls back to
`malloc()` + `onCopyBuffer()` because Vulkan does not override `onMapTensor` —
that is a real copy. MNN has no public API to allocate an AHB or export one
from a tensor. The fork's `VulkanAhbMemObj` is file-local to
`VulkanBackend.cpp`, so a wrapper compiled against public headers cannot safely
downcast `Backend::MemObj`.

Conclusion: the **caller must allocate, lock, bind and read the AHB**. The
wrapper binds it with `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)`
before `resizeSession`, so the fork's `VulkanBackend::onAcquire()` imports it;
`Tensor::wait(MAP_TENSOR_READ, true)` waits MNN's internal Vulkan fence. It is
not an AHB acquire/release fence or cache invalidation, so CPU coherency still
needs device validation. No new MNN-side API is required.

## Repo-wide AHB audit

The user believed ready-made AHB handling exists in `cam-ipa/src/buffer.rs` and
`cam-hal-android/src/gralloc.rs`. A full-repo audit found:

- `cam-ipa::AhwBuffer` (`cam-rust/cam-ipa/src/buffer.rs:105-193`) is a
  **non-owning descriptor** around a caller-supplied pointer. It does not
  allocate, lock or release an AHB; `as_ptr()` deliberately returns null
  because the buffer is meant for Vulkan import, not CPU mapping. It cannot
  feed the output slot.
- `cam-hal-android::AHardwareBufferInterop::allocate`
  (`cam-rust/cam-hal-android/src/gralloc.rs:104-119`) does allocate a real AHB,
  but `AHardwareBufferBacked` has no `ahb_ptr()` accessor and no `Drop`, and
  its `map_with_fence`/`unmap_with_fence` mapping is per-call, not persistent.
  Linking `cam-hal-android` into `anw` would also pull non-NDK HAL libraries
  (`-lgralloc`, `-lcamera3`, `-lhidl`, `-lbase`) into the APK.
- `HardwareBufferPool` and `cam-binder::HardwareBufferBridge` are accounting
  stubs (IDs and null host pointers), not real buffers.

Therefore no existing helper replaces the producer-side slot. Keep
`anw/src/ahb_output.rs` as the AHB owner and reuse the already-landed consumer
pieces: `cam_types::{AhbBinding, AhbFenceTokens}`, the `mnn_sys` import/run FFI,
and `IspFrame::new_external()`.

## Device evidence: binding loss, not allocation failure

The default-on app log records `output AHB=on verbose=off`, the fused pipeline
at `1600x1200`, raw `800x600`, and a repeated warning naming
`DisplayBlock/frame`. It contains no `AHardwareBuffer_allocate` or
`AHardwareBuffer_lock` failure, no input AHB import failure, and no
`AHB output import ... failed`. The recurring error is therefore loss of the
output tensor's AHB-backed binding, not a shortage of AHB slots or a bad input
buffer.

MNN fits each Vulkan `VkBuffer` to the tensor's aligned element count and
re-acquires buffers on resize; an AHB cannot be converted from an already
allocated tensor. A memfd/host pointer can be adopted after allocation, but an
AHB must exist before `Tensor::setDevicePtr(..., MNN_MEMORY_AHARDWAREBUFFER)` and
before session resize. The binding check now reports the tensor flags, device
pointer, expected AHB pointer and byte capacity, real input/output element
counts, and the host `OutputBufferPool` snapshot in the `-10` fallback warning.
The host pool is fixed at three buffers and can grow per-buffer capacity; the
AHB path has one slot per worker, so a `-10` does not mean a pool exhaustion.

## Fix

1. `IspFrame::new_external()` builds an allocation-free frame whose pixels are
   the caller-owned AHB mapping; `run_prepared()` uses it whenever
   `meta.zero_copy` is set and validates both the pointer and byte capacity
   captured by `prepare()`.
2. `anw` enables the output AHB path by default and keeps a reversible opt-out
   (`SOFTISP_OUTPUT_AHB=0` / `off` / `false` / `no`, `SOFTISP_OUTPUT_AHB_OFF`, or
   `/sdcard/download/softisp-output-ahb-off`; ignored below API 26). Each worker
   owns one large `AHardwareBuffer`, mapped once and registered with MNN as
   `MNN_MEMORY_AHARDWAREBUFFER` before `runSession`. The slot is never swapped:
   MNN imports it in `onAcquire`, so rebinding would leave the imported
   `VkBuffer` pointing at the old buffer. Capacity is
   `16 × width × height × 4`, rounded up to a power of two, which covers the
   fused graph's worst-case output without a resize.
3. A new FFI entry point, `mnn_run_ahb_output()`, runs the session, waits on the
   output tensor with `Tensor::wait(MAP_TENSOR_READ, true)`, validates the
   output element count against the slot, and returns **without**
   `copyToHostTensor`. It verifies that the pointer passed back from Rust is a
   real `AHardwareBuffer` and large enough for the tensor before running,
   refuses an in-run resize that would re-acquire the output binding, and only
   restores a lost binding when MNN cleared the flags but left the caller's AHB
   pointer in place. The existing `mnn_run_true_zero_copy()` remains for
   non-AHB callers and as a fallback for any `-10` result.
4. The slot travels with `Done` as a cloneable view. The worker waits only for
   its **own** presenter handoff before reusing the slot — never for the other
   engine — so the two engines stay asynchronous. The presenter releases the
   view immediately after its blit; repaint skips a view whose worker has
   marked it busy.
5. Verbose diagnostics are masked in normal runs by lowering the app logger's
   max level to `Info`. `SOFTISP_VERBOSE` / `/sdcard/download/softisp-verbose`
   restores `Debug` and enables the new per-frame slot/import/wait logs. The
   switch state itself is always written once at `INFO`.

## Verification

- `cargo fmt --all -- --check`: clean.
- Canonical per-crate clippy loop with `-D warnings`: clean.
- `cargo clippy -p softisp-anw --all-features --no-deps -- -D warnings`: clean.
- `cargo test --lib`: 857 passed / 2 ignored (`rectifier`), 895 passed /
  2 ignored (`rectifier mnn`). `softisp-anw`: 51 passed / 1 ignored.
- `./build-apk.sh`: APK built and signed with `libsoftisp.so` containing the
  AHB output path.

## Decision so far

Do not mask the copy with a reusable `Vec`; either eliminate it through an
AHB-backed output tensor or explain precisely why the fallback is required.
Keep the two engines asynchronous: no new cross-engine wait or shared lock on
the inference path. The output AHB is a caller-owned, large, reusable slot; the
session output is bound to it once for the session, and presentation reads the
mapped bytes only after MNN's tensor wait has synchronised the GPU write.
On-device validation of the default-on path is still required; until then the
host-copy fallback stays in place for failed imports and opt-out runs.
