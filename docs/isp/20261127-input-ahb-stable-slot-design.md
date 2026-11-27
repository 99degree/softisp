# Safe input AHB design after the de-stride-skip revert (2026-11-27)

## What the reverted change was trying to do

`1c6908f6` ("input AHB zero-copy + skip CPU de-stride when AHB present") made
`CameraSession::next_frame()` return `RawFrame { data: Vec::new(), ahb: Some(..) }`
whenever `AImage_getHardwareBuffer()` produced a pointer, and made the worker
set `staged = &[]`. The intent was to remove the per-frame `u16`→LE widening and
let MNN's Vulkan backend read the camera DMA buffer directly.

That idea is sound in isolation but the implementation assumed three things
that were not true:

1. `frame.ahb.is_some()` means MNN accepted the buffer. It does not: on the
   first frames the import returned `rc=-2` and the wrapper fell back to a host
   upload.
2. The `AImage` stays alive. It does not: `ImageGuard` deletes it at the end of
   `next_frame()`, so the borrowed `AHardwareBuffer*` is stale by the time
   inference runs.
3. A successful binding can be reused with a different AHB pointer. MNN's
   Vulkan fork caches the imported AHB in `onAcquire()` for a tensor/geometry;
   swapping the pointer under the same session geometry is ignored.

Because both the host `data` and `staged` slices were empty, every fallback
uploaded zeros and the preview went black. See
`docs/isp/20261127-black-frame-destride-skip-root-cause.md`.

A second, independent blocker showed up while validating the stable slot: the
wrapper's `ahb_capacity_bytes()` rejected every real AHB with `rc=-2`, and the
output path reported `bytes=0`. `ensure_ahb_import_loaded()` only tried
`dlsym("AHardwareBuffer_describe")` on `libnativewindow.so`, but that symbol is
provided by `libandroid.so`. The resulting null function pointer made the
capacity check fail before `setDevicePtr` was reached, so neither the input nor
the output AHB could be imported. The fix resolves both AHB symbols from either
library, with `libandroid.so` as the fallback.

## Better implementation

Use a **stable, worker-owned input AHB slot** instead of binding the camera's
transient `AImage` buffer directly:

- One `AHardwareBuffer` per worker, allocated as `BLOB` with
  `CPU_READ_OFTEN | CPU_WRITE_OFTEN | GPU_DATA_BUFFER` and mapped for the
  worker's lifetime. Capacity is `w*h*16` rounded to a power of two: MNN's
  Vulkan backend sizes int tensor storage at four bytes per element, and the
  extra margin covers padded/fused graph variants without swapping the binding.
- Camera pixels are de-strided directly into that mapping, so there is no
  second `Vec<u8>` staging buffer and no host→device tensor upload.
- The slot is bound to the frame input tensor before `resizeSession()` and is
  never swapped while the geometry is unchanged. MNN's cached Vulkan import
  therefore always points at the buffer the CPU just filled.
- A geometry change drops and recreates the slot, invalidates the prepared
  frame, and re-binds before the next resize.
- If allocation or binding fails, the normal host staging path remains
  available, so a fallback can never produce an empty tensor.

The camera still uses MediaNDK `AImageReader`/`AImage`. Its reader-owned AHB is
deliberately not exposed or bound: `AImage_delete` recycles it immediately after
`next_frame()`, and a stale pointer is what made the reverted change read zeros.

## Correctness invariants

- `ProcessParams.buf` and `ProcessParams.input_ahb` refer to the same mapping
  for a given frame.
- `PrepareParams::from(&params)` includes the input AHB generation, so a stale
  prepared frame is rejected after a slot recreation.
- The slot stays mapped and alive until the worker exits; `Drop` unlocks and
  releases it.
- No `AHardwareBuffer_lock`/`unlock` is added to the pure GPU output path.

## Verification

```bash
cargo fmt --all -- --check
cargo check -p softisp-anw --all-features
cargo test -p softisp-anw --lib --all-features
cargo clippy -p softisp-anw --all-features --no-deps -- -D warnings
./build-test-libs.sh --core-only
./push-test-libs.sh
./shim-debug.sh reload
./shim-debug.sh logs
./shim-debug.sh verify
```

Device success criteria: non-zero `rgb mean`, `non-black > 0%`, and no
`AHB input import ... failed`, `MNN inference failed`, `FATAL`, `SIGSEGV`, or
`SIGBUS` lines after launch.
