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
output path reported `bytes=0`. Two defects were involved:

1. `ensure_ahb_import_loaded()` only tried `dlsym("AHardwareBuffer_describe")`
   on `libnativewindow.so`; the symbol is also exported by `libandroid.so`.
2. More importantly, the function-pointer typedef declared `describe` as
   returning `int`, while the NDK declares it as `void`. Calling a void function
   through that pointer produced a garbage nonzero status, so the capacity
   check failed even after the library lookup was fixed.

The fix resolves each AHB symbol from either library and uses the correct
`void` signature for `AHardwareBuffer_describe`.

With the descriptor fixed, the output import stops falling back, but the
renderer reads zeros from the CPU mapping: the Vulkan write into the AHB is not
visible to the permanently mapped CPU pointer. This is the coherency gap already
recorded in `docs/mnn/20261126-mnn-vulkan-ahb-handling-study.md`. The output
slot therefore needs a CPU read window opened **after** MNN's fence wait: unlock
the worker mapping, `AHardwareBuffer_lock(CPU_READ)` to invalidate, blit, then
re-lock for the next worker. No lock is added on the pure GPU path itself.

## Better implementation

Camera frames use the MediaNDK AHB **directly** when the plane is already the
tight RAW16 layout the model expects. `next_frame()` calls
`AHardwareBuffer_acquire()` before `AImage_delete`, so the pointer stays valid
through inference regardless of the reader's recycling. Padded RAW and YUV
frames keep the host path because their plane layout is not the model input.
The worker binds the acquired pointer to the frame input tensor. Since MNN's
Vulkan backend caches the imported AHB per tensor, every pointer change triggers
`setDevicePtr` plus `tryResizeSession(..., needMalloc=1)` so `onAcquire()`
re-imports the new buffer. The host `data` slice remains valid and non-empty as
a fallback if the import fails; it is never the sole source when an AHB is
present.

For inputs **not** supplied by MediaNDK (control tensors and host-generated
frames), a worker-owned AHB pool is still used. It is allocated once per
geometry and reused, never per frame:

- One `AHardwareBuffer` per worker, allocated as `BLOB` with
  `CPU_READ_OFTEN | CPU_WRITE_OFTEN | GPU_DATA_BUFFER` and mapped for the
  worker's lifetime. Capacity is `w*h*16` rounded to a power of two: MNN's
  Vulkan backend sizes int tensor storage at four bytes per element, and the
  extra margin covers padded/fused graph variants without swapping the binding.
- Non-camera pixels are de-strided into that mapping, which is bound before
  `resizeSession()` and never swapped while the geometry is unchanged.
- A geometry change drops and recreates the slot, invalidates the prepared
  frame, and re-binds before the next resize.
- If allocation or binding fails, the normal host staging path remains
  available, so a fallback can never produce an empty tensor.

## Correctness invariants

- A camera AHB is released only after inference; the pooled AHB is released only
  when the worker exits or the geometry changes.
- `PreparedMeta` records the exact bound input AHB pointer; `run_prepared()`
  rebinds and forces re-acquire when it changes.
- Generation still guards pooled-slot recreation; camera bindings use generation
  `0` and are distinguished by pointer.
- The host fallback bytes are always non-empty, so a rejected AHB import can
  never reproduce the black-frame failure.
- No `AHardwareBuffer_lock`/`unlock` is added on the pure GPU inference path;
  the only new CPU window is after MNN's fence wait for output presentation.

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
