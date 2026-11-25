# P2h AHB fence plumbing findings

Date: 2026-11-26

## Symptom

The staged ISP path can prepare controller tensors before a frame arrives, but
the AHB handoff still has no explicit ordering object. `AHardwareBuffer` itself
does not carry a fence; the sync fd travels with the camera buffer slot or with
`AHardwareBuffer_lock`/`AHardwareBuffer_unlock`. The current wrappers discard
those fds: `AhwBuffer` stores only handle, dimensions, format and strides, and
`ProcessParams::{input_ahb, output_ahb}` store only `(ptr, len)`. The camera HAL
adapter also passes `-1` to `AHardwareBuffer_lock` and `null_mut()` to
`AHardwareBuffer_unlock`, so acquire waits and release fences are lost.

This prevents P2h from moving camera fill and metadata prefill into parallel
phases: without a generation-tagged fence the engine cannot prove that a
prepared tensor set matches the buffer it is about to read, and the display side
cannot prove that an output AHB is safe to recycle.

## Reproduction

1. Inspect `cam-rust/cam-ipa/src/buffer.rs`: `AhwBuffer` has no fence or
   generation fields.
2. Inspect `cam-rust/cam-isp/src/engine.rs`: the AHB fields are raw pointer/len
   tuples with no sync fd or generation.
3. Inspect `cam-rust/cam-hal-android/src/adapter.rs`: `AHardwareBuffer_lock`
   receives fence `-1`; `AHardwareBuffer_unlock` discards its output fence.
4. Inspect `MnnEngine::run_prepared()`: geometry is validated, but an AHB from a
   different capture generation would be accepted silently.

## Root cause

The staging API predates AHB ownership tracking. It models zero-copy buffers as
borrowed host pointers and assumes implicit driver ordering. That assumption is
invalid for camera-to-Vulkan and Vulkan-to-display handoffs, where both fences
and slot generation are part of the runtime contract.

## Fix applied

- Add non-owning, generation-tagged fence tokens to `cam-types` and make the
  AHB bindings in `ProcessParams` carry them.
- Add owning `SyncFence` and `AhbFenceState` helpers to `cam-ipa`; expose fence
  tokens and generation through the `Buffer` trait so context code can populate
  `ProcessParams` without owning raw fds. `begin()` refuses to recycle a slot
  whose release fence has not been taken by the display path.
- Use the existing Android `AHardwareBuffer_lock`/`AHardwareBuffer_unlock`
  declarations to pass an acquire fence into `lock` and capture the release
  fence from `unlock`. The fd passed to `lock` stays owned by the caller; the fd
  returned by `unlock` is owned by the fence state.
- Validate prepared/frame generation in `MnnEngine::run_prepared()`. A valid
  fence that cannot yet be consumed by the Vulkan backend is reported loudly,
  not silently treated as ready.
- Add an explicit `ZeroPolicy` flag. The host `MemBuffer` still zeroes both
  policies: skipping safely needs a producer callback that initialises every
  byte before the first read. A measured producer-driven clear skip remains a
  TODO.
- Add hermetic tests for fence ownership, generation mismatch and zeroing
  policy.

## Findings and limits

- Full P2h overlap remains blocked on P2b Vulkan external-memory/AHB import and
  on-device gralloc/BufferQueue validation. This change establishes the fence
  contract and validation points; it does not claim the camera fill or display
  handoff is live.
- The fence plumbing is Android/AHB-shaped. V4L2/dmabuf platforms need a separate
  generation-tagged `sync_file` path; this is recorded as a roadmap TODO rather
  than inferred from AHB wrapper behaviour.
- `AHardwareBuffer_lock` is a CPU-access API. Its acquire fd can seed a Vulkan
  wait, but the Vulkan backend still needs the P2b semaphore import before
  `run_prepared()` may treat the image as GPU-readable.
- Android `AHardwareBuffer_allocate` zeroing is allocator-dependent. The new
  policy does not yet skip any host clear: `MemBuffer` zeroes for both policies
  because skipping safely needs a producer callback that overwrites every byte.
  It only records the caller's full-overwrite intent for a future driver-side
  path; it does not assume the allocator returns zeroed memory.

## Verification

- `cargo fmt --all -- --check` and `git diff --check`: clean.
- Canonical per-crate clippy loop with `-D warnings`, including `softisp-anw`:
  clean.
- Hermetic tests: `cam-types` 1 passed, `cam-hal` 3 passed, `cam-core` 0 passed,
  `cam-onnx` 2 passed, `cam-motion` 6 passed, `cam-ipa` 10 passed,
  `cam-isp --features rectifier` 840 passed / 2 ignored, `softisp-anw` 49 passed.
- Real MNN suite `cam-isp --features "rectifier mnn"`: 875 passed / 2 ignored.
- On-device log check after the P2h APK (2026-11-26): 5434 lines across the
  current and rotated app logs; no fatal signal, Rust panic, tombstone, or app
  process kill. One recoverable `ACameraDevice: error 4` at 31.3 s was followed
  by `ACameraCaptureSession: ready` and continued frames through 31.9 s. Frame
  times stayed in the existing ~300–500 ms range. `HardwareBuffer.close`/release
  warnings came from SystemUI PID 26501, not the app process.
- `cargo test --workspace --doc --all-features` remains unavailable on
  Android/Termux because `--all-features` activates platform-specific crates;
  this is a local environment limitation, not a tree regression.
