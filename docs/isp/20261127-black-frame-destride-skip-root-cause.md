# Black-frame root cause: input AHB de-stride skip (2026-11-27)

## Symptom

With `dev/integrated-20261127`, the shim APK produced frames at `600x800` with
`rgb mean [0.0, 0.0, 0.0]` and `non-black 0.0%`, even though capture was
non-black and MNN reported no AHB/upload errors.

## Root cause

Commit `1c6908f6` changed `anw/src/pipeline_runner.rs` to skip the CPU
de-stride / `u16`→LE widening whenever `frame.ahb.is_some()`:

```rust
let staged: &[u8] = if frame.ahb.is_some() { &[] } else { stager.stage(&frame) };
```

That removes the only host-side pixel copy. The MNN path cannot rely on the
AHB alone: the device log shows `AHB input import for 'raw_frame' failed:
rc=-2; using host upload` on the first frames after launch, and an input
tensor that needs geometry change causes MNN to clear the AHB binding and
fall back to its managed buffer. Both paths need the staged bytes. Feeding an
empty slice produced a zero-filled input and therefore a black output.

The other two suspects are not involved:

- `1b6ad529` (camera AHB binding) is present in `test1`, which renders
  correctly.
- `b6d98c44` (sensor black-level plumbing) is also present in the verified
  branch; `CcmGuard` stats were non-zero in the broken runs, so Blc was not
  zeroing the frame.

## Fix

`dev/revert-destride-only` restores the unconditional

```rust
let staged = stager.stage(&frame);
```

and keeps AHB input binding and black-level plumbing intact.

## Verification (shim APK cover function)

```bash
./build-test-libs.sh --core-only
./push-test-libs.sh
./shim-debug.sh reload
./shim-debug.sh logs
./shim-debug.sh verify
```

Result:

- `md5 verification: PASS (8 libraries)`.
- 95+ frames processed; representative stats `rgb mean [42.9,43.6,42.8]`,
  `max 255`, `non-black 25.0%` — no black frames.
- No `FATAL`, `SIGSEGV`, `SIGBUS`, or input upload failures after startup.

## Follow-up (separate issue)

The output tensor still logs `AHB output binding lost on 'DisplayBlock/frame';
falling back to host copy` every frame. That is an output-path performance
issue, not the black-frame cause, and remains tracked separately.
