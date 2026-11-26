# AHB output binding loss — debug findings (2026-11-26)

## Symptom

The device log repeatedly reports:

```
AHB output binding lost on 'DisplayBlock/frame'; falling back to host copy.
MNN: output tensor 'DisplayBlock/frame' not AHB-backed: flags=0 (want 14),
device=12970367452988255504 want=12970367450304674384,
ahb=0xb4000079e1b2fa50 bytes=0, out_elems=1200 max_out=33554432 need=4800,
rebound=0 | host pool: buffers=3 capacity=7680000 f32 (87.9 MB total),
next_idx=1 | AHB slot: 134217728B, generation 0
```

A second, related warning is:

```
AHB output slot generation N still busy after 1M yields; presenting may tear
```

The first warning means the MNN graph's output tensor is not backed by the
imported AHB device memory: the binding is lost and the engine falls back to a
host copy into the same AHB mapping. The second means the presenter handoff
did not release an output slot within the bounded spin, so the next frame may
be presented from a buffer still in use.

## Reproduction

1. User installs `target/debug/apk/softisp-test.apk` (shim-only launcher),
   pushes the core and MNN libs with `./push-test-libs.sh`, and enables All
   files access for `com.softisp.camera`.
2. Launch with `am start -n com.softisp.camera/com.softisp.camera.ShimActivity`.
3. Let the pipeline run for a few seconds.
4. Read `/sdcard/download/softisp-app.log` and count
   `AHB output binding lost` / `still busy after 1M yields` lines.

Observed on the current log: 120 binding-loss warnings and 10 busy-slot
warnings. Startup reports `sensor_orientation=90`, `pipeline rotate_mode=3`,
and `output AHB=on verbose=off`.

## Root cause

Under investigation. The prior fix attempt is recorded in
`docs/isp/20261126-output-copy-zero-copy-audit.md` (oversized AHB slot, real
`AHardwareBuffer` pointer check, no in-run resize, `-10` diagnostics). That fix
did not eliminate the warning on this device.

## Fix applied

Pending.

## Findings

- Quantitative log evidence (current `/sdcard/download/softisp-app.log`):
  120 binding-loss warnings, 10 busy-slot warnings, and a cumulative
  `3905 frames processed`. The warning is intermittent (roughly a few percent
  of frames), not a total failure of the AHB path.
- The reported output is tiny: `out_elems=1200`, `need=4800` bytes, matching
  the `600x2` last-frame geometry in the `[stats]` line. The slot is 128 MB and
  the host pool is 87.9 MB, so neither size is a limiting factor.
- Every warning has `generation 0`; there is no evidence that a resized slot
  or generation bump causes the loss.
- `device` and `want` are different numeric backend/device ids in every
  occurrence, so the imported AHB device memory appears to be associated with
  a different device than the one the output tensor expects.
- The MNN-side detail reports `flags=0 (want 14)`, `bytes=0`, `rebound=0`:
  the output tensor carries no device flags and zero device bytes at the
  moment of the check, despite the non-null AHB pointer. This is not a host
  pool exhaustion problem (3 buffers, 87.9 MB, `next_idx=1`) and not a slot
  sizing problem (128 MB slot for a `need=4800` byte output).
- Both warnings coexist, which suggests a lifecycle/ordering defect (binding
  installed or released at the wrong point relative to the presenter handoff)
  rather than two unrelated size problems.
- Code-path mapping and the exact trigger condition are being collected in
  this file before any source change is made.
