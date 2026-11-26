# SoftISP Frame Process Duration Analysis (on-device log)

Source: `./shim-debug.sh logs` → `.tmp/shim-debug/app.log` + `logcat`, 2026-11-26.
Device: 970×1296 capture, 1080×2264 window, fused ISP + MNN/Vulkan, dual-worker ping-pong.

## Extracted `[loop]` frame timings

| Frame | Time (ms) | Total | media_ndk_capture | stager | mnn_prepare | mnn_inference | mnn_output | present |
|-------|-----------|-------|-------------------|--------|-------------|---------------|------------|---------|
| 1 | 572646 | **531.7 ms** | 198.0 | 189.3 | 0.0 | 60.0 | 0.0 | 84.3 |
| 2 | 573555 | **1107.8 ms** | 146.3 | 132.8 | 0.0 | 60.0 | 0.0 | 768.7 |

`mnn_inference` is **60 ms** on both frames — the GPU inference itself is fast and is **not** the bottleneck.

## Key findings

### 1. Presentation is the dominant cost (up to 69% of frame)

Frame 2: `present 768.7 ms` out of `1107.8 ms` total (69%). Frame 1: `present 84.3 ms` (16%).

This is either:
- the CPU BGRA→RGBA blit in `WindowTarget::present()` (Path A fallback), or
- `ASurfaceTransaction_apply()` / `ANativeWindow_lock` blocking waiting for SurfaceFlinger.

Corroborated by repeated `AHB output slot generation 0 still busy after 1M yields; presenting may tear` warnings — the presenter isn't releasing the slot fast enough, so the next-frame worker spins and the pipeline stalls.

### 2. Output AHB binding is lost every frame → zero-copy output does NOT work

Every frame:
```
AHB output binding lost on 'DisplayBlock/frame'; falling back to host copy.
MNN: output tensor 'DisplayBlock/frame' not AHB-backed: flags=0 (want 14),
device=12970367412742730480 want=12970367413279764176,
ahb=0xb400007142d7aed0 ... rebound=0
```

- `device` (MNN's current tensor device memory) ≠ `want` (the AHB pointer cast to `uint64_t`).
- `device` changes every frame (`…30480`, `…36272`, `…8816`, …), meaning MNN's Vulkan backend allocates **new device memory for the output tensor every frame** and overwrites the `setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` binding set in `prepare()`.
- `rebound` fails because `device != 0 && device != ahb_id`.

Consequence: `mnn_run_ahb_output()` returns `-10` every frame and falls back to `mnn_run_true_zero_copy()` into the host pool. The GPU writes to host memory, then `present()` does a CPU blit from host to the window — this is the `present` cost.

**Root cause:** `prepare()` binds the output AHB only when `needs_resize` (geometry change). Since geometry is constant (970×1296), `needs_resize` is false after frame 1, so `mnn_import_ahb_as_device_memory` is not re-called. But MNN's Vulkan backend re-allocates output device memory on every `runSession`/`onAcquire`, clearing the AHB flag. The binding set before the *first* resize does not survive.

### 3. Camera capture + CPU de-stride are 132–198 ms each

- `media_ndk_capture`: 146–198 ms (`AImageReader_acquireLatestImage` + `copy_pixels()` de-stride into `Vec<u16>`).
- `stager`: 132–189 ms (`FrameStager::stage()` u16→LE byte widening).

Combined 279–387 ms (52–72% when present is fast). These are **CPU per-pixel passes** that could be eliminated if the camera `AHardwareBuffer*` were fed directly as `input_ahb` (input zero-copy). `frame.ahb` is already populated from `AImage_getHardwareBuffer` and `params.input_ahb` is set, but `mnn_prepare` is `0.0` because the prepared path is reused (`needs_resize=false`), so the input AHB import only happened on frame 1 — and it's unclear whether it survived MNN's per-frame re-allocation the same way the output binding doesn't.

### 4. `mnn_prepare` and `mnn_output` are 0.0

- `mnn_prepare 0.0`: prepared path reused (`needs_resize=false`). Good — no resize stall.
- `mnn_output 0.0`: on the zero-copy path `data_out = Vec::new()`, so no host copy is measured. But this is misleading — the output is NOT actually zero-copy to the AHB (see finding #2); it goes to the host pool and then `present` pays for it.

## Recommendations (ranked by expected impact)

| # | Fix | Phase attacked | Expected impact |
|---|-----|----------------|-----------------|
| 1 | **Make output AHB binding survive `runSession`**: `mnn_run_ahb_output` now force-rebinds the output AHB before every `runSession` when MNN's Vulkan backend cleared `MNN_MEMORY_AHARDWAREBUFFER` (it re-allocates output device memory each frame). Previously rebound only when `device == 0 || device == ahb_id`, which always failed because `device` held MNN's fresh allocation. | output → present | Should collapse `present` from 768 ms to <10 ms by eliminating the GPU→host copy and CPU blit. |
| 2 | **Fix presentation blocking**: `present_ahb_raw()` via `ASurfaceTransaction_apply()` is blocking for hundreds of ms, or the CPU blit in `WindowTarget::present()` is slow. Add the display release fd to the slot and wait for it before reusing; ensure `ASurfaceTransaction_apply` is non-blocking or fenced. | present | Removes the 69% frame cost. |
| 3 | **Skip `FrameStager::stage()` when `input_ahb` is set** (`anw/src/pipeline_runner.rs`): `stager.stage(&frame)` always runs its u16→LE loop even when `frame.ahb` feeds MNN directly. Guard on `frame.ahb.is_some()`. | stager | Saves 132–189 ms / frame. |
| 4 | **Input AHB zero-copy**: ensure `mnn_import_ahb_as_device_memory` is called before every `runSession` (not just on resize) so the input AHB binding survives like the output. | media_ndk_capture / stager | Eliminates the CPU de-stride copy entirely. |
| 5 | **Real sync fences**: replace the CPU `AtomicBool` busy flag with `cam-ipa::fence.rs` `SyncFence`/`AhbFenceState` (camera acquire fd → Vulkan wait; GPU completion → display release fd). Unblocks the slot-busy spinning. | present / capture | Removes `still busy after 1M yields` stalls. |

## What IS working (from the log)

- `mnn_inference` is a flat **60 ms** — GPU inference is fast and consistent.
- `mnn_prepare` is **0.0** — no resize stall after the first frame; geometry cache works.
- Camera capture succeeds at 970×1296, RAW16, sensor_max 1023, 3A (CCM) stats are computing (CCT ~5516–5520).

## Immediate next step

Fix finding #1 (output AHB binding persistence) — this is the largest single win and directly explains the `present` spike. The `mnn_run_ahb_output` rebound logic needs to allow re-binding over MNN's device allocation, or `prepare()` should re-bind the output AHB before every `runSession` when geometry is stable but the binding was lost.
