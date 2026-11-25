---
title: "Perf study: pre-inference / inference / post-inference phases"
created_at: 2026-11-25T00:00:00+00:00
question: "Which code path dominates preview frame time after dedup and dual-engine pipelining?"
status: done (follow-up TODO recorded below)
---

# Perf study: pre-inference / inference / post-inference phases

## Symptom / question

The app currently reports aggregate stage timings from `IspFrame`
(`prep_duration_ns`, `inference_duration_ns`, `total_duration_ns`), but the
pre-inference number is known to include many heterogeneous operations and the
post-inference readback/controller work has no aggregate field. Device logs
show `pipeline stage=infer_done ... infer=~65 ms` and `prep=~215 ms` at
1600x1200 RAW -> 800x600 packed output, but that does not isolate:

- pool acquire and shape/state setup,
- per-tensor upload / dedup flush,
- final input resize / zero-copy import,
- MNN graph execution,
- MNN session-info/profiling best-effort call,
- output metadata resolution and frame-byte extraction,
- black-frame diagnostics and 3A stats readback.

## Plan

1. Add debug-level timers inside `MnnEngine::process()` for each phase and keep
   `IspFrame` fields compatible (prep = enter->MNN call including pool acquire;
   infer = MNN call only; total = enter->frame ready including post).
2. **Done:** extend the existing ignored perf probe to report aggregated prep /
   infer / post / total, so a single run can quantify phase costs without parsing
   logs.
3. **Done:** ran on device with a fresh APK and parsed the new `pipeline timer`
   debug lines at production preview size.
4. **Done:** recorded the measured numbers, identified the dominant code piece,
   applied the concrete patch, and verified it.

## Findings

- **Phase aggregation is implemented.** `probe_4k_perf` now reports mean prep,
  infer and post plus total p50/p95/max for both the single-engine and dual-engine
  runs. `post` is computed as `total - prep - infer`, so it contains output
  metadata, packed-byte extraction, profiling and stats/controller work without
  adding another `IspFrame` field.
- **The ignored MNN probes can now link on Android.** `cam-isp/build.rs` emits
  `-landroid` (and the other NDK system libraries) for every Android build that
  enables `mnn`/`ort`, instead of only when a local NDK checkout is found. The
  probes call the `ANativeWindow` C ABI and previously failed with undefined
  `ANativeWindow_acquire`/`release`.
- **Probe sanity check fixed.** Packed BGRA words were being interpreted as four
  `f32` bytes, so the probe reported every packed frame as blank. It now checks
  for any non-zero output byte, which is format-independent.
- **Packed-output warning fixed.** The engine compared a one-channel fused
  `isp.display_*` tensor against `channel_count() == 4` for `Bgra`/`Argb` and
  logged a false mismatch. It now expects one channel only when the graph shape
  really is one packed word; normal four-channel u8 graphs still expect four.
- **Local smoke measurement (160x120, dual fused, 2 measured frames):** prep
  ~10–13 ms, infer ~54–98 ms, post ~1–2 ms, total p50 ~68–115 ms. Inference
  dominates even at this tiny size, but these numbers include session warmup
  variance and are not a device production measurement. A fresh APK is required
  to parse the new `pipeline timer` debug lines on the real preview path.
- **Device measurement (FUSED, RAW 1600x1200 -> packed 800x600, 181 timed
  frames):** mean total 314.0 ms (p50 305.5, p95 419.0, max 550.5). The mean
  phase split is `setup 47.6`, `tensor_assign 10.3`, `input 46.9`, `infer 112.2`,
  `output_meta 17.2`, `output_copy 0.8`, `stats 79.3` ms. Inference is the
  largest single phase, but GPU->CPU statistics readback (`stats`) is a close
  second and happens on **every** frame because `stats_interval` defaults to 1
  and both engine workers leave it enabled.

## Next optimization: stop paying the stats stall twice per frame pair

**Symptom.** The dual-worker app creates two `MnnEngine`s with independent
controllers and leaves both with `stats_enabled = true`. Workers alternate
frames, so both pay the ~79 ms host readback stall on every frame they process;
the stall is serialized against inference and shows up directly in preview
latency.

**Root cause.** The dual-engine probe (`probe_4k_perf::run_dual`) already uses the
right configuration — engine A reads stats, engine B skips them — but the
production app wiring in `android-app/src/lib.rs` was never updated to match.
Independent controllers also mean engine B would never see AWB/CCM/tone updates
made by engine A, which is harmless only while both engines read stats
themselves. Once B skips stats it must share A's controller.

**Correction to the earlier plan.** Phasing both engines with `interval=2` and
opposite countdowns does **not** preserve one measurement per captured frame:
the workers already see alternate frames, so each engine measuring every other
one of its own frames would measure only half of all frames in total, with gaps.
The existing A-on/B-off split is the correct shape. It measures one of every two
captured frames, and the 3A controller already smooths those numbers.

**Fix applied.** `MnnEngine::controller` is now `Arc<Mutex<IspController>>`;
`share_controller(&Arc<_>)` and `controller_handle()` let two engines hold the
same 3A state while `IspEngine::controller()` still returns `&Mutex<_>`.
`android-app/src/lib.rs` creates one controller, gives it to both engines,
leaves engine A stats **on**, and calls `set_stats_enabled(false)` on engine B
before the workers start. `DualGpuEngine` no longer carries a second unused
controller: its trait method delegates to engine A. Unit tests cover shared
handles and the stats toggle; `mnn_buffer`'s test no longer uses a no-op
`drop(&mut slice)` that warned under the test build.

**Verification.** `cargo fmt --all -- --check`, `cargo clippy -p cam-isp --lib
--features "rectifier mnn" -- -D warnings`, `cargo clippy -p cam-camera-app
--all-features -- -D warnings` and `cargo test -p cam-camera-app --lib
--all-features` (46 passed) are clean. The new MNN unit tests pass (11 passed).
A 160x120 `perf_4k_dual` smoke run passes with engine A stats on / B off.
The full `cam-isp --lib` result is unchanged: 848 passed, 2 pre-existing
failures (`test_unified_force_input16`, `test_manager_build_and_process`), 2
ignored.

The ~95 ms in `setup` + `input` contains no allocation on the zero-copy path;
it is likely scheduler/log-write latency from per-frame debug records. It needs
a finer timer around those calls before it is changed.

## TODO (user idea): split the frame into prep → infer → post phases

Two related reorderings were suggested while reviewing the phase timings:

1. **Present before stats readback.** `MnnEngine::process()` currently returns
   only after the 3A tensor readback and controller update, so `PipelineRunner`
   cannot put finished pixels on the `ANativeWindow` until that GPU stall has
   completed. Send the output frame immediately after inference, then do the
   metadata/stats readback on the worker.
2. **Prepare before the frame arrives.** The camera layer is what blocks while
   the sensor buffer fills. The controller/manager already hold the previous
   frame's tuned CCM/AWB/tone/exposure values, which are exactly the values the
   next frame would consume. Write those runtime tensors and acquire/resize a
   session *before* calling `CameraSession::next_frame()`; once the frame is
   returned, only the input buffer pointer is newly known, so the worker can
   feed it and enter `runSession` immediately.

Both are directly related to this perf work. Together they turn the current
single `process()` call into three explicit phases:

1. `prepare()` — acquire/reserve a session, apply geometry, write controller
   metadata from the previous frame; runs while the camera is busy.
2. `run_prepared()` — consume the arrived input buffer, run inference, extract
   output bytes, return the frame immediately.
3. `finish()` — read stats tensors and update the controller after the frame
   has been handed to the presenting thread; runs while display/next-worker
   inference proceeds.

**Plan estimate (not implemented here).** This needs a staged-engine API, not
a tweak to the stats interval:

- Add opt-in "deferred stats" and "pre-arrival prep" switches on `MnnEngine`
  (default off, so existing probes that read `IspAuxOutput::calibration_stats`
  and callers that expect a single `process()` keep their semantics).
- Carry an opaque session/token between phases. The worker must consume it in
  the same thread before the next `process()`, so a session is never reused
  while a prep or readback is in flight. `PipelineRunner` already pins one
  engine per worker, so this fits without a new pool design.
- Borrowed MNN wrapper handles are required so `finish()` does not destroy the
  engine-owned interpreter/session; this is the main safety detail to review.
- First-frame and geometry-change cases fall back to the current inline path:
  no prior controller state or no known target geometry means no safe prep.
- The `camera` mutex should be held only for `next_frame()`, not across prep;
  worker B can prep while worker A waits for its sensor buffer.
- **Buffer ownership (implementation-stage refinement).** The strongest form of
  prep is to allocate/initialize the inference input buffer first and hand that
  same pointer/AHB to camera2 as the capture target. The camera then fills the
  buffer while prep and the previous frame's work happen; when it signals
  ready, the engine only binds the already-known pointer and enters inference.
  That makes the critical path "buffer ready → `runSession`", not "frame arrives
  → copy/prepare → `runSession`". The exact AHB/ImageReader/GraphicBufferProducer
  ownership, usage bits, fence/sync, and lifetime rules must be settled at
  implementation time; this idea must not weaken the zero-copy contract or
  reuse a buffer while the GPU still reads it. It is dependent on the P2b AHB
  work (camera reader currently lacks GPU usage bits, `AImage` is deleted before
  inference, and no output AHB is obtainable from `ANativeWindow`).

**Estimated payoff.** Device means were `setup 47.6 ms`, `tensor_assign 10.3
ms`, `input 46.9 ms`, `infer 112.2 ms`, `stats 79.3 ms`, total `314.0 ms`.
Overlapping prep with camera wait and stats with presentation can hide roughly
`58 ms` of pre-inference tensor work and `79 ms` of readback from the visible
frame path. With camera2 filling the pre-bound input buffer, the input wait and
copy leave the inference path too; `runSession` itself remains the hard floor.
Throughput is still bounded by MNN inference and camera cadence, so this is a
latency/responsiveness win, not a way to make the GPU faster. The design goal
is explicit: keep only genuinely critical work between "buffer ready" and
"frame displayed", and move everything else into the camera-fill, presentation,
or next-worker window.

**Impact on the current plan:** none for this commit. The A-on/B-off split is
still the correct first fix and removes one of the two readbacks per frame
pair. The staged prep/infer/post refactor is a separate P2 follow-up with a
larger API surface, so it is recorded here rather than folded into the
controller-sharing change.
- **Fresh APK log hygiene:** 0 `default for ... has no tensor`, 0
  `output channel mismatch`, 0 `present failed`, 0 `black frame`. The preview
  was non-black 100% of sampled frames at 800x600 with mean RGB around
  `[155..169, 147..165, 153..160]`. One `ACameraDevice: error 4` callback was
  observed and the recovery path reopened capture; the session reached `ready`
  again. This confirms the earlier warning fixes did not regress output.

## User correction: recurring diagnostics are functional, not noise

The following log lines are **not** harmless and must not be dismissed:

- `read_output_floats(CoarseHistogramBlock/frame) -> -3` — the FUSED profile
  has `use_histogram: false`, so AE clipping never receives a histogram from
  the graph. Enable the histogram in the production FUSED profile and verify
  `ctrl.update_histogram()` runs.
- `BayerWbBlock/gains: no such tensor` / `CcmBlock_ccm/matrix: no such tensor`
  — these are not failures in the fused path, but the default-input cache still
  queues writes for primitives the converter removed. The functional values
  must be folded into the surviving sink (`DemosaicCcmBlock/w` for the fused
  demosaic+CCM, `DemosaicBlock/w` when WB is folded without a fused CCM),
  and missing-sink writes must be filtered once at build time instead of
  warning every frame.
- `ACameraDevice: error 4` (`ERROR_CAMERA_DEVICE`) — fatal camera-device
  callback. The callback currently only logs; the app must close and reopen
  the session on the main loop, otherwise capture stays dead.

These are part of the full functional pipeline / 3A interaction and are being
fixed as part of this perf study, not silenced.

## BayerWbBlock "CCM metadata" check

The suggested path does **not** exist in the current graph as a literal tensor:

- `BayerWbBlock::output_tensors()` returns only `BayerWbBlock/frame`.
- `BayerWbBlock::extra_inputs()` declares `BayerWbBlock/gains`, `/zero`, `/one`.
  `BayerWbBlock/wb_applied` is an internal node, not a graph output.
- The controller does not read a WB-block output. It reads the pre-WB stats tap
  `aux_hook_src/out` (placed after BLC, before LSC/WB/demosaic), so its channel
  means describe the scene rather than its own WB correction. Reading a
  post-WB output would make AWB chase its own feedback.

The intended *functional* path is present, though:

1. `ChannelMeansBlock/frame` (4 CFA planes on the fused chain) is folded through
   `bayer_planes::planes_to_rgb()` before reaching `IspController`.
2. `IspController::update_channel_stats()` updates AWB gains, estimates CCT,
   selects/smooths/sanitizes the CCM, and publishes it via `get_ccm()`.
3. `MnnEngine::process()` snapshots `get_awb_gains()` / `get_ccm()` **before**
   inference and after the previous frame's stats update. The app passes no
   explicit `ccm_matrix` / `bayer_gains`, so these controller values are the
   source for the next frame.
4. The engine maps `[R,G,B]` gains to CFA plane order with
   `bayer_planes::gains_by_plane(bayer_pattern)` and, when `BayerWbBlock/gains`
   is absent from the converted session, folds them plus the controller CCM into
   the surviving `DemosaicCcmBlock/w` (or `DemosaicBlock/w`, or as a last resort
   `CcmBlock_ccm/matrix`). That fused 3x4 tensor is the "correspondent CCM
   tensor value" consumed by the next inference.

So: no WB metadata output tensor, but the controller already owns the WB/CCM
decision and emits the next frame's CCM-equivalent tensor through the fused
sink. The fix in this work makes the sink choice come from the converted
session rather than the profile declaration, so primitive and fused variants
both receive the controller decision exactly once.

A permanent explanation of the block, its plane ordering, its identity-graph
special case, and its fused-sink algebra is now in
`docs/isp/BAYER_WB_DESIGN.md`.

## Heavy-profile MNN test case is broken by the zero-copy `ProcessParams` fields

**Symptom.** `cargo test -p cam-isp --test test_mnn_engine HEAVY --features mnn`
fails to compile before any engine runs:

```
error[E0063]: missing fields `input_ahb`, `input_buffer`, `output_ahb` and 1
other field in initializer of `ProcessParams<'_>`
```

There are ten such initializers in `cam-rust/cam-isp/tests/test_mnn_engine.rs`,
including `run_mnn_profile_test()`, which is the helper used by
`test_mnn_all_profiles` for the HEAVY arm.

**Root cause.** The zero-copy surface added `input_buffer`, `output_buffer`,
`input_ahb` and `output_ahb` to `ProcessParams`. Every caller that still uses a
full struct literal must now name those fields. `ProcessParams::new()` already
sets them to `None`, so the fix is to keep each test's explicit fields and fill
the rest with struct-update syntax (`..ProcessParams::new(...)`) instead of
adding four `None` lines ten times.

**Fix applied.** All ten initializers now use
`..ProcessParams::new(...)`, so future fields get the safe default. Three more
stale assumptions in the same test file were uncovered while making HEAVY run:

1. `build_blocks(width, 0, 0)` now asserts `concrete_h > 0`; the helpers passed
   a zero height. They pass the real target height.
2. Pre-converted `.mnn` tests passed a dummy `RawInputBlock` to `build()`, but
   that block's `graph_input_name()` is what the engine uses to find the frame
   input. Primitive profiles start at `RawInputPackedBlock`, so inference failed
   with `frame input 'RawInputBlock/frame' not found in session`. They now pass
   the profile's real head via `profile_head()`.
3. Size assertions assumed `64*H*3` bytes and a full-width output. The packed
   head halves the width and the graph emits RGBA, so they now derive the
   expected size from `IspFrame::width/height/format`.

**Verification.** `test_mnn_all_profiles --ignored` gets through LITE, MED and
HEAVY; HEAVY converts 91 MNN ops, runs, and returns a valid `12×32 RGBA` frame.
PRO still fails conversion (`isp-domain model=0 isp Extra ops emitted=0`), which
is a separate unsupported-op issue. A standalone ignored
`test_mnn_heavy_profile` was added so HEAVY can be checked without the PRO arm.

**Perf-test layout.** `probe_4k_perf` is split into one ignored test per profile
(`perf_4k_fused`, `perf_4k_med`, `perf_4k_heavy`, `perf_4k_whole`,
`perf_4k_unified`), a `perf_4k_dual` test, and an explicit `perf_4k_all` matrix.
`perf_4k` remains as the FUSED-only breakage check so a routine verification
does not run every profile.

## Fused WB is intentional, not a missing stage

The fused `DemosaicCcmBlock` already has runtime extra inputs `/w` and `/b`.
`/w` is the whole `CCM × demosaic × WB` matrix, so WB is a column scale on it,
not a second dispatch. `profile_builder` now omits `BayerWbBlock` whenever the
fused demosaic+CCM op is enabled; `wb_into_demosaic` is kept as a compatibility
field but no longer controls block presence. The primitive chain keeps the
separate block. `DemosaicCcmBlock::fuse_weights_ext()` is the explicit
implementation of the algebra, and `set_extra_inputs()` writes the
already-normalized form because the fused RAW head has divided by `sensor_max`.

The name `w` is intentionally generic because it is not a WB gain vector. The
logical 3A name remains `BayerWbBlock/gains`; the engine always attempts that
write first and falls back to the fused sink. A repository-wide rename
(`BayerWbBlock/wb_gain`, unified suffixes for demosaic, CCM, tone, and shader
bindings) is recorded as a P3 follow-up in `docs/overview/ROADMAP.md`. Shared
Rust constants for the current names were tried and removed: the strings are
simple, and the rename belongs in the coordinated follow-up.

The ignored `probe_wb_fused` test previously set `wb_into_demosaic: true` on
both arms, so it compared a frame with itself. It now disables `demosaic_ccm`
on the "separate WB" arm (clearing only `wb_into_demosaic` would still omit the
block under the new rule) and uses `ProcessParams::new()` so new zero-copy
fields do not break compilation. The probe still needs a device/MNN run to
validate that the two spellings are pixel-equivalent.
