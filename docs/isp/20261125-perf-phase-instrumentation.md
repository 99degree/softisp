---
title: "Perf study: pre-inference / inference / post-inference phases"
created_at: 2026-11-25T00:00:00+00:00
question: "Which code path dominates preview frame time after dedup and dual-engine pipelining?"
status: in-progress
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
3. **Pending:** run on device with a fresh APK and parse the new `pipeline timer`
   debug lines at production preview size. The old log predates those timers.
4. Update this brief with measured numbers, identify the dominant code piece,
   and propose an optimization with a concrete patch.

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
