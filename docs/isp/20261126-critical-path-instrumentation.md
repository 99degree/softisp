# Critical-path phase instrumentation (2026-11-26)

## Symptom

P2h's goal is to make the visible per-frame path as close to `runSession` as
possible: upload metadata while the sensor fills the input buffer, present the
output as soon as inference ends, and defer 3A readback. The probe only reported
`prep`, `infer` and a residual `post`, so it could not say which part of that
residual was movable — output metadata, pixel copy, or deferred stats — nor how
much of `prep` was controller work versus tensor upload.

## Fix

`MnnEngine` already measured the fine phases for a `debug!` line. They are now
exposed as data rather than only a log:

- `pipeline::FramePhaseTimings` carries `setup_ns`, `tensor_upload_ns`,
  `inference_ns`, `output_meta_ns`, `output_copy_ns` and `readback_ns`, with
  `prefill_ns()`, `output_ns()`, `visible_ns()` and `total_ns()` helpers.
- `IspAuxOutput::timing` is `Some(...)` on MNN frames. `visible_ns()` excludes
  readback: it is what a staged caller waits for before presentation.
- `StagedFinish::readback_ns` reports the time inside `finish()` itself,
  separately from `total_duration_ns` which still includes prep.
- `MnnEngine::process()` copies the readback duration into the frame's timing so
  the non-staged path reports the same split.

`probe_4k_perf` now aggregates all of these. Each row gets a second line:

```text
critical <visible> ms = prefill <setup + upload> + infer <runSession> + output <meta + copy>; deferred readback <...>
```

and `perf_critical_path` runs FUSED/PackedRgb with stats on and off so the
readback that was moved off the visible path is quantified directly.

## Phase map

| Phase | Timer source | Movable off visible path? |
|---|---|---|
| setup | prepare() start → tensor work start | Yes, overlaps capture fill |
| tensor upload | `copyFromHostTensor` flush | Yes, overlaps capture fill |
| inference | `runSession` | No — this is the floor |
| output meta | post-inference shape/format resolution | Partly, with a pre-bound output |
| output copy | output tensor → caller buffer | Yes, with AHB display handoff |
| readback | `finish()` | Already deferred by staged callers |

Camera acquire and compositor handoff are deliberately absent: they live in
the application/camera layer, not the engine. Measuring them needs app-level
timestamps around `AImageReader_acquireLatestImage` / `ANativeWindow_lock`; that
remains in P2h and depends on P2b for an output AHB the compositor can consume.

## Running it

```bash
# focused critical-path split, 4K, stats on vs off
PERF_FRAMES=40 cargo test -p cam-isp --test probe_4k_perf perf_critical_path \
  --features mnn -- --ignored --nocapture

# full matrix (prints the same split on every row)
cargo test -p cam-isp --test probe_4k_perf perf_4k_all \
  --features mnn -- --ignored --nocapture
```

The current device baseline is in `README.md` ("Where the 4K time goes"): after
pooling the statistics taps, 3A is ~6% of the frame and the readback is ~1.7 ms
(~1% of p50) at 4K. Compare rows from the same run — absolute numbers drift
with device temperature.

## Verification

`cargo fmt --all -- --check`, `cargo check -p cam-isp --tests --features
"rectifier mnn"`, and `cargo test -p cam-isp --lib --features rectifier` (844
passed, 2 ignored) are clean. The probe itself is `#[ignore]` and needs the
real MNN Vulkan libraries and an Android device.

## TODO

- Add app-level timestamps for camera fill and display handoff so the probe
  can print the full five-way split named in P2h.
- Once P2b lands an output AHB, report display-release fence wait as its own
  phase instead of folding it into the app's presentation time.
- Confirm on-device that `visible_ns()` matches wall-clock time from buffer
  ready to presentation within the clock overhead of the two measurements.
