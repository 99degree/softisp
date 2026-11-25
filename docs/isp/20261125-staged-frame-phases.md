---
title: "Staged prep / infer / post frame phases"
created_at: 2026-11-25T00:00:00+00:00
status: implemented (pre-bound camera input buffer remains P2b-dependent)
---

# Staged prep / infer / post frame phases

## Symptom / question

`MnnEngine::process()` used to be a single call. Device phase timings recorded
in `docs/isp/20261125-perf-phase-instrumentation.md` show a 314 ms mean frame
with `setup 47.6 ms`, `tensor_assign 10.3 ms`, `input 46.9 ms`, `infer 112.2 ms`,
`output_meta 17.2 ms`, `output_copy 0.8 ms` and `stats 79.3 ms`. The camera mutex
already covered only `CameraSession::next_frame()`, but the engine still did all
prep work after the sensor buffer arrived and all 3A readback before the pixel
buffer was handed to the presenter.

## Root cause

The engine API had no way to reserve a session, upload controller tensors, run
inference and defer statistics into separate calls. Work that did not depend on
the arriving frame was serialized on the critical path, and pixels waited for a
GPU→CPU stats stall before presentation.

## Fix applied

- `cam-isp::engine::PrepareParams` captures only geometry/format/sensor/bayer
  values. It is converted from `ProcessParams` so a camera loop can compare a
  prepared frame with the frame that actually arrived.
- `MnnEngine::prepare()` acquires a `SessionGuard`, resizes the session when
  needed, reserves the output buffer and uploads the current controller tensors.
  It returns an opaque `PreparedFrame` token that owns those resources.
- `MnnEngine::run_prepared()` consumes that token, writes only the newly arrived
  input, runs MNN, decodes output bytes exactly as the old `process()` path did,
  and returns `(IspFrame, PendingStats)`. The frame is complete before the
  deferred statistics phase starts.
- `MnnEngine::finish()` performs the 3A tensor readback/controller update and
  returns `StagedFinish`. Dropping `PendingStats` skips that readback, so a
  failed send or a shutdown cannot leave the controller half-updated.
- `process()` is now the compatibility wrapper: `prepare` → `run_prepared` →
  `finish`, with `calibration_stats` and total timing restored before return.
  Existing callers and probes keep their previous semantics.
- `android-app/src/pipeline_runner.rs` prepares from the previous frame's
  geometry before `next_frame()`, runs the staged frame immediately after
  capture, sends the `Done` message, and only then calls `finish()`. First frame
  and geometry/format changes fall back to `process_staged()`. The camera mutex
  is still held only for `next_frame()`.
- The production path remains `OutputFormat::PackedRgb` through
  `isp.display_uint32_bgra`. No float fallback and no debug raw-grayscale path
  were introduced.

## Ownership and safety notes

- `PreparedFrame` holds the pool slot and output-buffer reservation. Only
  `run_prepared()` can consume it, so a reserved session is never reused while
  another frame is still in flight on the same worker.
- `PendingStats` keeps the same session alive until `finish()` has read the
  tensors. The token is not `Send`; each worker consumes it on the thread that
  prepared it.
- `PrepareParams` intentionally excludes per-frame controller values and
  zero-copy buffers. `run_prepared()` documents that callers must not change
  those between phases. The app's `PipelineConfig::params()` keeps them stable.
- Pre-binding the camera capture target to the inference input buffer is still
  an unimplemented refinement. It depends on the P2b AHB ownership/fence work;
  this change does not reuse any GPU-visible buffer while MNN still reads it.

## Verification

- `cargo fmt -p cam-isp -p softisp-anw -- --check`: clean.
- `cargo clippy -p cam-isp --lib --features "rectifier mnn" -- -D warnings`:
  clean.
- `cargo clippy -p softisp-anw --all-features --no-deps -- -D warnings`: clean.
- `cargo test -p cam-isp --lib --features "rectifier mnn" --no-fail-fast`:
  856 passed, 0 failed, 2 ignored (includes the neural-controller tests added
  on the rebased `master`).
- `cargo test -p softisp-anw --lib --all-features`: 46 passed.
- `cargo test -p cam-isp --test test_mnn_engine test_mnn_engine_staged_streaming
  --features mnn -- --ignored --nocapture`: 3 staged frames passed on the
  real MNN/Vulkan backend.
- Existing `test_mnn_engine_streaming`: 5 frames passed, so the default
  `process()` contract is unchanged.

## Residual work

- Rebuild the APK and capture production preview timings to measure the real
  latency win; this change adds the API and wiring but does not claim a new
  device number.
- P2b still owns the pre-bound camera input/AHB design.
- External cancellation / deterministic shutdown is recorded as P2d in
  `docs/overview/ROADMAP.md`.
