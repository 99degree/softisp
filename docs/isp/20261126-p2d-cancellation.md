# P2d — external pipeline cancellation (design)

## Need

P2d in `docs/overview/ROADMAP.md` asks for a way for code outside the camera/ISP
workers to stop the running pipeline and have every worker wind down
deterministically, releasing camera, MNN session, buffer-pool and staged-frame
resources cleanly. A cancel followed by a restart must not leave a stale camera
device or Vulkan buffers.

## Pre-change state

- `PipelineRunner` owns the camera `Arc<Mutex<Option<CameraSession>>>`, two
  pinned ISP engines (one per worker), and the reorder queue.
- A private `Arc<AtomicBool>` stop flag is set only in `Drop`; external code has
  no handle to it. Dropping the runner joins the workers first, so `Drop` itself
  is safe, but it is not an explicit cancellation surface.
- `worker_loop` checks the stop flag only at the top of its loop. It does not
  check immediately after capture, after handing a result to the presenter, or
  after deferred stats finish.
- `softisp_anw_stop()` is unrelated: it drops the separate `IpaContext` stored
  in `ANW_CTX`, not the NativeActivity preview pipeline.
- `CameraSession::drop` closes the capture session, reader and device; the
  runner keeps the session alive until the runner itself is dropped.

## Root cause

The stop signal is private and Drop-driven. Setting it from outside cannot
release the camera or join workers, and the workers can still stage/process a
frame captured just before the flag was set.

## Implementation

1. Add a cloneable `PipelineHandle` holding `Arc<AtomicBool>`. `cancel()` is
   idempotent (`Release` store); `is_cancelled()` uses an `Acquire` load.
2. Expose `PipelineRunner::handle()` and `PipelineRunner::cancel()`. `cancel()`
   sets the flag, joins both workers exactly once (draining the join handles),
   drains the reorder queue, and clears the camera session. Worker-owned engines
   drop when their threads exit; staged `PreparedFrame`/`PendingStats` guards
   drop in reverse declaration order before the engine.
3. Add worker safe points: after prepare, immediately after capture (before
   staging/ISP), after the result is handed to the presenter and `finish()`
   consumes `PendingStats`, and before idle sleeps. A frame already in flight
   completes and its stats readback finishes; no new work starts.
4. Register the active handle in a small `anw` crate-level registry and add C
   ABI entry points `softisp_anw_cancel_pipeline()` and
   `softisp_anw_pipeline_cancelled()`. Native code can request cancellation
   without touching the runner or the surface.
5. The `android_main` loop polls the registry. When cancellation is observed it
   calls `Preview::cancel()`, logs the transition, releases the native window
   and exits cleanly. A relaunch builds a fresh pipeline with no stale camera
   device or Vulkan buffers. Cancellation never runs on a binder/camera callback
   thread and never touches `ANativeWindow` from a worker.
6. Tests: idempotent/cloneable handle semantics; workers exit after cancel;
   engine `Drop` runs after the join; camera is cleared so `has_camera()` is
   false; reorder state and worker handles are empty; the C ABI registry reports
   cancel state correctly. The MNN staged guards are covered structurally by the
   existing `finish()` path plus the reverse drop order; no GPU is required.

## Constraints preserved

- Production output stays `OutputFormat::PackedRgb` with
  `isp.display_uint32_bgra`; no float fallback or debug raw path is activated.
- No UI/installation change and no APK install by the assistant.
- No alternative MNN build directory is used; `build_isp` remains the only
  MNN build directory.

## Verification

Run from the workspace root after `cargo fmt --all`:

- `cargo fmt --all -- --check` — clean.
- `cargo check --locked --workspace --all-targets` — clean.
- Per-crate `cargo clippy --locked -p <crate> --lib -- -D warnings` for the nine
  CI crates (`cam-isp` with `rectifier`) and `softisp-anw --all-features` — all
  clean.
- Hermetic lib tests — `softisp-anw` 49 passed/0 failed; `cam-isp --features
  rectifier` 831 passed/0 failed/2 ignored. The real-MNN suite also passed
  (866 passed/0 failed/2 ignored) with the bundled native libs.
- `git diff --check` — clean; the tracked-tree search for the legacy Vulkan
  build directory finds no matches.

On-device cancel/restart remains a user-side APK check; the assistant only reads
logs.
