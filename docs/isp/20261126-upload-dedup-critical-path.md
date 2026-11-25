# Upload deduplication on the visible critical path (2026-11-26)

## Symptom

The new phase split (`docs/isp/20261126-critical-path-instrumentation.md`)
shows prefill as `setup + tensor upload`, but `MnnEngine::set_extra_inputs()`
still pushes every runtime tensor into the session on every frame. The code
comment itself records the cost: ~60 `copyFromHostTensor` calls, each with its
own host→device barrier, dominated the frame budget (~174 ms/frame at
800×600 when writes were duplicated). Most of those 60 values are block
defaults — serialized constant weights captured once in `default_inputs` — and
do not change between frames of the same session geometry.

## Root cause

`set_extra_inputs()` queues writes by tensor name (so the final value wins),
then unconditionally uploads all of them. `copyFromHostTensor` persists in the
session's backend buffer until the tensor is rewritten or the session is
resized, so re-uploading an identical value buys nothing and puts a separate
host→device transfer on the visible path before `runSession`.

## Fix

- Add `upload_cache.rs`: a per-session map from tensor name to the last
  successfully uploaded value. `f32` values are compared by `to_bits()` so NaN
  payloads and `-0.0` are not treated as equal; arrays compare bit patterns;
  raw defaults compare bytes.
- `SessionSlot` owns the cache, so each pooled session has its own view of the
  device tensors. A cross-generation or parallel session cannot suppress
  another slot's upload.
- `set_extra_inputs()` skips a write when the cache says the device tensor
  already holds that exact value, and records it only after the upload call
  succeeds. A rejected upload is retried on the next frame instead of being
  cached as a lie.
- `SessionGuard::refresh_tensors()` clears the cache: `resizeSession` throws
  away the backend buffers, so every cached value is stale at that point. The
  vignetting gain map is written directly during resize; its names are not in
  the pending set, and clearing before the resize keeps this explicit.

## Expected effect

Frame 1 of each session still uploads every tensor. Frames 2..N upload only
values the controller actually changed (CCM/tone/WB/scene-dependent arrays),
so the per-frame prefill no longer pays for constant graph weights. No pixels
or graph semantics change: the device tensors keep the same bytes they were
already being given.

## Verification

Hermetic tests in `upload_cache.rs` cover unchanged/changed scalars, signed
zero and NaN bit patterns, array length/value changes, raw bytes, kind changes
under one name, and `clear()`. The canonical gate (fmt, per-crate clippy,
lib tests, `cargo check --features "rectifier mnn"`) must pass. On-device
`perf_critical_path` is needed to quantify the millisecond saving; it is
`#[ignore]` because it requires the MNN Vulkan libraries and a device.

## Remaining work

- App-level timestamps for camera fill and display handoff (P2h, depends on
  P2b for an output AHB).
- Confirm on device that upload phase shrinks while inference stays the floor,
  and that the first frame after a geometry change re-uploads everything.
