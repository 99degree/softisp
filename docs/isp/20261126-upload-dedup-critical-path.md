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

## Follow-up: host scratch pool for the coefficient arrays

The dedup above removes the *transfer*, but `set_extra_inputs()` still builds
each array/raw value in a fresh `Vec` every frame. `WarpGrid/grid` and
`WarpGrid/shading_lut` are the worst case: a full displacement map copied from
the caller into a new allocation on every frame even when the dedup cache later
skips the upload.

Design: the scratch buffers live on the same per-slot `UploadCache`, keyed by
tensor name. `take_f32`/`take_raw` remove the slot's buffer, clear it and
reserve the needed length; the write helpers fill it; the flush loop returns it
with `put_f32`/`put_raw` (including on a cache hit or an upload failure), so a
steady-state frame performs no host allocation for fixed-shape coefficients.
Keeping the scratch on the slot (rather than thread-local) matches the device
tensor residency and avoids a fresh `String` key per frame — the `pending`
HashMap key is moved back into the scratch map. Scratch buffers are plain host
memory and survive `clear()`; they are resized on next use, so a session
resize does not force a reallocation.

## Verification

Hermetic tests in `upload_cache.rs` cover unchanged/changed scalars, signed
zero and NaN bit patterns, array length/value changes, raw bytes, kind changes
under one name, `clear()`, and pointer-level reuse of f32/raw scratch buffers.
The canonical gate (fmt, per-crate clippy, lib tests,
`cargo check --features "rectifier mnn"`) must pass. On-device
`perf_critical_path` is needed to quantify the millisecond saving; it is
`#[ignore]` because it requires the MNN Vulkan libraries and a device.

## Remaining work

- App-level timestamps for camera fill and display handoff (P2h, depends on
  P2b for an output AHB).
- Confirm on device that upload phase shrinks while inference stays the floor,
  and that the first frame after a geometry change re-uploads everything.
