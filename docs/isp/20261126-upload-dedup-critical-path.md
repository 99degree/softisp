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
then unconditionally uploads all of them. The working hypothesis was that
`copyFromHostTensor` persists in the session's backend buffer until the tensor
is rewritten or the session is resized, so re-uploading an identical value buys
nothing and puts a separate host→device transfer on the visible path before
`runSession`. That persistence premise has already failed once on device (see
the prior incident below), so the dedup is now **opt-in and off by default**:
the safe path uploads every queued tensor, and enabling dedup requires an
explicit switch plus a reused-slot A/B check.

## Fix

- Add `upload_cache.rs`: a per-session map from tensor name to the last
  successfully uploaded value. `f32` values are compared by `to_bits()` so NaN
  payloads and `-0.0` are not treated as equal; arrays compare bit patterns;
  raw defaults compare bytes.
- `SessionSlot` owns the cache, so each pooled session has its own view of the
  device tensors. A cross-generation or parallel session cannot suppress
  another slot's upload.
- `set_extra_inputs()` skips a write only when `cache_enabled()` and the cache
  says the device tensor already holds that exact value; it records the value
  only after the upload call succeeds. With the switch off (the default) every
  queued value is uploaded and nothing is recorded. A rejected upload is retried
  on the next frame instead of being cached as a lie.
- `SessionGuard::refresh_tensors()` clears the cache: `resizeSession` throws
  away the backend buffers, so every cached value is stale at that point. The
  vignetting gain map is written directly during resize; its names are not in
  the pending set, and clearing before the resize keeps this explicit.

## Expected effect

With dedup off, every frame uploads the full queued set — the behaviour before
the cache was introduced — while still reusing host scratch allocations. With
dedup explicitly enabled, frame 1 of each session uploads every tensor and
frames 2..N upload only values the controller actually changed (CCM/tone/WB/
scene-dependent arrays), so the per-frame prefill no longer pays for constant
graph weights. No pixels or graph semantics change when the switch is on: the
device tensors keep the same bytes they were already being given.

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

## On-device log check (2026-11-26, blue-frame report)

User report: output turns blue after a few seconds with the dedup/scratch APK.
The current `/sdcard/download/softisp-app.log` (19:26 UTC run, APK built 03:22
HKT) was checked without changing source.

Evidence from the log:

- Upload dedup was active in that APK: steady-state `pipeline stage=tensor_assign`
  is ~1–2 ms in this run, versus 5–36 ms in the preceding 18:17 run at the same
  1600×1200 RAW path. No `upload of … rejected`, `failed to upload`, resize
  failure, panic or kill appears after startup; both 4-slot pools are created
  successfully at 19:26:28 and 19:26:31.
- Controller values continue to move every frame: CCT 5200 → 4967,
  `CcmGuard stats` R_gain 5.28 → 4.86 and B_gain 2.35 → 2.28. The AWB/CCM
  state feeding the writes is therefore not frozen.
- The canonical fused sink `DEMOSAICCCMBLOCK/W` is present in the converted
  model; the legacy spelling alone is reported as absent, which is expected.
  The camera reports one recoverable `ACameraDevice error 4` at 19:26:49.858
  (frame ~359), immediately followed by `ACameraCaptureSession: ready` and
  further frames. This correlates with the user's "after a few seconds" better
  than any tensor event.

Conclusion: the log confirms the cache is skipping uploads and shows no failed
or rejected write, but it does not record skipped tensor names/values, so it
cannot by itself prove or disprove a stale-upload colour cast. A targeted A/B
run is required; do not revert the cache on log evidence alone.

### Prior incident report (2026-11-25, closest match)

The repository already contains a near-identical incident:
`docs/isp/20261125-ahb-buffer-pool-and-zero-copy-analysis.md` (§"What the log
actually shows", lines 173–211), introduced by `310995dc`
(`feat(isp): add delta tracking to extra-input uploads (roadmap item 6)`) and
removed by `9c48011e` (`fix(isp): re-upload runtime tensors and use packed
display output`).

- Symptom: visible preview went black after the early frames. Slot 0's first
  frame was bright; slots 1–3's first frames were exactly zero; later frames
  recovered as 3A values drifted and changed keys were re-uploaded.
- First diagnosis: `extra_input_cache` was engine-wide while each session had
  independent tensors. It was moved into `SessionSlot` and cleared on resize.
- Device recheck **failed**: reused sessions with unchanged runtime tensors
  still produced zero output (`session_max_abs=0.0000`, `needs_resize=false`).
  The recorded conclusion was that after a session runs, its input tensors
  cannot be assumed to still hold the previous host values.
- Fix then: delete the delta cache entirely and always upload. Device pass:
  65 frames at 800×600, `non-black 100.0%`, mean ≈ `[160,158,159]`.

Differences now: the complaint is a blue/color cast rather than black frames,
the current cache is per-slot and clears on resize, and a recoverable camera
device error falls in the same window. This is not proof for today's frame, but
it is the strongest prior: it directly falsifies the persistence premise and
identifies reused sessions as the dangerous case. The decisive A/B sample is
therefore the **second and later use of each slot with `needs_resize=false`**.

### A/B instrumentation (landed 2026-11-26)

`upload_cache.rs` reads two opt-in switches once, from the environment on the
host and from marker files in the public Download directory on the device:

| Switch | Host env | Device marker | Effect |
|--------|----------|---------------|--------|
| trace | `SOFTISP_CACHE_TRACE` | `/sdcard/download/softisp-cache-trace` | one `tensor cache frame [...]` debug line per `set_extra_inputs()` showing the colour-critical tensors as `name(upload\|hit)=value` |
| dedup | `SOFTISP_CACHE_ON` | `/sdcard/download/softisp-cache-on` | consult the last-value cache and skip identical uploads; without it every queued value is uploaded |

Both default to off. Normal builds therefore upload every tensor (the old safe
behaviour); dedup can be brought back without a rebuild by adding the marker
and relaunching. Trace covers only the fused demosaic/CCM weight, WB gains, CCM
matrix, saturation, tone scalars, FCS gain and `isp_pipeline` colour inputs; one
compact line per frame avoids rotating the crash log away. The former
`SOFTISP_CACHE_OFF` / `softisp-cache-off` switch is obsolete: its effect is now
the default, so a stale marker file is harmless.

Device procedure with a single APK: launch once to get the always-upload
baseline, then `touch /sdcard/download/softisp-cache-on` (and optionally the
trace marker) and relaunch for the dedup sample. Compare `DEMOSAICCCMBLOCK/W`
and `BAYERWBBLOCK/BAYER_GAINS` values in the trace lines against the `CcmGuard`
controller lines. Look for `name(hit)` on a reused slot with `needs_resize=false`
while the picture is wrong; that is the signature of the 2026-11-25 regression.
A single stale name can be invalidated narrowly, but a reused-slot hit that
tracks the colour fault means the whole device-residency premise is unsafe
again — remove the dedup marker until a fence-backed proof replaces it. Remove
all markers afterwards.

## Remaining work

- App-level timestamps for camera fill and display handoff (P2h, depends on
  P2b for an output AHB).
- Confirm on device that upload phase shrinks while inference stays the floor,
  and that the first frame after a geometry change re-uploads everything.
- Run the marker-file A/B above, sampling reused slots with
  `needs_resize=false` as in the 2026-11-25 incident. If one tensor is proven
  stale, invalidate only that name; if reused-slot hits track the colour fault,
  leave the dedup marker off until device residency is proven with fences.
