---
title: "Analysis: AHB output buffer not returned to pool — no valid frames"
created_at: 2026-11-25T00:00:00+08:00
question: "Is the AHB sharing mechanism failing: output frame/tensor handed to Android is never freed back to the buffer pool, so no valid frame can be output?"
status: resolved — hypothesis refuted; real defects identified below
---

# Analysis: AHB output buffer not returned to pool

## Symptom (user report)

- Local repo updated with the new code (dual-Vulkan fused head, AHB
  zero-copy input/output via AHardwareBuffer Vulkan device memory,
  OutputBufferPool).
- Suspected failure: the AHB sharing mechanism — the output tensor/frame
  buffer is sent to Android (display/consumer side) and is **not freed
  back to the buffer pool** afterwards, so the pool runs dry and
  **no valid frame can be output** (preview stays blank after the first
  frame, or no frames at all).
- Related prior analysis:
  [`20261124-first-frame-normal-subsequent-blank-analysis.md`](20261124-first-frame-normal-subsequent-blank-analysis.md)
  (first frame normal, subsequent blank — probe/perf context).

## Evidence — logs grabbed

- Current: `/sdcard/download/softisp-app.log` (124 862 B, ends
  11-24 16:35:25 UTC = 00:35 local, copied to
  `.tmp/softisp-app-latest.log`).
- Rotated: `/sdcard/download/softisp-app.log.1` (524 297 B, sessions
  14:09 / 14:44 / 14:50 / 15:31 UTC).

## Verdict: AHB hypothesis REFUTED for this build

1. **The AHB import/export path never runs in the APK.**
   - `mnn_import_ahb_as_device_memory` / `mnn_release_ahb_device_memory`
     are only called in `mnnengine.rs` inside the `if zero_copy` block,
     and `zero_copy = p.output_buffer.is_some()`.
   - The only producer of `output_buffer` / `output_ahb` /
     `input_ahb` is `CameraIspService::process_raw_frame_zc`
     (`integration.rs:243-254`) — **zero callers** anywhere in the
     APK's dependency graph.
   - `android-app` depends only on `cam-isp` + `cam-types`
     (`android-app/Cargo.toml`); `cam-hal-android` (HardwareBufferPool)
     and `cam-binder` (HardwareBufferBridge) are **not linked**.
   - `grep 'input_buffer|output_buffer|_ahb' android-app/src` → 0
     matches: the app never sets them.
   - The log line `path=raw_zero_copy` is a **misleading label** — the
     `InputMode::RawInt` branch names itself `raw_zero_copy`
     unconditionally (mnnengine.rs:2220), regardless of zero-copy.
2. **No buffer is left un-freed in the app's real output path.**
   - `OutputBufferPool::release()` is a deliberate no-op (round-robin);
     the pool structurally cannot run dry. Log confirms 2 pools (one
     per engine) cycling `idx 0,0,1,1,2,2,…` — exactly the dual-engine
     interleaving — with exactly 2 `grow` lines (once per pool).
   - Display: every present takes `ndk` 0.9.0's
     `NativeWindowBufferLockGuard`, whose `Drop` calls
     `ANativeWindow_unlockAndPost` and **asserts ret==0** — a buffer
     that failed to post would panic; there is no panic in either log,
     and zero `present failed` ERROR lines.
   - Camera input: all `AImage`s are wrapped in `ImageGuard`
     (`AImage_delete` on drop); no acquire path skips it.

## What the log actually shows (real defects)

1. **The installed APK is stale**: it logs
   `pipeline built: profile FUSED … (dual GPU engine)` — HEAD's
   `e49adb6f` ("use single GPU engine to isolate 4K black screen
   issue") switched the app to `(single GPU engine)`. The device runs a
   pre-`e49adb6f` build (dual-engine ping-pong, `DualGpuEngine`).
2. **Intermittent exactly-zero display output** (the "no valid frame"
   moments):
   - Every run: frame 1 bright (`[172.4,181.5,160.4]`), **frame 5
     exactly `[0.0,0.0,0.0]`, non-black 0.0%**, frame 10 dim
     (`[23.7,…]`), frame 15+ normal (`[131.7,…]`).
   - Old log frame 80: `mean [0.0,0.0,0.0], non-black 66.7%` — mean 0
     with 2/3 of planes >0 = R,G tiny-positive, B plane exactly 0.
   - Crucially, on the black frames the **early-chain stats are
     non-zero** (`stats: planes [0.146,…]` = ChannelMeansBlock) while
     the **tail output is zero** → the failure is in the chain's tail
     (tone/gamma/display), not in buffer hand-off.
   - Frame 80 black had `needs_resize=false` (no resize involved), so
     it is not a resize artifact; frame 5 coincides with AWB
     `Scene transition: Dark → Indoor`.
   - Tail levers written per frame (`mnnengine.rs:1146-1158`):
     `ToneBlock/contrast|brightness|gamma_recip|exposure` (exposure
     floored at 0.05), plus `Gamma/*`, `AutoContrast/*` from
     `isp.tone.*`. Strongest hypothesis: **tone-controller feedback
     (ToneStatsBlock → get_tone_params) drives the display tail to
     black during startup convergence and scene transitions** — to be
     confirmed by logging the tone params on black frames.
3. **`read_output_floats(CoarseHistogramBlock/frame) → -3`
   (tensor not found) every frame** → the histogram AE input is
   permanently missing; `controller_exposure::update_histogram` never
   receives data.
4. **Param tensors missing from the FUSED graph** (warned every frame):
   `BayerWbBlock/gains`, `CcmBlock_ccm/matrix`, `DemosaicBlock/w|b`,
   `FcsBlock/gain|bias`, plus several `default for … has no tensor`
   → controller write-backs are no-ops (3A cannot affect the image).
5. **`ACameraDevice: error 4` after 20–38 s in every run** → the
   camera session dies (known ~20 s issue; run then ends with
   Pause/Stop/NativeWindowDestroyed).
6. Noise: `InitWindow callback reached` is logged **every loop
   iteration** (lib.rs:793-802), not on InitWindow — misleading
   diagnostic, harmless.

## Working-tree state at time of analysis (user WIP, untouched)

- `build-apk.sh`: `cargo build -p cam-camera-app` line commented out
  (line 31) — makes step 3 patch a possibly stale `libcam_camera_app.so`.
- `cam-rust/cam-isp/src/profile_builder.rs`: WIP geometry fix —
  `UnpackCfaBlock::with_downscale(2)` + demosaic concrete dims halved
  to `cfa_h/cfa_w`.
- `cam-rust/cam-isp/src/unified_pipeline.rs`: test `target_width`
  640 → 1280.

## Black-frame correlation analysis (2026-11-25 02:00 local)

Frame↔slot mapping for the 16:35 run (dual engine A/B × 4-session pool,
strict alternation, round-robin within engine):

- `needs_resize=true` × 8 consecutive at 6126/6925/7525/7972/8394/8864/9319/
  9757 ms = **each of the 8 slots' first use** (A/s0, B/s0, A/s1, B/s1,
  A/s2, B/s2, A/s3, B/s3 → frames 1–8). After that: `needs_resize=false`
  × 28 (slots already sized) — the "resize storm" is startup-only.
- Measured frames (stats log every 5): frame 1 (A/s0, first use, resize
  182 ms) = **bright** [172.4]; frame 5 (A/s2, first use, resize 34 ms) =
  **exactly black**; frame 10 (B/s0, second use, no resize) = **dim**
  [23.7]; frame 15+ = normal. Frames 2–4, 6–9 are UNMEASURED — they may
  have been black too; the every-5 stats cadence hides it.
- `ChannelMeansBlock` (linear composer order: display → stats_downscale →
  zone_stats → channel_means, i.e. nominally *downstream* of DisplayBlock)
  is NON-ZERO [0.146,…] on the black frame 5 → either the stats chain taps
  the graph before DisplayBlock, or the session tensor is fine while the
  pool readback is zero. **Instrumentation added to discriminate** (below).
- Hypotheses tested and killed:
  - `ToneBlock/gamma_recip = 1/tone_gamma → inf → pow(x,inf)=0`: REFUTED —
    `sanitize_tone` clamps `tone_gamma` to [0.5, 4.0] (recip ∈ [0.25, 2]).
  - `packed_display` heuristic misfire (known blank-frame trap per comments
    at mnnengine.rs:2447/2476): constant per engine (`has_fused_display`,
    `out_type` are init-time/frame-constant) → cannot explain *intermittent*
    zeros. It IS the likely cause of the pre-existing
    `test_manager_build_and_process` 4×-bytes failure (f32 output read via
    packed branch → `valid_bytes = out_elems×4` returned un-narrowed).
  - Gain-map-stale-handles after resize: the fix already exists
    (`slot.refresh_tensors(interp)` after `resize()`, mnnengine.rs:2003,
    with the author comment "…or those writes land in freed memory and the
    frame comes back black") — yet frame 5 was still black, so either a
    handle path was missed or the zero originates elsewhere.
- `mnn_run_with_output` (mnn_wrapper.cpp:412) may call `resizeSession`
  internally when the element count changed — a possible *second* resize
  after Rust wrote the extra inputs. Not yet verified against the FFI
  check order.

## ROOT CAUSE (confirmed 2026-11-25 03:35 local, device log)

New instrumented APK (single engine) log: `/sdcard/download/softisp-app.log`,
run at 19:30 UTC, 1514 lines. Three `black frame:` WARNs — every one with
`branch=raw out_type=4 out_elems=1440000 graph=800x600 needs_resize=true
session_display=Some(1440000) session_max_abs=0.0000`.

Interpretation:
- The **session's own DisplayBlock output tensor is all zeros** — not a
  readback/branch/present problem, not AHB, not gamma clamping. The graph
  genuinely produced a black display tail.
- Single-engine switch did NOT fix it; it merely reduced the resize frames
  from 8 (dual engine × 4 slots) to 4 (single engine × 4 slots).
- New-run timing: 4 `needs_resize=true` frames = each of the 4 pool slots'
  first use (3282/3793/4280/4718 ms). **Slot 0's first frame is BRIGHT
  [155.7]; slots 1/2/3's first frames are exactly BLACK** (4038/4481/4892 ms).
  Frame 5 (slot 0 reused) dim [16.6]; frames 10+ normal [114.6].

Cause: `MnnEngine::extra_input_cache` (mnnengine.rs:197) is a single
`Mutex<HashMap<String, Vec<u8>>>` shared by ALL sessions, but each session
in `SessionPool` has its own `tensor_pool` of independent tensors.
`write_f32`/`write_f32_array`/`write_raw` skip an upload whenever the cached
bytes for a name are unchanged. Frame 1 (slot 0) populates the cache;
frames 2–4 (slots 1–3) then skip every write → their runtime input tensors
stay at zero → `DisplayBlock/frame` zeros. As controller values converge,
changed keys are re-uploaded to whichever slot is current, so later frames
recover. `set_extra_inputs` is the only cache consumer (one call site).

Fix (applied below): move the delta cache into `SessionSlot` so each session
tracks its OWN uploads, and clear it on `resize()` (resize reallocates the
tensors, so cached values no longer describe their contents).

Applied changes:
- `mnn_session_pool.rs`: `SessionSlot` now owns
  `extra_input_cache: Mutex<HashMap<String, Vec<u8>>>` (initialised empty per
  slot); imports `HashMap`.
- `mnnengine.rs`: removed the engine-wide field and constructor init;
  `process()` passes `&slot.extra_input_cache` to `set_extra_inputs`; after
  `refresh_tensors()` on resize the slot's cache is cleared.
- Verification so far: `cargo fmt --all -- --check` OK;
  `cargo clippy -p cam-isp --lib --features "rectifier mnn" -- -D warnings`
  OK (only pre-existing C++ wrapper warnings);
  `cargo test -p cam-isp --lib --features "rectifier mnn"` → 844 passed,
  the SAME 2 pre-existing failures (no regressions).
- Device verification **FAILED**: new log 3002 lines, 14 black frames across
  three single-engine runs. The first four one-use frames are now bright, but
  frames 6–10 (reuses of the same four slots) are black with
  `needs_resize=false`, `session_max_abs=0.0000`. This shifts the fault from
  first-use/resize to **reused sessions with unchanged runtime tensors**, i.e.
  the delta-cache skip itself is unsafe: after a session runs, its input
  tensors cannot be assumed to still hold the previous host values. A per-slot
  cache still suppresses the upload on the second use of each slot; as
  controller values drift, writes resume and frames recover (frames 11–15).
- Next fix applied: remove the delta cache entirely (always upload every
  runtime input before inference). The traffic is a few hundred bytes per
  frame; correctness outweighs it. The per-slot cache was removed too, so
  `mnn_session_pool.rs` is back to its original state.
- Per user direction, the app's preview output format was changed from
  `FloatRgb` to `PackedRgb` ("unpack RGB"): `android-app/src/pipeline.rs`
  now sets `OUTPUT_FORMAT = OutputFormat::PackedRgb` and `float_output()`
  returns false, so the fused `isp.display_uint32_rgba` path hands back
  BGRA bytes directly and no planar-f32 conversion sits between the graph
  and `present()`.
- Follow-up from user: the fused `isp.display_uint32_rgba` shader writes ONE
  BGRA uint32 per pixel (confirmed in MNN's `isp_display_uint32_rgba.comp`),
  while `DisplayBlock` declared `[1,1,H,W/2]` for PackedRgb. That would make
  `copyToHostTensor` return half a frame. Fixed `packed_output_width()` to use
  `W` for the fused opset, and added an explicit uint32/int32 tensor-type
  check in `mnnengine.rs` (`packed_u32`) with its own readback branch and
  `branch=packed_u32` diagnostic. Primitive PackedRgb packing stays W/2.
- Verification after both changes: `cargo fmt --all -- --check` OK;
  `cargo clippy -p cam-isp --lib --features "rectifier mnn" -- -D warnings`
  and `cargo clippy -p cam-camera-app --all-features -- -D warnings` OK;
  `cargo test -p cam-isp --lib --features "rectifier mnn"` → 845 passed
  (one new test for the fused packed output width), the same 2 pre-existing
  failures; `cargo test -p cam-camera-app --lib --all-features` → 46 passed.
  Device verification (single-engine APK) PASSED: last run reported 65 frames
  at 800x600, `rgb mean ≈ [160,158,159]`, `max 255`, `non-black 100.0%` at
  every checkpoint, no `present failed`. Rate: 64 frames in 30.64 s ≈ **2.09
  fps** at 800x600 (frame took 533–606 ms warnings; isp ≈ 240 ms).
- Per user request, the single-engine isolation commit `e49adb6f` was reverted
  on top of the fixes and pushed as `759003e6`; the packed geometry fix is
  `5fe35e0f`. A dual-engine APK is being rebuilt for a comparison run.
- Correction on the `session_max_abs` evidence: the detector samples every
  `len/128` bytes; for `len=1,920,000` that step is 15,000 — a multiple of 4,
  so it always sampled the zero low byte of a `0xFF000000` BGRA word. The main
  readback itself was fine; the frame was opaque black because the shader wrote
  RGB=0 (gamma=0), and presentation failed only because of the wrong dims.
- Device test of `13f70f42` was still all-black. Log evidence:
  `black frame:` reported `bytes=1920000 branch=packed_u32 out_type=6
  out_elems=480000`, while `present failed: source frame too small: 1920000
  bytes for 1600x1200 BGRA, need 7680000`. The graph output is 800x600 packed
  (WIP downscale), but `mnnengine.rs` deliberately kept the *requested*
  1600x1200 geometry for PackedRgb, so the compositor rejected every frame.
  Fixed by trusting the output tensor's `[1,1,H,W]` shape for PackedRgb too.
- Second cause: the packed shaders (`isp_display_uint32_rgba.comp` etc.) read
  gamma from `Uniforms[2]` in the binding-0 SSBO, but `DisplayBlock` passes
  gamma as extra input[1] (binding 5), which only `isp_display.comp` reads.
  `u.gamma == 0` → `pow(x, 1e6) == 0` → every word was `0xFF000000` (matching
  the observed `session_max_abs ≈ 1.71e38`). Patched MNN's `VulkanFuse.cpp`
  (external dependency at `~/MNN`, not repo-tracked) to copy the gamma input
  into `mConstStorageBuffer[2]` for every `isp.display_*` op, then rebuilt the
  libs with `scripts/build_mnn_libs.sh arm64-v8a`.
- Committed and pushed as `13f70f42` after rebasing onto upstream
  `ca6ebe15` (sensor orientation fix). User WIP in `build-apk.sh`,
  `profile_builder.rs` and `unified_pipeline.rs` was stashed around the
  rebase and restored; `.agent/` remains local scratch. APK is being
  rebuilt once more so the device test includes the upstream orientation fix.

## Instrumentation added (commit-pending)

In `mnnengine.rs` right after the `data_out` branch chain: on an all-zero
frame (sampled at 128 points, allocs only when black), WARN
`black frame: bytes=… branch=… out_type=… out_elems=… graph=WxH
needs_resize=… session_display=Some(n)/None session_max_abs=…`.

- `session_max_abs > 0` + zero bytes → **readback/branch bug** (pool buffer
  not what MNN wrote, or wrong branch).
- `session_max_abs = 0` (or `None`) → **graph tail genuinely produced
  black** (zeroed inputs / display op).
- `branch` + `out_type` also validate the `packed_display` heuristic live.
- fmt + clippy (`-D warnings`, rectifier) green; APK rebuild in progress.

## Verification status (2026-11-25 01:45 local)

- **APK rebuilt** from current tree (single-engine HEAD + user WIP):
  `target/debug/apk/softisp-camera.apk` (210 628 201 B, signed 01:24),
  verified: `(single GPU engine)` string in the app lib,
  `libsoftisp_cpp_shared.so` NEEDED patch applied (no `libc++_shared.so`).
  Copied to `/sdcard/download/softisp-camera.apk` for install.
- **Install**: `./install-apk.sh` FAILED on this device —
  `pm install-write` (stdin/reverse mode) → `SecurityException: Reverse mode
  only supported from shell or system` (Termux is not shell UID).
  `docs/build/DEBUG_APK.md` §1 explicitly forbids `pm` installs from Termux
  and mandates manual install by the user — followed that instead.
- **Gate tests** (with `mnn` feature, `LD_LIBRARY_PATH=cam-rust/lib/aarch64-v8a`):
  - `cargo test -p cam-isp --lib --features "rectifier mnn"` →
    **844 passed, 2 failed, 2 ignored**. Both failures **reproduce at clean
    HEAD** (verified by stashing the WIP edits) → pre-existing defects, NOT
    caused by the WIP:
    1. `unified_pipeline::tests::test_unified_force_input16` —
       `runSession failed: COMPUTE_SIZE_ERROR` (graph dims inconsistent).
    2. `manager::tests::test_manager_build_and_process` — output frame is
       4× the expected byte count (230400 = 160×90×16 instead of ×4 →
       likely 2× spatial upscale, RGBA-f32 vs BGRA-u8 or doubled dims).
    Both are the same *geometry* bug class the WIP `profile_builder.rs`
    downscale fix is aimed at — candidate shared root cause with the
    black-frame tail issue.
  - `cargo test --workspace --doc --all-features` → blocked: cannot download
    `openssl 0.10.82` from `static.crates.io` (DNS down / offline env).
    Environmental; rerun when network returns.

## Fix / next steps

- AHB: nothing to fix in the running path — it is dead code for the
  APK. If AHB zero-copy is wanted later, `process_raw_frame_zc` needs
  callers (HAL path) and `cfb6ed31` notes device-binding wiring in the
  MNN Vulkan backend is still pending.
- Rebuild + reinstall the APK from current tree (single-engine HEAD +
  WIP) — the device build predates `e49adb6f`.
- Then chase the tail-zero frames: log `ToneBlock/*` /
  `isp.tone.*` values on black frames; verify whether
  `ToneStatsBlock` feedback drives them.
- Add `CoarseHistogramBlock` to the FUSED graph or drop its read;
  wire the missing `BayerWbBlock/gains` + `CcmBlock_ccm/matrix`
  tensors so 3A actually reaches the graph.
- User reported inverted colours on the visible preview. Root cause:
  `isp.display_uint32_rgba` packs LE bytes `[R,G,B,A]`, but the host
  `WindowTarget::present()` assumes BGRA input and swaps R/B when writing
  R8G8B8A8 window bytes. Switched `PackedRgb` to `isp.display_uint32_bgra`
  (which packs `[B,G,R,A]`), committed/pushed as `3f4b2661`; APK rebuilding.

## Inference performance study (dual-engine APK, 800x600 output)

Measured from the 05:43 dual-engine device run (66 `output_frame` samples,
1600x1200 RAW → 800x600 packed BGRA):

| stage | mean | median | min | max | share of total |
|-------|------|--------|-----|-----|----------------|
| total (prep + infer) | 310.6 ms | 311.9 | 186.6 | 469.2 | 100% |
| prep (shape/allocation/**tensor_assign**) | 221.8 ms | 227.1 | 116.9 | 301.2 | 71% |
| ├─ tensor_assign (resizeSession + extra-input uploads) | 173.4 ms | 176.9 | 90.2 | 242.7 | **56%** |
| └─ other prep | 48.4 ms | — | — | — | 16% |
| infer (`runSession` + zero-copy readback) | 88.8 ms | 89.5 | 59.7 | 173.8 | 29% |

`total = prep + infer` exactly (221.8 + 88.8 = 310.6); `tensor_assign` is a
sub-interval of `prep`, not an additive stage. End-to-end app rate was
**2.09 fps** (64 frames / 30.64 s), so camera capture + `present()` add another
~170 ms/frame beyond the engine's 310 ms.

### Root cause of the 173 ms tensor_assign

`MnnEngine::set_extra_inputs()` issues **~60+ separate uploads per frame**,
each going through `MnnTensorSafe::upload_f32/upload_bytes` →
`mnn_set_input_float` → `Tensor::create(host)` + `copyFromHostTensor()`. On
Vulkan each call is an independent host→device transfer with its own barrier;
at ~2.5 ms each this accounts for the whole 173 ms. The payload is tiny —
mostly 4-byte scalars and 3–27 element arrays — so this is **per-call latency,
not bandwidth**.

Two structural sources of waste:

1. **Duplicate writes.** `extra_input_defaults()` (written in one bulk loop at
   the top) and the explicit write block overwrite the same tensors every
   frame. Confirmed overlaps: `EeBlock/kernel_ee`, `EeBlock/bias_ee`,
   `Gamma/{inv_gamma,min,max,lift,norm}` (5),
   `isp_pipeline/{white,wb,ccm,gamma,lift,pattern,sat,dpc}` (~8),
   and `ToneBlock/{contrast,brightness,exposure}` — roughly **18–20 redundant
   uploads/frame (~45–50 ms)**. The values are not always identical (the
   explicit path applies controller/ISP overrides), so the defaults must stay
   for tensors with no explicit writer; the fix is a name-level de-duplication
   that writes each tensor exactly once with its final value.

2. **Invariant constants are still runtime inputs.** `AutoContrast/half=0.5`,
   `AutoContrast/zero=0`, `Gamma/norm=1`, `DisplayBlock/scale=1`, the per-block
   `zero`/`one` clip bounds, demosaic tables, EE kernel, etc. never change for
   a given sensor. If MNN's converter can keep them as ONNX initializers for
   custom `isp.*` ops, they would be uploaded **once at build** instead of
   every frame (~30 more uploads removed).

### Why the obvious fix (skip unchanged uploads) was rejected

A delta cache was tried twice (engine-wide, then per-`SessionSlot`) and both
times produced black frames on the device. MNN's Vulkan backend does **not**
preserve a runtime input tensor's host contents after `runSession()`; skipping
an upload leaves the tensor at zero. Re-uploading every runtime input is a
correctness requirement, so any optimization must reduce the **number** of
upload calls, not skip them.

### Prioritized opportunities

| # | Change | Expected gain | Risk |
|---|--------|---------------|------|
| 1 | De-duplicate `extra_input_defaults` vs explicit writes (write each name once with its final value) | −45–50 ms/frame (~15%) | Low — same values, ordering preserved |
| 2 | Bake invariant constants as ONNX initializers (verify MNN converter keeps them for `isp.*` ops) | −60–75 ms/frame (~20–25%) | Medium — converter-dependent, needs device check |
| 3 | Pack all scalar runtime params into ONE `[N]` tensor and `Gather`/`Slice` inside the graph | 60 uploads → 1, potentially −150 ms/frame | High — graph/converter/shader rewrite |
| 4 | True dual-engine pipelining. `DualGpuEngine::process()` is currently strictly sequential (`next().process()`), so the second engine gives **no** overlap — only doubled session memory. Real pipelining needs async MNN sessions / separate Vulkan queues to overlap frame N+1's uploads with frame N's 89 ms `runSession` | up to −89 ms/frame | High — MNN backend change |
| 5 | AHB zero-copy for input **and** output is present in `mnn_wrapper.cpp` but unused by the app (`p.input_ahb`/`p.output_ahb` are never set); wiring the HAL path would remove the remaining host readback | −readback portion of the 89 ms | Medium — needs HAL + device-memory import verified |
| 6 | Sensor binning / lower capture resolution (e.g. 800x600 instead of 1600x1200) | proportional across all stages | Low but reduces image detail |

**Recommended order:** 1 → 2 → device re-measure. Items 3–5 are architectural
and should follow a roadmap entry (per AGENTS.md §6, no large refactors without
one). Item 1 is safe to implement immediately; item 2 needs a quick converter
probe before committing to it.

### Cleanup performed this session

- Restored `cam-rust/cam-isp/src/unified_pipeline.rs` from HEAD — both the
  pre-existing `target_width` 640→1280 experiment and my accidental sed of
  `hd()` 1280→640 were dead experiments.
- Kept `build-apk.sh`'s skipped standalone `cargo build` (user confirmed
  `cargo apk2` builds the library) and clarified its comments.
- Kept and committed `profile_builder.rs`'s half-resolution CFA dims: the
  INT16 fused head emits `[1,4,H/2,W/2]`, so `DemosaicCcmBlock` and
  `DemosaicBlock` must be told the CFA geometry, not the full sensor geometry.
  Committed as `8fb20c96` together with the `build-apk.sh` comment cleanup.

## Deep-dive: opportunity #4 (dual-engine pipelining) — 2026-11-25

`DualGpuEngine::process()` is `self.next().process(params)` — a plain
round-robin selector, no overlap. `IspEngine: Send + Sync`, so two separate
engines can be driven from two worker threads; MNN's `runSession()` holds a
per-interpreter lock (`Interpreter.cpp:519-537`), so the *four sessions inside
one engine cannot overlap*, but two engines (two interpreters) can.

The app loop (`android-app/src/lib.rs:535-721` `render_once`) is strictly
sequential: `next_frame()` → `stage()` → `process()` → `present()`, and the
next capture only starts after present. `FrameStager` owns one reusable `Vec`
whose borrow is valid only until the next `stage()`, so it cannot be shared
between workers. `CameraSession` is `Send` (not `Sync`) and must sit behind a
`Mutex`. `WindowTarget<'static>` and `last_frame` must stay on the main thread.

Concrete design adopted: a `PipelineRunner` owning
`Arc<Mutex<Option<CameraSession>>>`, two pinned engines (one per worker), an
unbounded `mpsc` result channel, an `AtomicU64` sequence counter assigned under
the camera lock, and a `BTreeMap` reorder queue on the main thread. At most two
results are in flight (one per worker), so the channel is naturally bounded.
Workers capture+stage+process; main thread presents in sequence order and owns
all repaint/stats state. Shutdown sets an `AtomicBool` and joins both workers.

Hazards: out-of-order completion (solved by seq + reorder), stager aliasing
(solved by per-worker stager), window lifecycle (present stays on main), and
warm-up resize hitch (both engines are already built before workers spawn).

## Deep-dive: opportunity #5 (AHB zero-copy) — 2026-11-25

The AHB surface in cam-isp is a **stub that cannot move pixels on Vulkan**:

- `mnn_import_ahb_as_device_memory()` (mnn_wrapper.cpp:632-673) only calls
  `AHardwareBuffer_getFd` for a log line, then `close(fd)`, then
  `tensor->setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)`. That only writes
  `mBuffer.flags`/`device`. Vulkan `onAcquire()` (VulkanBackend.cpp:374-401)
  ignores the flag and allocates from its own pool, overwriting `device`.
  Only the OpenCL backend consumes `setDevicePtr`.
- `mnn_run_true_zero_copy()` (mnn_wrapper.cpp:560-603) binds caller pointers
  into `buffer().host` and calls `runSession()`. Vulkan ignores host pointers
  too (the repo's own `mnn_run_with_output` comments document this and use
  `copyFromHostTensor`/`copyToHostTensor` instead). It also uses
  `getSessionInput(sess, nullptr)` — the alphabetically first input, which in
  our ~60-input graph is **not** the RAW frame — and never calls
  `tryResizeSession`.
- `mnnengine.rs` passes `output_name` as the tensor name for **both** AHB
  imports (input should use `self.frame_input_name`), so both bind the same
  output tensor. All four FFI return codes are discarded.
- The APK never sets `input_ahb`/`output_ahb`; `PipelineConfig::params()`
  (android-app/src/pipeline.rs:81-96) leaves them `None`, and `zero_copy` is
  gated on `p.output_buffer.is_some()` so the branch is entirely dead.
- The camera reader is created with `AImageReader_new` (no GPU usage bits) and
  `AImage` is deleted in `next_frame()` before inference. `ANativeWindow` has
  no NDK API to obtain its `AHardwareBuffer`.

Conclusion: full end-to-end AHB zero-copy needs MNN Vulkan backend changes
(import in `onAcquire`, per-tensor external memory with non-freeing release),
camera `newWithUsage` + AHB lifetime changes, and an output-surface design —
all outside the repo and not device-testable here. Implementing it now would
reintroduce black frames. Action taken: fix the in-repo correctness bugs and
document the blocker; do not wire the APK.

### Follow-up: MNN global-AHB serialization removed (2026-11-25)

The user flagged that the MNN display-AHB patch used a process-wide global
(`g_publishedDisplayAhb` + `mnnSetDisplayAhb`) substituted by `getBuffer()` for
every `deviceId()==0` tensor. With two ISP workers this would serialize on and
race over one buffer. Replaced in `~/MNN` (not the softisp repo) with the
per-tensor API the user prefers:

- Deleted the global, `mnnSetDisplayAhb`, `VulkanBackend::setSharedAhb` and
  `mSharedAhb` from `VulkanBackend.{cpp,hpp}`.
- `VulkanBackend::onAcquire()` now imports the AHB carried by
  `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` into a
  `VulkanBufferFromAHB` owned by a `VulkanAhbMemObj` stored in the tensor's
  `TensorUtils::sharedMem`. Each tensor/engine owns its own imported VkBuffer;
  no global and no serialization. Small tensors never take this path — only
  tensors explicitly flagged by the caller (the large display output).
- `getBuffer()` and all three `onCopyBuffer()` directions now resolve the
  VkBuffer through `getBuffer()` so AHB-backed tensors never get
  `deviceId()` mis-cast to `VulkanBuffer*`.
- Rebuilt with `./scripts/build_mnn_libs.sh arm64-v8a` (the script's `build_isp`
  path is the only supported MNN build). The external `VulkanFuse.cpp` gamma
  patch is still present and was preserved by the rebuild.

In-repo correctness fixes applied alongside:
- `mnnengine.rs` input AHB import now uses `self.frame_input_name`
  (`frame_name_ptr`), not the output name; all import/release FFI return codes
  are checked and logged.
- `mnn_wrapper.cpp` `mnn_run_true_zero_copy()` now takes explicit input/output
  names, uses `tryResizeSession`, and uploads/reads via
  `copyFromHostTensor`/`copyToHostTensor` (host-pointer binding is ignored by
  Vulkan). `AHardwareBuffer_getFd` precedence bug fixed; fd query is now
  diagnostic-only.

Verification: `cargo fmt --all -- --check` OK; `cargo clippy -p cam-isp --lib
--features "rectifier mnn" -- -D warnings` OK; `cargo clippy -p cam-camera-app
--all-features -- -D warnings` OK; `cargo test -p cam-isp --lib --features
"rectifier mnn" --no-fail-fast` → 845 passed, the same 2 pre-existing failures,
2 ignored; `cargo test -p cam-camera-app --lib --all-features` → 46 passed.
Device end-to-end AHB is still blocked (camera AImage lifetime, no output AHB
from ANativeWindow); the APK remains on the packed BGRA readback path.
