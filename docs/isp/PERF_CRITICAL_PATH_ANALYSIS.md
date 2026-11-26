# SoftISP Per-Frame Critical-Duration Analysis (ISP input → MNN inference → SurfaceFlinger)

## Goal

Isolate the critical path per frame (`camera fill → prepare/upload → runSession → output handoff → present`) and rank every remaining optimization by expected payoff at 4K. This is the engineering counterpart to the roadmap backlog in `docs/overview/ROADMAP.md` (rows #7/#12/#13).

---

## 1. Measured phase split (per `FramePhaseTimings`)

`cam-rust/cam-isp/src/pipeline/types.rs:31-71` defines `FramePhaseTimings`, already populated by `MnnEngine::run_prepared()` and surfaced via `IspFrame.aux.timing`. `app.rs:render_once()` additionally measures `capture`, `stage`, and `present` around the engine.

| Phase | Where measured | What it pays | On the visible critical path? |
|-------|---------------|--------------|------------------------------|
| `capture` | `app.rs:render_once()` | `AImageReader_acquireLatestImage` + `copy_pixels()` de-stride into `Vec<u16>` | **Yes** (serialized behind camera mutex) |
| `stage` | `app.rs:render_once()` | `FrameStager::stage()` — u16 → LE byte widening | **Yes** (CPU per-pixel loop) |
| `setup` | `mnnengine.rs:prepare()` | Controller snapshot + session slot acquire | Yes (but small) |
| `tensor_upload` | `mnnengine.rs:prepare()` / `run_prepared()` | ~25 `copyFromHostTensor` extra-input flushes (CCM, tone, gains, …) | **Yes** (host→device, one barrier each) |
| `inference` | `mnnengine.rs:run_prepared()` | MNN `runSession` — GPU dispatch (`runSession`) | **Yes** (irreducible floor; Vulkan compute) |
| `output_meta` | `mnnengine.rs:run_prepared()` | Output shape/type/layout derivation + `CString` names | Yes (small) |
| `output_copy` | `mnnengine.rs:run_prepared()` | `f32→u8` narrowing or `to_vec()`; skipped for zero-copy | **Yes** for non-zero-copy |
| `present` | `app.rs:render_once()` | `WindowTarget::present()` CPU BGRA→RGBA blit OR `AhbDisplay::present_ahb_raw()` ASurfaceTransaction | **Yes** |
| `readback` | `MnnEngine::finish()` | Deferred 3A stats readback (`copyToHostTensor`) | **No** — staged after present |

`FramePhaseTimings.visible_ns()` = prefill + inference + output (excludes readback). `total_ns()` adds readback. `app.rs` reports `capture + stage + isp + present` per frame against `FRAME_BUDGET`.

---

## 2. Remaining optimizations ranked by impact (4K)

| # | Optimization | Phase attacked | Where | Status / effort | Notes |
|---|-------------|----------------|-------|-----------------|-------|
| 1 | Camera: pass `AImage_getHardwareBuffer` → `input_ahb`, skip `copy_pixels()` de-stride | capture | `anw/src/camera.rs`, `anw/src/pipeline_runner.rs` | **Landed** (input AHB bound before resize; `copyFromHostTensor` skipped in `mnn_wrapper.cpp`) | `RawFrame::ahb` now carries `AHardwareBuffer*`; `prepare()` imports it before `resizeSession`; `mnn_run_true_zero_copy`/`mnn_run_with_output` skip `copyFromHostTensor` when `flags == MNN_MEMORY_AHARDWAREBUFFER`. `FrameStager::stage()` still runs but its result is unused when `input_ahb` is set — could skip widening entirely. |
| 2 | Presentation: hand AHB straight to SurfaceFlinger via `ASurfaceTransaction`, skip CPU BGRA→RGBA blit | present | `anw/src/ahb_display.rs`, `anw/src/app.rs` | **Landed** (Path B `AhbDisplay` dlsyms `ASurfaceControl_*` from `libnativewindow.so`; best-effort, falls back to `WindowTarget::present`) | Eliminates the per-pixel host blit on the visible path. Falls back gracefully on older devices. |
| 3 | Skip `FrameStager::stage()` widening when `input_ahb` is set | stage | `anw/src/pipeline_runner.rs` | **Not done** | `stager.stage(&frame)` always runs its `u16 → LE` loop even when `input_ahb` feeds MNN directly. Guard on `frame.ahb.is_some()` to avoid the O(H×W) CPU pass. |
| 4 | Batch extra-input uploads into single const buffer + `MNNVulkanHotSwapConstBuffer` | tensor_upload | `mnnengine.rs:set_extra_inputs()` + `mnn_wrapper.cpp` | **Not done** | ~25 per-frame `copyFromHostTensor` calls, each a host→device barrier. Combine scalars into one `float[N]` tensor uploaded once, shader indexes into it. Requires graph + shader change. |
| 5 | Skip unchanged extra-input uploads (delta tracking) | tensor_upload | `mnnengine.rs:set_extra_inputs()` | **Done** (off by default) | `extra_input_cache` dedups unchanged scalars; `SOFTISP_CACHE_ON` re-enables after blue-frame report. |
| 6 | Remove CPU lane-split scratch (SplitInt32/SplitFloat) — do split in Vulkan shader | stage (or inference) | `mnnengine.rs:1934-2007` + shader | **Not done** | O(H×W) CPU loop + mutex per frame for non-RawInt modes. Shader-side even/odd split removes it entirely. |
| 7 | Wire `OutputBufferPool` into non-zero-copy output path | output_copy | `mnn_buffer/mod.rs:OutputBufferPool` + `mnnengine.rs` | **Not done** | Pool exists (`#[allow(dead_code)]`) but not wired into fallback path; eliminates per-frame `Vec` allocation (~33 MB at 4K). |
| 8 | Resize/warm every session slot once at startup | inference (first frame) | `mnn_session_pool.rs:SessionSlot.resized_for` | **Not done** | First-frame resize stalls when switching preview↔capture; pre-resize common resolutions avoids it. |
| 9 | Ping-pong output AHB of frame N into input of frame N+2 behind fences | capture + inference | `anw/src/ahb_output.rs` + `cam-ipa::fence.rs` | **Blocked** (needs real sync fd lifecycle; acquire fd not yet waited by Vulkan path) | Both copies on GPU; currently CPU `AtomicBool` busy flag only. |
| 10 | Lower preview resolution / fp16 storage / lighter profile | inference (`runSession`) | `mnnengine.rs`, `PipelineProfile` | **Not done** (algorithmic) | Largest lever on `runSession` itself but changes image quality / memory. |

---

## 3. What is still NOT zero-copy (end-to-end audit)

| Hop | Mechanism | Zero-copy? | Gap |
|-----|-----------|------------|-----|
| Camera → ISP input | `AImage_getHardwareBuffer` → `ProcessParams::input_ahb` → MNN `setDevicePtr(..., MNN_MEMORY_AHARDWAREBUFFER)` → Vulkan `onAcquire` (`VulkanBufferFromAHB`) | **Yes** (when `frame.ahb` is set) | `copy_pixels()` still runs but its output is unused; `FrameStager::stage()` still widens — skip both when `input_ahb` active |
| MNN input → GPU | Vulkan `onAcquire` imports AHB as `VkDeviceMemory`; `copyFromHostTensor` skipped in `mnn_wrapper.cpp` | **Yes** | None |
| GPU compute | MNN Vulkan backend `runSession` | N/A (compute, not copy) | N/A |
| GPU → Display output | `mnn_run_ahb_output()` writes into caller AHB; `AhbDisplay::present_ahb_raw()` → `ASurfaceTransaction` → SurfaceFlinger | **Yes** (Path B) | Falls back to `WindowTarget::present()` CPU blit on older devices |
| 3A stats readback | `copyToHostTensor` deferred to `finish()` | **No** (host readback stall) | Staged after present; excluded from `visible_ns()` |

---

## 4. Recommended next steps

1. **Skip `FrameStager::stage()` when `input_ahb` is set** — `anw/src/pipeline_runner.rs:316`. Saves the O(H×W) CPU widen loop on every zero-copy frame.
2. **Batch extra-input uploads** — `mnnengine.rs:set_extra_inputs()` → single const-buffer upload via `MNNVulkanHotSwapConstBuffer`. Removes ~25 host→device barriers per frame (largest remaining tensor_upload cost).
3. **Wire `OutputBufferPool` into the non-zero-copy fallback** — eliminates ~33 MB per-frame `Vec` allocation at 4K for the host-copy path.
4. **Real sync fences** — replace the CPU `AtomicBool` busy flag in `anw/src/ahb_output.rs` with `cam-ipa::fence.rs` `SyncFence`/`AhbFenceState` (camera acquire fd → Vulkan wait; GPU completion → display release fd). Unblocks row #9 (ping-pong AHB).

---

## 5. How to verify

- `cargo fmt --all -- --check` — clean.
- `cargo clippy -p softisp-anw --all-features --no-deps -- -D warnings` — clean.
- `cargo clippy -p cam-isp --lib --features rectifier -- -D warnings` — clean.
- `cargo test -p cam-isp --lib --features rectifier --no-fail-fast` — 862 passed, 0 failed, 2 ignored.
- On-device: `SOFTISP_VERBOSE` / `/sdcard/download/softisp-verbose` logs per-slot AHB handoff; confirm no `AHB input import ... failed` / `-10` fallback and no black frames.
- Per-phase timing is visible in `IspFrame.aux.timing` (`setup_ns`, `tensor_upload_ns`, `inference_ns`, `output_meta_ns`, `output_copy_ns`, `readback_ns`) and in `app.rs` frame-budget logs (`capture / stage / isp / present ms`).
