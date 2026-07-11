# cam-isp Architecture Improvements

## Recently Fixed

| # | Issue | Fix |
|---|-------|-----|
| 1 | **HDR merge hardcoded ARGB8888** — `merge_three_exp`, `merge_two_exp`, `merge_n_frames` used `frame.data[base+1/2/3]` directly; would silently corrupt on FloatRgb input | Added `pixel_rgb()` helper that checks `frame.float_data` (NCHW planar f32) first, falls back to ARGB8888 bytes. Merge functions now use `pixel_rgb()` / `write_argb()` helpers. |
| 2 | **`submit_hdr_frame` consumed the frame** — took `IspFrame` by value, caller lost the processed frame for display while waiting for burst completion | Changed to `&IspFrame`, clones internally. Caller retains their copy. |
| 3 | **Doc warnings** — 15 `rustdoc::broken_intra_doc_links` warnings from tensor shapes `[1,3,H,W]` being parsed as link references | Added `#![allow(rustdoc::broken_intra_doc_links)]` + escaped brackets in warp_engine.rs |
| 4 | **Build warnings** — 3 unused variables/imports | Removed dead `LUT_MASK`, unused `warn!` import, unused `_n` |
| 5 | **HDR EV passed but ignored** — `_ev: f32` was received but frames always got `ev: 0.0` | Stored EV in `HdrFrame.ev`, sorted by EV before submitting |
| 6 | **SessionPool monolithic in mnnengine.rs** (1504 lines) | Extracted SessionPool/SessionGuard/SessionSlot into `mnn_session_pool.rs` (155 lines). mnnengine.rs reduced to 1390 lines. |
| 7 | **Profile builder monolithic in profile.rs** (1047 lines) | Extracted `build_blocks()`, `build_aux_blocks()`, `block_count()`, `node_estimate()` into `profile_builder.rs` (306 lines). profile.rs reduced to 680 lines. |
| 8 | **HDR PPM encode wrong dimensions** — sqrt heuristic for non-square frames | `encode()` now takes `width`/`height` from frame metadata. |
| 9 | **MNN converter temp file leak risk** — `mkstemp` files could leak if process crashes mid-conversion | RAII `TempFileGuard` class auto-unlinks on destruction. Global registry with `atexit` + signal handlers (SIGINT/SIGTERM/SIGABRT). Thread-safe. |

## Near-term (High Impact, Low Risk) — Complete ✓

All 4 near-term items completed:

1. ✅ **SessionPool extraction** — `mnn_session_pool.rs` with `SessionPool`, `SessionGuard`, `SessionSlot`. Same interface, smaller file.
2. ✅ **Profile builder extraction** — `profile_builder.rs` with `build_blocks()`, `build_aux_blocks()`, `block_count()`, `node_estimate()`.
3. ✅ **HDR encode dimension fix** — `encode()` now accepts `width`/`height` from frame metadata, no sqrt heuristic.
4. ✅ **MNN converter temp file hardening** — RAII `TempFileGuard` + global registry + signal handlers.

## Medium-term (Architecture)

### 5. Split `IspController` (1293 lines) into domain modules
`controller.rs` contains AE, AWB, AF, CCM, tone all in one impl struct. The `ControllerApi` trait abstraction is correct but the concrete implementation is monolithic.

**Plan:**
- `controller/ae.rs` — Auto-exposure
- `controller/awb.rs` — White balance
- `controller/af.rs` — Auto-focus  
- `controller/ccm.rs` — Color correction
- `controller/tone.rs` — Tone mapping
- `controller/mod.rs` — IspController shell that delegates

### 6. Engine registry as proper plugin system
Currently `engine.rs` has a static `ENGINE_REGISTRY` using `Vec<Box<dyn Fn() -> Box<dyn IspEngine>>>`. Priorities are hardcoded.

**Plan:** Replace with a priority-ordered `BTreeMap<i32, Box<dyn Fn() -> ...>>` that allows runtime registration/unregistration. Each engine provides a `priority()` method instead of the global list.

### 7. Unified error type consolidation
There are multiple error enums: `IspError`, `HdrError`, `String`-based errors from MNN FFI, `Result<(), String>` in places.

**Plan:** Make `HdrError` convertible to `IspError` via `From` impl. Move MNN FFI errors to typed variants in `IspError`. Remove `String`-based error returns from public API.

## Long-term (Research)

### 8. GPU-accelerated HDR merge via MNN
Current HDR merge is CPU-based (Mertens luminance weighting). Plan to:
- Build an ONNX model from `HdrMergeBlock` + `HdrToneBlock`
- Convert to MNN via the same buffer-based converter
- Run on GPU (Vulkan) for 10-20× speedup

### 9. Optical flow alignment for multi-exposure
Replace identity `align_frames()` with:
- MNN-based optical flow (e.g., RAFT or FlowNet)
- EIS motion vector-based homography warp
- Handles ghosting in moving scenes

### 10. Neural HDR merge (Kalantari et al.)
Train a lightweight CNN for multi-exposure fusion:
- Input: 3 exposures (under/neutral/over) as 4×4 Bayer blocks
- Output: HDR RGB
- Integrate as optional `merge_method: NeuralHdr`

## Metrics

| Metric | Before | After | Target |
|--------|--------|-------|--------|
| Build warnings | 0 | 0 | 0 |
| Doc warnings | 0 | 0 | 0 |
| Test pass rate | 734/734 | 734/734 | 734/734 |
| HDR merge latency (FHD) | ~5ms CPU | ~5ms CPU | <1ms GPU |
| HDR align latency | 0ms (noop) | 0ms (noop) | <3ms (optical flow) |
| mnnengine.rs lines | 1504 | 1390 | <800 |
| mnn_session_pool.rs lines | — | 155 (new) | — |
| controller.rs lines | 1293 | 1293 | <400 per module |
| profile.rs lines | 1047 | 680 | <300 + builder |
| profile_builder.rs lines | — | 306 (new) | — |
| hdr.rs lines | 756 | 747 | <500 |
| mnn_convert_api.cpp temp safety | manual unlink | RAII + atexit + signals | no leaks |
