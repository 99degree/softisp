# cam-isp Architecture Improvements

## Recently Fixed

| # | Issue | Fix |
|---|-------|-----|
| 1 | **HDR merge hardcoded ARGB8888** — `merge_three_exp`, `merge_two_exp`, `merge_n_frames` used `frame.data[base+1/2/3]` directly; would silently corrupt on FloatRgb input | Added `pixel_rgb()` helper that checks `frame.float_data` (NCHW planar f32) first, falls back to ARGB8888 bytes. Merge functions now use `pixel_rgb()` / `write_argb()` helpers. |
| 2 | **`submit_hdr_frame` consumed the frame** — took `IspFrame` by value, caller lost the processed frame for display while waiting for burst completion | Changed to `&IspFrame`, clones internally. Caller retains their copy. |
| 3 | **Doc warnings** — 15 `rustdoc::broken_intra_doc_links` warnings from tensor shapes `[1,3,H,W]` being parsed as link references | Added `#![allow(rustdoc::broken_intra_doc_links)]` + escaped brackets in warp_engine.rs |
| 4 | **Build warnings** — 3 unused variables/imports | Removed dead `LUT_MASK`, unused `warn!` import, unused `_n` |
| 5 | **HDR EV passed but ignored** — `_ev: f32` was received but frames always got `ev: 0.0` | Stored EV in `HdrFrame.ev`, sorted by EV before submitting |

## Near-term (High Impact, Low Risk)

### 1. Extract `SessionPool` from `mnnengine.rs` (1504 → ~module)
`mnnengine.rs` contains three concerns in one file:
- `SessionPool` (session reuse, acquire/release, multi-slot)
- `MnnEngine` (build/inference orchestration, ~700 lines)
- `bench_one` / `norm` (benchmarking helpers)

**Plan:** Move `SessionPool` (lines 79-165) and `SessionGuard` into `mnn_session_pool.rs`. Keep `MnnEngine` and `IspEngine` impl in `mnnengine.rs`. Remove `bench_one` if unused.

**Files:** `cam-isp/src/mnn_session_pool.rs` (new), `cam-isp/src/mnnengine.rs` (simplified)

### 2. Extract `build_blocks()` from `profile.rs` (1047 → ~module)
`profile.rs` mixes profile constants (7 static `PipelineProfile` values) with the `build_blocks()` method (~500 lines of block construction + wiring logic).

**Plan:** Move `build_blocks()` and `build_aux_blocks()` into `profile_builder.rs`. Profile constants stay in `profile.rs`.

**Files:** `cam-isp/src/profile_builder.rs` (new)

### 3. FloatRgb path for HDRWorker encoding
The `encode()` function writes PPM with dimensions derived from pixel count (`sqrt` heuristic), which is wrong for non-square frames. Should use `IspFrame.width` / `height` from the HDR frame metadata.

**Plan:** Change `encode()` to accept `width`/`height` from the merged frame dimensions, remove the sqrt heuristic.

### 4. MNN converter temp file hardening
`mkstemp`-based temp files in `mnn_convert_api.cpp` can leak if the converter process crashes mid-conversion.

**Plan:** Add `atexit` cleanup handler + signal-safe temp file tracking. Or switch to `memfd_create` for non-Android platforms.

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

| Metric | Current | Target |
|--------|---------|--------|
| Build warnings | 0 | 0 |
| Doc warnings | 0 | 0 |
| Test pass rate | 734/734 | 734/734 |
| HDR merge latency (FHD) | ~5ms CPU | <1ms GPU |
| HDR align latency | 0ms (noop) | <3ms (optical flow) |
| mnnengine.rs lines | 1504 | <800 |
| controller.rs lines | 1293 | <400 per module |
| profile.rs lines | 1047 | <300 + builder |
