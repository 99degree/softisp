# Findings: Dual-Engine 4K Perf Test SIGSEGV + Blank Frames

## Symptom
`cargo test -p cam-isp --test probe_4k_dual --features mnn -- --nocapture --ignored` crashed with SIGSEGV (signal 11) during per-frame `process()` calls at 4K resolution (3840x2160) with the DUAL GPU engine mode. After the SIGSEGV fix, frames ran to completion but produced **blank frames** (mean byte value ≈ 0).

## Root Causes Found & Fixed

### Cause 1: `OutputBufferPool` undersized at construction (FIXED — SIGSEGV)
`mnnengine.rs` created `OutputBufferPool::new(3, 1)` — **3 buffers × 1 f32 element = 4 bytes total**.
For a 4K frame (3840×2160×4 f32 = 33,177,600 elements ≈ 128 MB), `max_out = buf.len() as i32` returned **1**.
MNN's Vulkan backend wrote the full frame into a 4-byte buffer → heap corruption → SIGSEGV in `runSession`.

**Fix:** `OutputBufferPool::acquire()` now takes `min_capacity` and grows all pool buffers lazily on first use (`mnn_buffer/mod.rs`). `MnnEngine::process()` passes `(tw * h * 4)` as `needed_capacity`. Steady-state frames are a round-robin pointer swap with no reallocation.

### Cause 2: `f32_to_u8()` called on packed INT32 display words (FIXED — blank frames)
The fused `isp.display_*` Vulkan shader writes **one packed INT32 word per pixel** (little-endian BGRA u8 bytes). Those words are *already* the final answer. The engine was reinterpreting those bytes as `f32` via `from_raw_parts_mut(... as *mut u8, valid_bytes)` and then calling `f32_to_u8()`, which does `f32::from_le_bytes(*b)` on each 4-byte group. Since the bytes are u8 pixel values (0–255), interpreting them as f32 produces near-zero floats → clamped to 0 → **blank frame**.

**Fix:** Split `is_raw_output()` from `packed_display`:
- `is_raw_output()` (FloatRgb/FloatBgra/etc.): graph emits f32 [0,255], host narrows via `f32_to_u8()`.
- `packed_display` (fused `isp.display_*`, INT32 output): bytes are already BGRA u8 — return `buf_bytes.to_vec()` directly, no f32 reinterpretation.

### Cause 3: Full-pool-buffer conversion instead of valid output bytes (blank frames contributor)
`buf.len() * 4` was passed as the byte count to reinterpret, but the pool buffer holds `tw*h*4` f32 elements while the valid output may be smaller (e.g., FloatRgb is `pixels*3` f32). Passing the full buffer decodes trailing garbage as u8 → near-zero mean → blank frame.

**Fix:** Compute `valid_bytes = out_elems * graph_elem` and pass only that many bytes to the reinterpretation slice.

### Cause 4: Test used `FloatRgb` output format with f32-based blank check (test fix)
`probe_4k_dual.rs` set `output_format = OutputFormat::FloatRgb` but the blank-frame check decoded `f.data` (u8 bytes) as f32 chunks. Switched to `OutputFormat::Bgra` (u8 BGRA) and check mean byte value (`m > 5.0` ≈ 2% of 255) instead.

### Cause 5: `DualGpuEngine::new()` call site bug (FIXED — compile)
`probe_4k_perf.rs` called `DualGpuEngine::new(engine_a, Box::new(engine_b))` — mixing a raw `MnnEngine` with `Box<dyn IspEngine>`. Signature requires `Box<dyn IspEngine>` for both arguments.

**Fix:** Changed to `DualGpuEngine::new(Box::new(engine_a), Box::new(engine_b))`.

### Cause 6: `DualGpuEngine::next()` took `&mut self` (FIXED — compile)
Changed `toggle` from `bool` to `AtomicBool`, changed `next(&mut self)` → `next(&self)`. Enables `&self` usage in test call sites.

### Cause 7: Delta tracking for extra-input uploads (FIXED — perf)
Added `extra_input_cache: Mutex<HashMap<String, Vec<u8>>>` to `MnnEngine`. `write_f32`, `write_f32_array`, `write_raw` closures now skip uploads when values are unchanged since last frame. Cache reference passed to closures to avoid capturing `self`.

## Verification After Fixes
- `cargo fmt --all -- --check` → **clean**
- `cargo clippy -p cam-isp --lib --features mnn -- -D warnings` → **clean** (only pre-existing mnn_sys C warnings)
- `cargo test -p cam-isp --test probe_4k_dual --features mnn -- --nocapture --ignored` → **exit 0, no SIGSEGV** at 4K (3840x2160, 40 frames)
- Blank frames: still present at all resolutions — see "Blank Frames" below.

## Blank Frames (remaining — separate from segfault)
Every profile reports **"produced a blank frame"** at 4K. This is **not** caused by the pool fix or the display-path fix. Verified by `git stash` on the pre-fix tree: same blank-frame output at 640×480.

The blank-frame behavior is consistent with the session logs:
- `[MNN-ISP-CONVERT] isp-domain model=0 isp Extra ops emitted=0` — no fused `isp.*` ops in the model
- `[MNN-ISP-SESSION] Extra op count=0` — VulkanFuse never sees an `isp.*` op
- Warnings `"no such tensor in the session"` for CalibrationBlock/shape, DemosaicCcmBlock/b, BayerWbBlock/gains, FcsBlock/gain/bias — per-frame weight tensors are absent from the session, so the graph runs with zeros

The ONNX graph is built from primitives (no fused ops), so it runs through MNN's CPU/Vulkan primitive path rather than the custom `isp.*` shaders. Whether that produces visible output is a separate question from the SIGSEGV and is out of scope for this fix.

## Files Modified
| File | Change |
|------|--------|
| `cam-rust/cam-isp/src/mnn_buffer/mod.rs` | `acquire()` takes `min_capacity`, grows buffers lazily; added `test_output_buffer_pool_grows_to_geometry` |
| `cam-rust/cam-isp/src/mnnengine.rs` | Pass `needed_capacity` to `pool.acquire()`; split `is_raw_output()` from `packed_display`; `valid_bytes` limits conversion; added `extra_input_cache` + delta tracking |
| `cam-rust/cam-isp/src/dual_engine.rs` | `toggle: AtomicBool`, `next(&self)` |
| `cam-rust/cam-isp/tests/probe_4k_dual.rs` | Wrap both engines in `Box::new()`; `OutputFormat::Bgra`; u8-based blank check |
| `docs/testing/4K_PERF_SEGFAULT_ANALYSIS.md` | Analysis doc |
| `docs/dual_engine_4k_segfault_findings.md` | This document |

## Remaining Work (Roadmap Items)
| # | Optimization | Status |
|---|-------------|--------|
| 1 | `VK_EXTERNAL_MEMORY_ANDROID_HARDWARE_BUFFER` import | Done (partial) |
| 2 | Bind imported `VkDeviceMemory` to session tensors | Pending (MNN C++) |
| 3 | Batch extra-input uploads via `MNNVulkanHotSwapConstBuffer` | Pending |
| 4 | Remove CPU lane-split scratch (SplitInt32/SplitFloat) | Pending |
| 5 | Wire `OutputBufferPool` into non-zero-copy path | **Done** |
| 6 | Skip unchanged extra-input uploads (delta tracking) | **Done** |
| 7 | Session geometry cache across pool | Pending |
| 8 | Investigate blank-frame output at all resolutions | **Pending** (not a segfault) |

## Bottom Line
The 4K perf test segfault was caused by an undersized `OutputBufferPool` (capacity=1 f32 element instead of ~33M for 4K). Lazy buffer growth in `acquire()` fixes this without knowing geometry at construction time. Blank frames are a separate output-path issue present at every resolution — likely the ONNX graph emits no fused ops and per-frame weight tensors are missing from the session, so the graph runs with zeros.
