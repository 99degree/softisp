# 4K Perf Test Segfault Analysis

## Summary
The `probe_4k_perf` integration test **segfaulted (SIGSEGV, signal 11)** during per-frame `process()` calls at 4K resolution (3840x2160) with the DUAL GPU engine mode. Multiple root causes were identified and fixed. **The segfault is resolved: the 4K test now runs to completion with exit 0.** (Frames are blank at all resolutions — see "Blank frames (separate, pre-existing issue)" below.)

## Root Causes Found & Fixed

### Cause 1: `OutputBufferPool` undersized at construction (FIXED)
`mnnengine.rs` created `OutputBufferPool::new(3, 1)` — **3 buffers × 1 f32 element = 4 bytes total**.
For a 4K frame (3840×2160×4 f32 = 33,177,600 elements ≈ 128 MB), `max_out = buf.len() as i32` returned **1**.
MNN's Vulkan backend wrote the full frame into a 4-byte buffer → heap corruption → SIGSEGV.

**Fix:** `OutputBufferPool::acquire()` now takes `min_capacity` and grows all buffers lazily on first use (`mnn_buffer/mod.rs`). `MnnEngine::process()` passes `(tw * h * 4)` as `needed_capacity` (`mnnengine.rs`).

### Cause 2: `DualGpuEngine::new()` call site bug (FIXED)
`probe_4k_perf.rs:198` called `DualGpuEngine::new(engine_a, Box::new(engine_b))` — mixing a raw `MnnEngine` with `Box<dyn IspEngine>`. Signature requires `Box<dyn IspEngine>` for both arguments.

**Fix:** Changed to `DualGpuEngine::new(Box::new(engine_a), Box::new(engine_b))`.

### Cause 3: `DualGpuEngine::next()` took `&mut self` (FIXED)
Changed `toggle` from `bool` to `AtomicBool`, changed `next(&mut self)` → `next(&self)`. Enables `&self` usage in test call sites.

### Cause 4: Delta tracking for extra-input uploads (FIXED)
Added `extra_input_cache: Mutex<HashMap<String, Vec<u8>>>` to `MnnEngine`. `write_f32`, `write_f32_array`, `write_raw` closures now skip uploads when values are unchanged since last frame. Cache reference passed to closures to avoid capturing `self`.

## Verification After Fixes
- `cargo test --lib -p cam-isp --no-fail-fast` → **813 passed**
- `cargo clippy -p cam-isp --lib --features rectifier -- -D warnings` → **clean**
- `cargo fmt --all -- --check` → **clean**
- `cargo check -p cam-isp --lib --features rectifier` → **clean**
- `PERF_FRAMES=3 PERF_W=640 PERF_H=480 probe_4k_perf` → **2 passed, no segfault**
- `PERF_FRAMES=2 PERF_W=3840 PERF_H=2160 probe_4k_perf` → **2 passed, no segfault** (previously crashed at first frame with SIGSEGV)

## Blank Frames (separate, pre-existing issue — NOT a segfault)

Every profile reports **"produced a blank frame"** at 4K (and at 640×480 too). This is **not** caused by the pool fix. Verified by `git stash` on the pre-fix tree: same blank-frame output at 640×480 with exit 101 (SIGSEGV) — so blank frames exist independently of the crash.

The blank-frame behavior is consistent with the session logs:
- `[MNN-ISP-CONVERT] isp-domain model=0 isp Extra ops emitted=0` — no fused `isp.*` ops in the model
- `[MNN-ISP-SESSION] Extra op count=0` — VulkanFuse never sees an isp.* op
- Warnings `"no such tensor in the session"` for CalibrationBlock/shape, DemosaicCcmBlock/b, BayerWbBlock/gains, FcsBlock/gain/bias — per-frame weight tensors are absent from the session, so the graph runs with zeros

The ONNX graph is built from primitives (no fused ops), so it runs through MNN's CPU/Vulkan primitive path rather than the custom `isp.*` shaders. Whether that produces visible output is a separate question from the SIGSEGV and is out of scope for this fix.

## Files Modified
| File | Change |
|------|--------|
| `cam-rust/cam-isp/src/mnn_buffer/mod.rs` | `acquire()` takes `min_capacity`, grows buffers lazily; added `test_output_buffer_pool_grows_to_geometry` |
| `cam-rust/cam-isp/src/mnnengine.rs` | Pass `needed_capacity` to `pool.acquire()` |
| `cam-rust/cam-isp/src/dual_engine.rs` | `toggle: AtomicBool`, `next(&self)` |
| `cam-rust/cam-isp/tests/probe_4k_perf.rs` | Wrap both engines in `Box::new()` |
| `docs/testing/4K_PERF_SEGFAULT_ANALYSIS.md` | This document |

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
| 8 | Investigate blank-frame output at all resolutions | Pending (not a segfault) |

## Bottom Line

The 4K perf test segfault was caused by an undersized `OutputBufferPool` (capacity=1 f32 element instead of ~33M for 4K). Lazy buffer growth in `acquire()` fixes this without knowing geometry at construction time. The test now **compiles and runs** at 4K with **no SIGSEGV**. Blank frames remain — a separate output path issue unrelated to the crash, present at every resolution.