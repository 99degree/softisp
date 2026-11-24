# 4K Perf Test Segfault Analysis

## Summary
The `probe_4k_perf` integration test **segfaulted (SIGSEGV, signal 11)** during per-frame `process()` calls at 4K resolution (3840x2160) with the DUAL GPU engine mode. Multiple root causes were identified and fixed; one remaining runtime segfault persists at 4K.

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
- Compilation of `probe_4k_perf` test → **passes** (was failing before fix)

## Remaining Issue: 4K Runtime Segfault (NOT YET FIXED)

After all code fixes, the test **compiles and runs but still crashes with SIGSEGV at 4K** during frame processing. The crash occurs in `MnnEngine::process()` → `mnn_run_with_output()` → `net->runSession(sess)`.

### Evidence from crash log
1. Both engines build successfully (VulkanFuse ops accepted, sessions created)
2. Session logs show `tensorNumber=0` — **no input/output tensors registered** in the session
3. Warnings: `"no such tensor in the session"` for CalibrationBlock/shape, DemosaicCcmBlock/b, bayer_pattern tensors, BayerWbBlock/gains, FcsBlock/gain/bias
4. Warning: `"MISSING isp.* opset at convert"` — ONNX graph had isp-domain nodes but MNN model has 0 isp.* Extra ops
5. **SIGSEGV occurs during first `process()` call**, after engine build completes

### Hypotheses for Remaining Segfault

| Hypothesis | Likelihood | Evidence |
|------------|------------|----------|
| Session `tensorNumber=0` means `getSessionInput(sess, nullptr)` returns invalid/wrong tensor | High | Session log shows `tensorNumber=0`; `getSessionInput(nullptr)` returns first tensor alphabetically, which may be a weight tensor not the frame input |
| Vulkan backend can't handle 4K frame buffers (GPU memory limits) | Medium | 4K = 33M elements × 4 bytes = 128MB per buffer; Vulkan may fail allocation |
| Input tensor type mismatch: raw u16 data vs INT32 tensor expectation | Medium | `input_buffer` is None, so raw u16 buffer is used; tensor expects INT32 packed lanes |
| `refresh_tensors()` doesn't re-resolve handles after `resize()` | Low | Code explicitly calls `slot.refresh_tensors(interp)` after resize |

### Reproduction
```
cd cam-rust
PERF_FRAMES=10 PERF_W=3840 PERF_H=2160 cargo test -p cam-isp --test probe_4k_perf --features mnn -- --ignored --nocapture
```
Crashes at 4K with SIGSEGV during frame processing.

### Workaround for Testing
Run at lower resolution (e.g., PERF_W=1296 PERF_H=970) to test at FHD, or run single-engine (non-dual) mode to isolate whether the issue is dual-engine specific or affects all 4K inference.

### Files Modified
| File | Change |
|------|--------|
| `cam-rust/cam-isp/src/mnn_buffer/mod.rs` | `acquire()` takes `min_capacity`, grows buffers lazily |
| `cam-rust/cam-isp/src/mnnengine.rs` | Pass `needed_capacity` to `pool.acquire()`; add `extra_input_cache`; delta tracking in write_* closures; pass `cache` ref to set_extra_inputs |
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

## Bottom Line

The 4K perf test segfault was primarily caused by an undersized `OutputBufferPool` (capacity=1 f32 element instead of ~33M for 4K). Lazy buffer growth in `acquire()` fixes this. The test now **compiles and runs** but still crashes with SIGSEGV at 4K during frame processing — likely due to `tensorNumber=0` in the MNN session (no input/output tensors registered) or Vulkan backend limitations at 4K resolution. Further investigation needed at the MNN Vulkan backend level or by testing at lower resolutions.