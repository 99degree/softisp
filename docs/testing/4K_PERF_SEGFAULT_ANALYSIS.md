# 4K Perf Test Segfault Analysis

## Summary
The `probe_4k_perf` integration test **segfaulted (SIGSEGV, signal 11)** when running the DUAL GPU engine mode at 4K resolution (3840x2160). The root cause was the `OutputBufferPool` being created with capacity `1` f32 element (4 bytes), far too small for a 4K output frame (~132MB). MNN's Vulkan backend wrote past the end of this tiny buffer → heap corruption → segfault.

## Root Cause (Confirmed)

### Primary: `OutputBufferPool` undersized at construction

`mnnengine.rs:239` created `OutputBufferPool::new(3, 1)` — **3 buffers × 1 f32 element = 4 bytes total**.

For a 4K frame (3840×2160×4 f32 = 33,177,600 elements ≈ 128 MB), `max_out = buf.len() as i32` returned **1**, and `mnn_run_true_zero_copy()` / `mnn_run_with_output()` wrote the full frame into a 4-byte buffer → immediate heap corruption → SIGSEGV.

**The pool was sized once at `with_pool_size()` construction time**, before any frame geometry was known, using a placeholder capacity of `1`.

### Secondary: `DualGpuEngine::new()` call site bug

`probe_4k_perf.rs:198` called `DualGpuEngine::new(engine_a, Box::new(engine_b))` — mixing a raw `MnnEngine` with a `Box<dyn IspEngine>`. The signature requires `Box<dyn IspEngine>` for **both** arguments. This was a compile error, not a runtime bug, but it prevented the test from running at all.

## Fixes Applied

### Fix 1: `OutputBufferPool::acquire()` now grows lazily (`mnn_buffer/mod.rs`)

```rust
pub fn acquire(&mut self, min_capacity: usize) -> (usize, &mut [f32]) {
    // Grow all buffers if any is too small — do this once, up front.
    if min_capacity > self.buffers.first().map(|b| b.len()).unwrap_or(0) {
        ... // buf.resize(min_capacity, 0.0) for all buffers
    }
    ...
}
```

The first `acquire()` call at 4K grows all 3 buffers to `tw * h * 4` f32 elements (~33M elements = ~128MB total). Subsequent frames reuse without reallocation.

### Fix 2: `MnnEngine::process()` passes needed capacity (`mnnengine.rs`)

```rust
let needed_capacity = (tw * h * 4) as usize;
let mut pool = self.buf_pool.lock().unwrap();
let (buf_id, buf) = pool.acquire(needed_capacity);
let max_out = buf.len() as i32;
```

### Fix 3: `probe_4k_perf.rs` dual-engine call site fixed

Changed `DualGpuEngine::new(engine_a, Box::new(engine_b))` → `DualGpuEngine::new(Box::new(engine_a), Box::new(engine_b))`.

### Fix 4: `DualGpuEngine::next()` now takes `&self` instead of `&mut self`

Changed `toggle` from `bool` to `AtomicBool` so the engine can be borrowed immutably for alternation (enables `&self` usage in tests).

## Verification

After fixes:
- `cargo test --lib -p cam-isp --no-fail-fast` → **813 passed**
- `cargo clippy -p cam-isp --lib --features rectifier -- -D warnings` → **clean**
- `cargo fmt --all -- --check` → **clean**
- `PERF_FRAMES=10 cargo test -p cam-isp --test probe_4k_perf --features mnn -- --ignored --nocapture` → **2 passed, no segfault**

## Files Modified

| File | Change |
|------|--------|
| `cam-rust/cam-isp/src/mnn_buffer/mod.rs` | `acquire()` takes `min_capacity`, grows buffers lazily |
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

## Bottom Line

The 4K perf test segfault was caused by an undersized `OutputBufferPool` (capacity=1 f32 element instead of ~33M for 4K). Lazy buffer growth in `acquire()` fixes this without knowing geometry at construction time. The test now runs cleanly at 4K with no segfault.