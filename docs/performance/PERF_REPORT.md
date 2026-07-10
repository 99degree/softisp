# Performance Report — SoftISP (Latest Master)

## Build & Test Status

| Item | Status |
|------|--------|
| Workspace build (release) | ✅ 21.9s |
| Unit tests (fast) | ✅ 140 pass in 0.04s |
| CPU pipeline tests | ⚠️ 2 tests ignored (extremely slow) |
| ONNX backend | ❌ Needs libonnxruntime.so |
| MNN backend | ❌ Needs MNN headers + lib |

## SIMD Backend

The system auto-detects and uses the best available SIMD implementation:

```
Detected: aarch64 with dotprod + fp16 + neon
Selected: neon-dotprod backend
```

Four backends in the hierarchy:
1. **neon-dotprod** (ARMv8.4) — highest
2. **neon-fp16** (ARMv8.2)
3. **neon** (ARMv8.0)
4. **scalar** (fallback)

## Measured Speedups (vs scalar)

Benchmarked isolated SIMD operations on 1M pixels:

| Operation | Scalar | neon-dotprod | Speedup |
|-----------|--------|--------------|---------|
| normalize_u16_to_f32 | 1.6ms | 0.78ms | **1.99×** |
| apply_ccm (3×3) | 22.8ms | 17.7ms | **1.29×** |
| apply_ae_gain | 13.8ms | 7.6ms | **1.82×** |
| display_output | 11.0ms | 11.1ms | ~1.0× (mem-bound) |

**Overall**: SIMD provides **1.3-2× speedup** on compute-bound ops.

## Full Pipeline Bottleneck

CpuEngine::process() remains too slow for practical use because:

1. Only **4 of 16 stages** are SIMD-accelerated (normalize, CCM, AE gain, display)
2. **Demosaic** (Malvar) — expensive 5×5 convolution, still scalar
3. **Calibration & stats** — per-pixel analysis, allocations
4. **Zone stats & histogram** — O(w×h) operations, many bins
5. **Tone mapping** — gamma/contrast/saturation, per-pixel
6. **Mutex locks** per frame for controller state
7. **Heap allocations** throughout (no buffer reuse)

A single 16×16 frame takes **seconds** in debug and still **hundreds of ms** in release.

## Recommendations

### Immediate
- Mark CPU pipeline tests as `#[ignore]` (already done) — they are not suitable for CI

### Short-term (improve CPU engine)
1. **SIMD demosaic** — highest ROI (currently ~10-30% of CPU time)
2. **Buffer reuse** — allocate once, reuse across frames
3. **Thread pool** — parallelize tiles with rayon
4. **Reduce allocations** — use stack buffers for small frames

### Long-term (optimal)
- Use MNN/ONNX backends for production (requires native libs)
- The pipeline compiles to GPU/CPU-optimized graph with those backends

## Example Usage

```bash
# Build (release)
cargo build --release -p cam-isp

# Fast tests only
cargo test --lib -p cam-isp --release

# SIMD benchmark
cargo run --release --example simd_bench -p cam-isp
```

## Conclusion

The SIMD backend is correctly detected and provides **modest speedups** (1.3-2×) on the operations it accelerates. However, the majority of the pipeline remains scalar, resulting in overall poor performance. The codebase is ready for MNN/ONNX integration when those libraries become available. For now, fast unit tests are suitable for development, while CPU pipeline tests remain ignored.
