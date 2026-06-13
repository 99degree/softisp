# SIMD Backend Performance

## System
- **Architecture**: aarch64 (ARM64)
- **Detected features**: `dotprod`, `fp16`, `neon`
- **Selected backend**: `neon-dotprod` (best available)

## Benchmark Results

### 1. normalize_u16_to_f32
Convert 1M u16 pixels to f32 [0,1]
- Scalar: 1.6ms (644 ops/s)
- **neon-dotprod: 0.78ms (1,275 ops/s)**
- **Speedup: 1.99×**

### 2. apply_ccm (3×3 matrix)
- Scalar: 22.8ms (43.9 ops/s)
- **neon-dotprod: 17.7ms (56.5 ops/s)**
- **Speedup: 1.29×**

### 3. apply_ae_gain (multiply + clamp)
- Scalar: 13.8ms (72.7 ops/s)
- **neon-dotprod: 7.6ms (131 ops/s)**
- **Speedup: 1.82×**

### 4. display_output (RGB → BGRA)
- Scalar: 11.0ms (90.8 ops/s)
- **neon-dotprod: 11.1ms (90.4 ops/s)**
- **Speedup: ~1.0× (no gain)** (likely memory-bound)

## Analysis

The SIMD optimizations provide **meaningful speedups** (1.3× to 2×) on compute-intensive operations. However, the full CPU pipeline remains slow due to:

1. **Other expensive stages**: demosaic, calibration, stats, zone calculations
2. **Memory bandwidth**: display_output is memory-bound; SIMD doesn't help
3. **Mutex contention**: controller state uses locks per frame
4. **Allocation churn**: many temporary Vec allocations (no buffer reuse)

## Bottlenecks (from CpuEngine::process)

| Stage | Est. Cost | SIMD'd? |
|-------|-----------|---------|
| normalize_u16_to_f32 | Low | ✅ Yes |
| Demosaic (Malvar) | **High** | ❌ No |
| Calibration stats | High | ❌ No |
| Zone stats | Medium | ❌ No |
| apply_ccm | Medium-low | ✅ Yes |
| apply_ae_gain | Medium-low | ✅ Yes |
| Tone mapping | High | ❌ (partially?) |
| display_output | Medium | ✅ Yes (but memory bound) |

## Recommendations

- **Prioritize SIMD for demosaic** — biggest impact potential (currently pure scalar)
- **Reuse buffers** — avoid per-frame allocations in hot path
- **Parallelize tiles** — process image regions in parallel (rayon)
- **Profile with perf** — identify actual hotspots on device

## Testing

Run the benchmark:
```bash
cargo build --release --example simd_bench -p cam-isp
cargo run --release --example simd_bench -p cam-isp
```

Expected output: `neon-dotprod` backend with speedup numbers as above.
