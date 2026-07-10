# SoftISP Performance Benchmark Results

## Test Environment
- Platform: Android (Termux)
- CPU: ARM64 (Snapdragon 8 Gen 2)
- Backends: CPU, MNN Vulkan GPU
- Rust: 1.75+

## GPU Performance (Vulkan)

| Resolution | Latency | FPS | Model Size | Profile |
|------------|---------|-----|------------|---------|
| HD (1280×720) | 1.47 ms | **680** | 18 MB | UNIFIED |
| FHD (1920×1080) | 2.28 ms | **438** | 41 MB | UNIFIED |
| 4K→FHD (3840×2160 → 1920×1080) | 6.6 ms | **152** | 166 MB | UNIFIED |
| 4K→4K (3840×2160 → 3840×2160) | 23.7 ms | **42** | 166 MB | UNIFIED |
| 4K (3840×2160) | 12.87 ms | **78** | 166 MB | UNIFIED |

## Stress Test Results (30s duration)

| Test | Frames | Avg FPS | P50 Latency | P99 Latency | Status |
|------|--------|---------|-------------|-------------|--------|
| 4K→FHD UNIFIED | 4566 | 152.2 | 6.6 ms | 8.1 ms | ✅ PASS |
| 4K→4K ARGB8888 UNIFIED | 1262 | 42.1 | 23.6 ms | 28.2 ms | ✅ PASS |

## CPU Performance

| Resolution | Latency | FPS | Model Size |
|------------|---------|-----|------------|
| HD (1280×720) | 442 ms | 2 | 18 MB |
| FHD (1920×1080) | 983 ms | 1 | 41 MB |
| 4K (3840×2160) | 3904 ms | 0.25 | 166 MB |

## Speedup (GPU vs CPU)

| Resolution | Speedup |
|------------|---------|
| HD (1280×720) | **340×** |
| FHD (1920×1080) | **438×** |
| 4K (3840×2160) | **312×** |

## Small Frame Performance (CPU)

| Resolution | Avg Time | FPS |
|------------|----------|-----|
| 16×16 | 87.6µs | 11,422 |
| 32×32 | 221.2µs | 4,520 |
| 64×48 | 598.9µs | 1,670 |
| 128×96 | 2.6ms | 379 |

## Pipeline Composition Time

| Resolution | ONNX Emit |
|------------|-----------|
| HD (1280×720) | 95 ms |
| FHD (1920×1080) | 170 ms |
| 4K (3840×2160) | 997 ms |

## Pipeline Architecture

The GPU pipeline uses 4 stages with IspChainFusion:

```
RawInput (INT16 packed) → Unpack+Demosaic+CCM → WarpGrid → Display
     ↓                        ↓                    ↓           ↓
  1 dispatch              1 dispatch          1 dispatch   1 dispatch
```

Total: 4-5 GPU dispatches for the full pipeline.

## Test Suite Results

### Unit Tests (cam-isp library)

| Suite | Tests | Passed | Failed | Duration |
|-------|-------|--------|--------|----------|
| Lib unit tests | 705 | 705 | 0 | 36s |
| Neural controller | 6 | 6 | 0 | 2.6s |
| Neural controller ONNX | 6 | 6 | 0 | 2.0s |

**Recent fixes:**
- ✅ CCM 4-channel support (matrix Vec<f32>, dynamic sizing)
- ✅ All extra_inputs feed through neural controller
- ✅ Feedback loop: Frame N params → Frame N+1 controller

### Integration Tests

| Test | Status | Notes |
|------|--------|-------|
| test_e2e_isp_pipeline | ✅ PASS | 2/2 tests |
| test_mnn_engine | ✅ PASS | 1/8 (7 ignored) |
| test_neural_controller_onnx | ✅ PASS | 6/6 tests |
| test_mnn_heavy_profiler | ✅ PASS | Profile conversion |

### Benchmarks (MNN Vulkan)

| Example | Resolution | Profile | FPS | P99 Latency |
|---------|------------|---------|-----|-------------|
| stress_unified_4k | 4K→FHD | UNIFIED | 152 | 8.1ms |
| stress_unified_4k_to_4k | 4K→4K | UNIFIED | 42 | 28ms |
| bench_e2e_pipeline | FHD | UNIFIED | 438 | 2.3ms |

## Pipeline Profile Comparison

| Profile | Blocks | FPS (4K→FHD) | Output Format |
|---------|--------|--------------|---------------|
| LITE | 20 | ~280 | PackedRgb |
| MED | 20 | ~149 | ARGB |
| HEAVY | 20 | ~149 | ARGB |
| PRO | 23 | ~145 | ARGB |
| UNIFIED | 25 | 152 | ARGB |

## Test Commands

```bash
# GPU benchmark (Vulkan)
ENGINE=vulkan cargo run --release --features mnn --example bench_e2e_pipeline -p cam-isp

# CPU benchmark
ENGINE=cpu cargo run --release --features mnn --example bench_e2e_pipeline -p cam-isp

# Stress test 4K→FHD (30s)
cargo run --release -p cam-isp --example stress_unified_4k --features mnn

# Stress test 4K→4K ARGB (30s)
cargo run --release -p cam-isp --example stress_unified_4k_to_4k --features mnn

# Run all tests (705 lib tests)
cargo test -p cam-isp --features mnn --release

# Integration tests
cargo test --test test_e2e_isp_pipeline --features mnn
cargo test --test test_neural_controller_onnx
cargo test --test test_mnn_engine --features mnn

# Generate API docs
cargo run --release -p cam-isp --example gen_api_docs
```

## Future Work

- [ ] Fix CPUEngine performance (vectorization, SIMD, parallelization)
- [ ] Add MNN library to CI environment
- [ ] Create synthetic benchmark comparing backends
- [ ] Add memory allocation benchmarking with pre-allocated buffers