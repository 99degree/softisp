# SoftISP Performance Benchmark Results

## Test Environment
- Platform: Android (Termux)
- CPU: ARM64 (Snapdragon 8 Gen 2)
- Backends: CPU, MNN Vulkan GPU

## GPU Performance (Vulkan)

| Resolution | Latency | FPS | Model Size |
|------------|---------|-----|------------|
| HD (1280×720) | 1.47 ms | **680** | 18 MB |
| FHD (1920×1080) | 2.28 ms | **438** | 41 MB |
| 4K (3840×2160) | 12.87 ms | **78** | 166 MB |

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

## Test Commands

```bash
# GPU benchmark (Vulkan)
ENGINE=vulkan cargo run --release --features mnn --example bench_e2e_pipeline -p cam-isp

# CPU benchmark
ENGINE=cpu cargo run --release --features mnn --example bench_e2e_pipeline -p cam-isp

# Small frame benchmark
cargo run --release --example bench -p cam-isp

# Run all tests (637)
cargo test -p cam-isp --features mnn --release
```
