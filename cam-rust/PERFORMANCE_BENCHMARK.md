# SoftISP Performance Benchmark Results

## Test Environment
- Platform: Android (Termux)
- CPU: ARM64
- Backends: CPU, MNN (CPU), MNN (Vulkan)

## Benchmark Results

### Small Frame Performance (CPU Backend)

| Resolution | Avg Time | FPS | Throughput |
|------------|----------|-----|------------|
| 16×16 | 87.6µs | 11,422 fps | - |
| 32×32 | 221.2µs | 4,520 fps | 17.9 MB/s |
| 64×48 | 598.9µs | 1,670 fps | - |
| 128×96 | 2.6ms | 379 fps | - |

### E2E Pipeline Performance

| Resolution | CPU | MNN CPU |
|------------|-----|----------|
| HD (1280×720) | 442 ms (2 FPS) | 442 ms (2 FPS) |
| FHD (1920×1080) | 983 ms (1 FPS) | 983 ms (1 FPS) |
| 4K (3840×2160) | 3,904 ms (0.25 FPS) | 3,904 ms (0.25 FPS) |

### Pipeline Composition Time

| Resolution | ONNX Emit Time |
|------------|----------------|
| HD (1280×720) | 102 ms |
| FHD (1920×1080) | 229 ms |
| 4K (3840×2160) | 875 ms |

## Analysis

### Strengths
1. **Small frame performance**: Excellent for thumbnail/preview processing
2. **Pipeline composition**: Fast ONNX graph generation
3. **Consistent latency**: Low variance (P50 ≈ P99)

### Areas for Improvement
1. **Large frame processing**: 4K requires GPU acceleration for real-time
2. **Model size**: Large models for full pipeline (166 MB for 4K)
3. **Vulkan backend**: Currently crashes during MNN conversion for complex models

### Recommendations
1. **Fix Vulkan backend**: Debug MNN conversion for complex ONNX models
2. **Use quantized models** (INT8) for mobile deployment
3. **Implement tiling** for large frame processing

## Test Commands

```bash
# Small frame benchmark
cargo run --release --example bench

# E2E pipeline benchmark (CPU)
ENGINE=cpu cargo run --release --features mnn --example bench_e2e_pipeline

# E2E pipeline benchmark (Vulkan)
ENGINE=vulkan cargo run --release --features mnn --example bench_e2e_pipeline

# Block-level benchmark (requires mnn feature)
cargo run --release --features mnn --example bench_blocks
```
