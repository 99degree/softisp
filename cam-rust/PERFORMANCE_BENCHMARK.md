# SoftISP Performance Benchmark Results

## Test Environment
- Platform: Android (Termux)
- CPU: ARM64
- Backend: CPU (no GPU acceleration)

## Benchmark Results

### Small Frame Performance (CPU Backend)

| Resolution | Avg Time | FPS | Throughput |
|------------|----------|-----|------------|
| 16×16 | 87.6µs | 11,422 fps | - |
| 32×32 | 221.2µs | 4,520 fps | 17.9 MB/s |
| 64×48 | 598.9µs | 1,670 fps | - |
| 128×96 | 2.6ms | 379 fps | - |

### E2E Pipeline Performance (CPU Backend)

| Resolution | Avg Time | FPS | Model Size |
|------------|----------|-----|------------|
| HD (1280×720) | 211ms | 5 fps | 18 MB |
| FHD (1920×1080) | 470ms | 2 fps | 41 MB |
| 4K (3840×2160) | 2,006ms | 0.5 fps | 166 MB |

### Pipeline Composition Time

| Resolution | ONNX Emit Time |
|------------|----------------|
| HD (1280×720) | 72 ms |
| FHD (1920×1080) | 158 ms |
| 4K (3840×2160) | 586 ms |

## Analysis

### Strengths
1. **Small frame performance**: Excellent for thumbnail/preview processing
2. **Pipeline composition**: Fast ONNX graph generation
3. **Consistent latency**: Low variance (P50 ≈ P99)

### Areas for Improvement
1. **Large frame processing**: 4K requires GPU acceleration for real-time
2. **Model size**: Large models for full pipeline (166 MB for 4K)

### Recommendations
1. **Enable MNN/Vulkan backend** for GPU acceleration
2. **Use quantized models** (INT8) for mobile deployment
3. **Implement tiling** for large frame processing

## Test Commands

```bash
# Small frame benchmark
cargo run --release --example bench

# E2E pipeline benchmark
cargo run --release --example bench_e2e_pipeline

# Block-level benchmark (requires mnn feature)
cargo run --release --features mnn --example bench_blocks
```
