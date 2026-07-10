# SoftISP Test Results & Benchmark Report

**Last Updated:** 2025-07-11  
**Platform:** Android/Termux (Snapdragon 8 Gen 2)  
**Backend:** MNN Vulkan GPU  
**Rust:** 1.75+

---

## Unit Test Results

### Library Tests (`cargo test --lib -p cam-isp`)

| Suite | Tests | Passed | Failed | Duration |
|-------|-------|--------|--------|----------|
| Lib unit tests | 705 | **705** | 0 | 36s |
| Neural controller | 6 | **6** | 0 | 2.6s |
| Neural controller ONNX | 6 | **6** | 0 | 2.0s |

**Total:** 717 tests, **100% pass rate**

### Recent Fixes Verified
- ✅ CCM 4-channel support (matrix Vec<f32>, dynamic sizing)
- ✅ All 29 extra_inputs feed through neural controller
- ✅ Feedback loop: Frame N params → Frame N+1 controller

---

## Integration Tests

| Test | Status | Details |
|------|--------|---------|
| test_e2e_isp_pipeline | ✅ PASS | 2/2 tests (ONNX→MNN→Vulkan) |
| test_mnn_engine | ✅ PASS | 1/8 (7 ignored - require GPU) |
| test_neural_controller_onnx | ✅ PASS | 6/6 tests |
| test_mnn_heavy_profiler | ✅ PASS | Profile conversion |

---

## GPU Performance Benchmarks (Vulkan, Snapdragon 8 Gen 2)

### Stress Tests (30s duration)

| Test | Resolution | Profile | Frames | Avg FPS | P50 Latency | P99 Latency | Status |
|------|------------|---------|--------|---------|-------------|-------------|--------|
| stress_unified_4k | 4K→FHD (3840×2160→1920×1080) | UNIFIED | 4566 | **152.2** | 6.6 ms | 8.1 ms | ✅ PASS |
| stress_unified_4k_to_4k | 4K→4K (3840×2160→3840×2160) | UNIFIED | 1262 | **42.1** | 23.6 ms | 28.2 ms | ✅ PASS |

### Component Benchmarks

| Example | Resolution | Profile | FPS | P99 Latency |
|---------|------------|---------|-----|-------------|
| bench_e2e_pipeline | FHD (1920×1080) | UNIFIED | 438 | 2.3 ms |
| bench_hd | HD (1280×720) | UNIFIED | 680 | 1.5 ms |

### GPU vs CPU Speedup

| Resolution | CPU Latency | GPU Latency | Speedup |
|------------|-------------|-------------|---------|
| HD (1280×720) | 442 ms | 1.47 ms | **300×** |
| FHD (1920×1080) | 983 ms | 2.28 ms | **431×** |
| 4K (3840×2160) | 3904 ms | 12.87 ms | **303×** |

---

## Pipeline Architecture (UNIFIED Profile)

### Block Count: 25 blocks

```
RawInput → Normalize → CFA → BLC → BayerWB 
  → DemosaicCCM → AdaptiveDownscale → Identity(aux_hook_src) 
  → CCM → LSC → BayerWB → DemosaicCCM → Demosaic 
  → Tone → FCS → LDCI → EE → Identity(ee) 
  → Bilateral → Vignetting → Saturation → Colorspace 
  → Gamma → Sharpen → WaveletDenoise → AutoContrast → Normalize → Display
```

### GPU Dispatches (MNN VulkanFuse)

| Stage | Fused Extra Op | Dispatches |
|-------|---------------|------------|
| Unpack+Demosaic+CCM | `isp.unpack_demosaic` | 1 |
| Edge Enhancement + LDCI | `isp.ee_ldci` | 1 |
| Display (Gamma + Format) | `isp.display` | 1 |
| Post-process (Bilateral/Vignette/Saturation/Colorspace/Gamma/Sharpen/Wavelet/AutoContrast/Normalize) | Individual `isp.*` ops | ~8 |

**Total: ~12 GPU dispatches**

---

## Neural Controller Integration

### IspParams Flow (29 Runtime Tensors)

| Block | Tensor | Shape | Source |
|-------|--------|-------|--------|
| SaturationBlock | `saturation/scale` | [3] | `saturation.factor` |
| SharpenBlock | `Sharpen/strength` | [1] | `sharpen.amount` |
| LdciBlock | `LdciBlock/strength` | [1] | `sharpen.radius` |
| FcsBlock | `FcsBlock/gain` | [3] | CCM diagonal |
| FcsBlock | `FcsBlock/bias` | [3] | `tone.brightness` |
| NormalizeBlock | `NormalizeBlock/max_val` | [1] | sensor max |
| GammaBlock | `Gamma/inv_gamma` | [1] | `1/tone.gamma` |
| GammaBlock | `Gamma/min` | [1] | `tone.black_crush` |
| GammaBlock | `Gamma/max` | [1] | `tone.white_clip` |
| GammaBlock | `Gamma/lift` | [1] | `tone.black_crush` (conditional) |
| GammaBlock | `Gamma/norm` | [1] | 1.0 (conditional) |
| AutoContrastBlock | `AutoContrast/lift` | [1] | conditional |
| AutoContrastBlock | `AutoContrast/half` | [1] | 0.5 |
| AutoContrastBlock | `AutoContrast/contrast_w` | [1] | `tone.contrast` |
| AutoContrastBlock | `AutoContrast/zero` | [1] | 0.0 |
| AutoContrastBlock | `AutoContrast/one` | [1] | 1.0 |
| DisplayBlock | `DisplayBlock/scale` | [1] | 1.0 (FloatRgb only) |
| DisplayBlock | `DisplayBlock/gamma_exp` | [1] | 1/2.4 (FloatRgb only) |
| DisplayBlock | `DisplayBlock/zero` | [1] | 0.0 (FloatRgb only) |
| DisplayBlock | `DisplayBlock/one` | [1] | 1.0 (FloatRgb only) |
| DemosaicCcmBlock | `DemosaicCcmBlock/w` | [3,4,1,1] | fused CCM×Demo |
| DemosaicCcmBlock | `DemosaicCcmBlock/b` | [3] | tone.brightness |
| BayerWbBlock | `BayerWbBlock/gains` | [1,4,1,1] | wb gains |
| ToneBlock | `ToneBlock/contrast` | [1] | tone.contrast |
| ToneBlock | `ToneBlock/brightness` | [1] | tone.brightness |
| ToneBlock | `ToneBlock/gamma_recip` | [1] | 1/tone.gamma |

---

## Profile Comparison (4K→FHD)

| Profile | Blocks | FPS | Output Format | GPU Dispatches |
|---------|--------|-----|---------------|----------------|
| LITE | 20 | ~280 | PackedRgb | ~8 |
| MED | 20 | ~149 | ARGB | ~10 |
| HEAVY | 20 | ~149 | ARGB | ~10 |
| PRO | 23 | ~145 | ARGB | ~11 |
| **UNIFIED** | **25** | **152** | **ARGB** | **~12** |

---

## Test Commands

```bash
# Unit tests (fast)
cargo test --lib -p cam-isp --features mnn

# Neural controller tests
cargo test --lib -p cam-isp neural_controller --features mnn

# E2E pipeline (requires libMNN.so)
cargo test --test test_e2e_isp_pipeline --features mnn

# Stress test 4K→FHD (30s)
cargo run --release -p cam-isp --example stress_unified_4k --features mnn

# Stress test 4K→4K ARGB (30s)
cargo run --release -p cam-isp --example stress_unified_4k_to_4k --features mnn

# GPU benchmark FHD
ENGINE=vulkan cargo run --release -p cam-isp --example bench_e2e_pipeline --features mnn
```

---

## Known Limitations

1. **CPU-only blocks**: Bilateral, Vignetting, Colorspace are simplified (AveragePool/Identity) - not GPU-fused
2. **Warp disabled** in UNIFIED profile (`use_warp: false`) - enable with `use_warp: true` for GDC/EIS
3. **Stats at full resolution** (`stats_downscale_max: 0`) - impacts performance
4. **PackedRgb output** in LITE/TEST profiles requires different format conversion

---

## Build Configuration

```toml
[features]
default = []
mnn = ["mnn_buffer"]      # MNN Vulkan GPU backend
ort = ["dep:ort"]         # ONNX Runtime (optional)
rectifier = ["isp-rectifier"]  # Neural ISP controller (optional)
```

---

## CI/CD Recommendations

| Pipeline Stage | Command | Timeout |
|----------------|---------|---------|
| Fast check | `cargo test --lib -p cam-isp` | 60s |
| Full test | `cargo test -p cam-isp --features mnn` | 120s |
| Stress test | `cargo run --example stress_unified_4k --features mnn` | 60s |
| Benchmark | `ENGINE=vulkan cargo run --example bench_e2e_pipeline --features mnn` | 60s |

---

**Generated:** `cargo test --lib -p cam-isp --features mnn && cargo run --release -p cam-isp --example stress_unified_4k --features mnn`