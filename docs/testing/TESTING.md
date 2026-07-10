# Testing Guide

## Quick Test (default)
Runs only fast unit tests (no CPU pipeline execution):

```bash
cargo test --lib -p cam-isp
# or
bash bench-tests.sh quick
```

**Result:** 705 tests pass in ~36s

## Extended Test (includes CPU pipeline)
Runs all unit tests plus CPU pipeline integration tests (slow!):

```bash
cargo test --lib -p cam-isp -- --include-ignored
# or
bash bench-tests.sh extended
```

**Result:** 705 tests pass + 2 additional pipeline tests (4×4 frame, 10 frames)
**Warning:** CPU pipeline tests are extremely slow (seconds per frame) and may appear to hang.

## Why CPU pipeline is slow

The `CpuEngine` runs the full 16-stage ISP pipeline in pure Rust:
- DPC (defective pixel correction)
- Gaussian denoise
- Calibration stats
- AWB/AE/CCM controllers
- Demosaic (Malvar)
- Tone mapping, FCS, LDCI, Warp, etc.

Even for a 4×4 frame, this involves thousands of floating-point operations and multiple mutex locks for controller state.

For this reason, **CPU pipeline tests are excluded from default test runs**.

## Backend Strategy

The project supports multiple backends:

| Backend | Feature | Status | Performance |
|---------|---------|--------|-------------|
| CPU | default | ✅ Works | Very slow |
| ONNX | `ort` | ❌ Needs `libonnxruntime.so` | Fast (if available) |
| MNN | `mnn` | ❌ Needs `libMNN.so` + headers | Fast (if available) |

When ONNX/MNN libraries are available, `FusedPipeline::build()` auto-selects the fastest backend (MNN Vulkan > ONNX NNAPI > CPU).

## Integration Tests

Integration tests (in `cam-isp/tests/`) are separate from unit tests:

```bash
# Run integration tests
cargo test --test pipeline_test -p cam-isp -- --include-ignored
cargo test --test test_e2e_isp_pipeline --features mnn
cargo test --test test_mnn_engine --features mnn
cargo test --test test_neural_controller_onnx
```

These also use the CPU backend and are marked as slow.

## Current Test Results (2025-07-11)

### Unit Tests (cam-isp library)
```
test blocks::ccm::tests::test_ccm_extra_inputs ... ok
test blocks::ccm::tests::test_ccm_id ... ok
test blocks::ccm::tests::test_ccm_instance_id ... ok
test blocks::ccm::tests::test_ccm_with_channels ... ok      (FIXED)
test blocks::ccm::tests::test_ccm_different_channels ... ok  (FIXED)
test blocks::ccm::tests::test_ccm_nodes ... ok
...
test result: ok. 705 passed; 0 failed
```

**Recent fix:** CCM block 4-channel support - matrix/bias changed from fixed `[f32;9]`/`[f32;3]` to `Vec<f32>` with dynamic sizing in `with_channels(ch)`.

### Neural Controller Tests
```
test test_isp_params_structure ... ok
test test_params_flow_to_frame ... ok
test test_mock_model_controller ... ok
test test_rule_based_vs_neural ... ok
test test_per_frame_variation ... ok
test test_temporal_smoothing_effect ... ok
test result: ok. 6 passed
```

### Integration Tests
| Test | Status | Duration |
|------|--------|----------|
| test_e2e_isp_pipeline | ✅ 2 passed | 0.04s |
| test_mnn_engine | ✅ 1 passed (7 ignored) | 0.01s |
| test_neural_controller_onnx | ✅ 6 passed | 2.6s |

## Stress Tests

### 4K→FHD (3840×2160 → 1920×1080)
```
cargo run --release -p cam-isp --example stress_unified_4k --features mnn
```
**Results (30s):**
- Frames: 4566
- Average FPS: 152.2
- P50 Latency: 6.6ms
- P99 Latency: 8.1ms
- ✅ PASS

### 4K→4K ARGB8888 (3840×2160 → 3840×2160)
```
cargo run --release -p cam-isp --example stress_unified_4k_to_4k --features mnn
```
**Results (30s):**
- Frames: 1262
- Average FPS: 42.1
- P50 Latency: 23.6ms
- P99 Latency: 28.2ms
- ✅ PASS

## CI/CD Recommendations

In continuous integration:
- Use `cargo test --lib -p cam-isp` for fast feedback (~36s)
- Optionally run `--include-ignored` on nightly builds to validate CPU pipeline
- For full performance validation, use a machine with MNN/ONNX libraries installed

## Future Work

- [ ] Fix CPUEngine performance (vectorization, SIMD, parallelization)
- [ ] Add MNN library to CI environment
- [ ] Create synthetic benchmark comparing backends
- [ ] Add memory allocation benchmarking with pre-allocated buffers