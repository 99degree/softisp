# Testing Guide

## Quick Test (default)
Runs only fast unit tests (no CPU pipeline execution):

```bash
cargo test --lib -p cam-isp
# or
bash bench-tests.sh quick
```

**Result:** 133 tests pass in ~0.1s

## Extended Test (includes CPU pipeline)
Runs all unit tests plus CPU pipeline integration tests (slow!):

```bash
cargo test --lib -p cam-isp -- --include-ignored
# or
bash bench-tests.sh extended
```

**Result:** 133 tests pass + 2 additional pipeline tests (4×4 frame, 10 frames)
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
```

These also use the CPU backend and are marked as slow.

## Performance Benchmark (Component-level)

To benchmark individual ISP components without the full pipeline overhead:

```bash
# Not yet implemented - use external tools like perf/valgrind
```

## CI/CD Recommendations

In continuous integration:
- Use `cargo test --lib -p cam-isp` for fast feedback
- Optionally run `--include-ignored` on nightly builds to validate CPU pipeline
- For full performance validation, use a machine with MNN/ONNX libraries installed

## Future Work

- [ ] Fix CPUEngine performance (vectorization, SIMD, parallelization)
- [ ] Add MNN library to CI environment
- [ ] Create synthetic benchmark comparing backends
- [ ] Add memory allocation benchmarking with pre-allocated buffers
