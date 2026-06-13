# Build Status — SoftISP Rust Port

## Current State

✅ **All 133 unit tests pass** (no failures)
✅ **Workspace builds** in release mode (3m 46s)
⚠️ **CPU pipeline works** but is extremely slow (seconds per frame)
❌ **ONNX/MNN backends** not available (missing native libraries)

## Quick Start

```bash
# Build all crates
cargo build --release

# Run fast unit tests (recommended)
cargo test --lib -p cam-isp

# Run all tests including CPU pipeline (slow)
cargo test --lib -p cam-isp -- --include-ignored
```

## Test Organization

### Fast Unit Tests (default)
Located in `cam-isp/src/*.rs` `#[cfg(test)]` modules. These test individual components:

- Auto-exposure (ae)
- Autofocus (af)
- Black level & calibration
- CCM engine
- Config parsing
- Controller state machine
- EIS gyro stabilization
- Genetic optimizer
- Pipeline profile building
- Predictor & regression
- Scene classification
- Store & persistence
- Warp functions
- ... and more

**Runtime:** ~0.1s

### Extended Tests (ignored by default)
Tests that invoke `CpuEngine::process()`:

- `cpu::tests::test_cpu_engine_process` — single 16×16 frame
- `manager::tests::test_manager_build_and_process` — PipelineManager full flow
- Integration tests in `tests/pipeline_test.rs` — multi-frame AWB/AE convergence

**Runtime:** Very slow (full 16-stage pipeline, even for tiny frames)

## Backend Availability

| Feature | Crate Flag | Dependency | Status |
|---------|------------|------------|--------|
| ONNX Runtime | `ort` | `libonnxruntime.so` | ❌ Not installed |
| MNN | `mnn` | `libMNN.so` + headers | ❌ Headers missing |
| MNN Buffer | `mnn_buffer` | (optional) | Code present |

To enable MNN/ONNX:
1. Install the native library (e.g., via apt, conda, or prebuilt .so)
2. Set library path (LD_LIBRARY_PATH or rpath)
3. Build with `--features mnn` or `--features ort`

## Examples

| Example | Status | Notes |
|---------|--------|-------|
| `pipeline` | ⚠️ Hangs | Full CPU pipeline, slow even for tiny frames |
| `bench` | ⚠️ Hangs | Same issue |
| `mnn_memfd_test` | Needs MNN | Requires `mnn` feature + libMNN |

All examples that call `CpuEngine::process()` suffer from the same performance issue.

## Known Issues

1. **CPU pipeline performance** — The pure Rust implementation is not optimized. Consider:
   - Using SIMD intrinsics (faster math)
   - Parallel processing with rayon
   - Avoiding unnecessary allocations (reuse buffers)
   - Profiling with `perf`/`flamegraph` to find bottlenecks

2. **Missing MNN headers** — The `mnn_sys` build needs `MNN/Interpreter.hpp`. We have `libMNN.so` but not the C++ headers. Solutions:
   - Download MNN source and copy headers to `/data/data/com.termux/files/home/MNN/include`
   - Use prebuilt package that includes headers

3. **Examples hang** — Don't use `cargo run --example pipeline` for timing. It's not a benchmark, just a demo.

## File Locations

```
cam-rust/
├── Cargo.toml                 # Workspace (10 crates)
├── cam-isp/                   # Main ISP crate
│   ├── src/
│   │   ├── cpu.rs            # CpuEngine (slow but works)
│   │   ├── onnx/mod.rs       # ONNX engine (ort feature)
│   │   ├── mnnengine.rs      # MNN engine (mnn feature)
│   │   ├── mnn_sys.rs        # MNN FFI bindings
│   │   └── examples/
│   │       ├── pipeline.rs   # Demo (hangs)
│   │       └── bench.rs      # Benchmark (hangs)
│   └── tests/
│       └── pipeline_test.rs  # Integration tests (ignored)
├── lib/                       # Native libraries
│   ├── aarch64/libMNN.so
│   ├── arm64-v8a/libMNN.so
│   └── ...
└── target/release/            # Compiled artifacts
```

## Recommendations

1. **For development:** Use `cargo test --lib -p cam-isp` for fast feedback.
2. **For performance analysis:** Profile the CPU pipeline with `perf` or `flamegraph` to identify hotspots.
3. **For production:** Deploy with MNN backend (requires proper library packaging).
4. **For CI:** Run quick tests only; extended tests on a powerful nightlies runner.

## Next Steps

- [ ] Investigate why CPU pipeline is so slow (algorithmic or implementation?)
- [ ] Obtain MNN C++ headers and link properly
- [ ] Or obtain libonnxruntime.so and enable `ort` feature
- [ ] Optimize CpuEngine with rayon parallelism and SIMD
- [ ] Add pre-allocated buffer API to reduce heap churn
- [ ] Create proper performance regression tests (using fast backend)
