# MNN Backend Status

## Current State

| Component | Status | Notes |
|-----------|--------|-------|
| C++ FFI wrapper (`mnn_sys/`) | ✅ Compiles | `mnn_wrapper.cpp` builds via `build.rs` |
| Rust MNN engine (`mnnengine.rs`) | ✅ Compiles | Behind `mnn` feature flag |
| MNN runtime libs (`libMNN.so`) | ❌ Missing | Not in repo or device |
| MNN convert (`libMNNConvertDeps.so`) | ✅ Present | `vendor/mnn/mnnconvert/lib/` |
| Unit tests with `mnn` feature | ❌ Link fails | Missing `libMNN.so` at link time |

## What Works

```bash
# Check compilation (no linking)
cargo check -p cam-isp --features mnn    # ✅ Passes

# Build C++ wrapper only
cargo build -p cam-isp --features mnn      # ✅ Builds libmnn_wrapper.a
```

## What Fails

```bash
# Link-time error — libMNN.so not found
cargo test -p cam-isp --features mnn       # ❌ ld: cannot find -lMNN
```

## Required to Enable MNN

1. **Obtain prebuilt MNN libraries** (one of):
   - Build MNN from source for Android ARM64
   - Download official MNN release for Android
   - Use MNN from device vendor partition (`/vendor/lib64/libMNN.so`)

2. **Place libraries in expected path**:
   ```
   cam-rust/cam-isp/lib/aarch64/
   ├── libMNN.so
   ├── libMNN_Express.so
   ├── libMNN_Vulkan.so
   ├── libMNN_CL.so
   └── libMNN_GL.so
   ```

3. **Set environment if using custom path**:
   ```bash
   export LD_LIBRARY_PATH=/path/to/mnn/lib:$LD_LIBRARY_PATH
   export MNN_LIB_DIR=/path/to/mnn/lib
   ```

## Quick Verify Script

```bash
cd cam-rust && ./test_mnn.sh
```

## MNN Buffer Tests (no runtime needed)

The `mnn_buffer` module provides zero-copy buffer abstractions:
- `MemfdBuffer` — memfd-backed buffer for DMA
- `CmaBuffer` — contiguous memory allocator buffer
- `MnnBuffer` — generic trait over MNN tensor memory

These can be tested without libMNN.so:
```bash
# Memfd buffer example (no MNN runtime needed)
cargo run --example mnn_memfd_test -p cam-isp --features mnn
```
