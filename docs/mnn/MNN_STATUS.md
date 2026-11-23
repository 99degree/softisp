# MNN Backend Status

## Current State

| Component | Status | Notes |
|-----------|--------|-------|
| C++ FFI wrapper (`mnn_sys/`) | ✅ Compiles | `mnn_wrapper.cpp` builds via `build.rs` |
| Rust MNN engine (`mnnengine.rs`) | ✅ Compiles | Behind `mnn` feature flag |
| MNN runtime libs (`libMNN.so`) | ✅ Present | `cam-rust/lib/aarch64-v8a/libMNN.so` |
| MNN Vulkan backend (`libMNN_Vulkan.so`) | ✅ Present | `cam-rust/lib/aarch64-v8a/libMNN_Vulkan.so` |
| Vulkan ICD loader (`libvulkan.so`) | ✅ Present | `cam-rust/lib/aarch64-v8a/libvulkan.so` |
| MNN convert (`libMNNConvertDeps.so`) | ✅ Present | `cam-rust/lib/aarch64-v8a/libMNNConvertDeps.so` |
| Unit tests with `mnn` feature | ⚠️ Link-dependent | Needs `LD_LIBRARY_PATH` pointing to libs |
| `libmnn_convert_api.a` | ✅ Present | Static convert API built via `build.rs` |
| `libmnn_vulkan_init.a` / `libmnn_vulkan_stubs.a` | ✅ Present | Vulkan init + stubs for VulkanFuse |
| `libmnn_wrapper.a` | ✅ Present | C++ MNN wrapper static lib |

## What Works

```bash
cd cam-rust
export LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a

# Check compilation
cargo check -p cam-isp --features mnn    # ✅ Passes

# Build with MNN
cargo build -p cam-isp --features mnn      # ✅ Builds
```

## Required to Run MNN Tests

```bash
export LD_LIBRARY_PATH=$PWD/cam-rust/lib/aarch64-v8a
cargo test -p cam-isp --features mnn --lib --no-fail-fast
```

Libraries must be on `LD_LIBRARY_PATH` at link time and runtime. Without them, MNN-dependent tests are skipped/link-fail — default (no-feature) tests still pass.

## MNN Buffer Tests (no runtime needed)

The `mnn_buffer` module provides zero-copy buffer abstractions:
- `MemfdBuffer` — memfd-backed buffer for DMA
- `CmaBuffer` — contiguous memory allocator buffer
- `MnnBuffer` — generic trait over MNN tensor memory

These can be tested without libMNN.so:
```bash
cargo run --example mnn_memfd_test -p cam-isp --features mnn
```
