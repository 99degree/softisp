# SoftISP — CLAUDE.md (short)

## What this is
Complete camera ISP pipeline in Rust with Vulkan GPU acceleration via MNN. Converts raw Bayer sensor data → display-ready images in real time.

Raw → Unpack → BLC → WB → Demosaic → CCM → Tone → Denoise → Display

Pipeline stages are `IspBlock`s composable into ONNX, fused into GPU `isp.*` ops, executed on CPU (`CpuEngine`) or GPU (`MnnEngine`/Vulkan).

## Where to find more info
- `README.md` — overview, key features, quick start, APK build
- `docs/overview/` — PROJECT_STATUS.md, ROADMAP.md, SUMMARY.md, PROGRESS.md, TECH_SUMMARY.md
- `docs/architecture/` — pipeline blocks, custom opset design, ISP ops plan
- `docs/mnn/` — MNN status, Vulkan backend, convert, zero-copy optimization (ARCH_ISP_OUTPUT_ZERO_COPY.md), profile optimization
- `docs/testing/` — test plan, results
- `cam-rust/README.md` — build/test/lint (CI gate), APK build guide
- `docs/guides/` — MNN Vulkan guide, memfd guide, NDK camera capture study

## MNN sub-project (cam-rust/cam-isp/mnn_sys/)
- C++ C wrapper around libMNN.so + libMNN_Vulkan.so (Android NDK build)
- `mnn_wrapper.cpp` — `mnn_run_true_zero_copy`, `mnn_import_ahb_as_device_memory`, session/tensor FFI
- `mnn_sys.rs` — Rust FFI bindings
- `mnnengine.rs` — `MnnEngine` (Vulkan backend, `mnn` feature flag)
- `mnn_session_pool.rs` — session pool with per-slot resize tracking
- `mnn_buffer/` — MemfdBuffer, CmaBuffer, OutputBufferPool zero-copy abstractions
- Prebuilt libs in `cam-rust/lib/aarch64-v8a/`: libMNN.so, libMNN_Vulkan.so, libvulkan.so, libMNNConvertDeps.so

## Build / test / lint (CI gate)
```
cd cam-rust
cargo fmt --all -- --check
cargo clippy -p cam-isp --lib --features rectifier -- -D warnings
cargo test -p cam-isp --lib --features rectifier --no-fail-fast   # 813 lib tests pass
```

## Zero-copy ISP → Android display (current work)
- `ProcessParams.input_buffer` / `output_buffer` → `mnn_run_true_zero_copy()` (no heap Vec, no f32→u8 copy)
- `mnn_import_ahb_as_device_memory()` / `mnn_release_ahb_device_memory()` — AHardwareBuffer → VkDeviceMemory import FFI (mnn_wrapper.cpp/h)
- Remaining: bind imported VkDeviceMemory to MNN Vulkan session input/output tensors for true host↔GPU zero-copy on Vulkan (see docs/mnn/ARCH_ISP_OUTPUT_ZERO_COPY.md)

## Performance (MNN Vulkan, Snapdragon-class)
- HEAVY 4K→4K: **79.8 FPS** avg; UNIFIED 4K→4K tiled 2×2: **141.2 FPS** avg
- Primitive chain: 1.9 / 4.0 FPS at 4K/FHD
