# SoftISP — CLAUDE.md

## Project summary
SoftISP is a complete camera ISP pipeline in Rust with Vulkan GPU acceleration via MNN. Converts raw Bayer sensor data into display-ready images in real time:

```
Raw → Unpack → BLC → WB → Demosaic → CCM → Tone → Denoise → Display
```

Pipeline stages are `IspBlock`s composable into ONNX, fused into GPU `isp.*` ops, executed on CPU (`CpuEngine`) or GPU (`MnnEngine`/Vulkan).

## MNN repo relationship
- MNN source lives at `/data/data/com.termux/files/home/MNN/` (separate repo, not part of softisp).
- SoftISP only consumes prebuilt MNN libs: `cam-rust/lib/aarch64-v8a/libMNN.so`, `libMNN_Vulkan.so`, `libvulkan.so`, `libMNNConvertDeps.so`.
- MNN C++ FFI wrapper lives in `cam-rust/cam-isp/mnn_sys/` (`mnn_wrapper.cpp/h`), compiled against `libMNN.so`/`libMNN_Vulkan.so`.

## Where to find more info (docs/)
- `README.md` — overview, key features, quick start, APK build
- `docs/overview/` — PROJECT_STATUS.md, ROADMAP.md, SUMMARY.md, PROGRESS.md, TECH_SUMMARY.md
- `docs/architecture/` — pipeline blocks, custom opset design, ISP ops plan
- `docs/mnn/` — MNN status, Vulkan backend, convert, zero-copy optimization (ARCH_ISP_OUTPUT_ZERO_COPY.md), profile optimization
- `docs/testing/` — test plan, results
- `cam-rust/README.md` — build/test/lint (CI gate), APK build guide
- `docs/guides/` — MNN Vulkan guide, memfd guide, NDK camera capture study

## Build / test / lint (CI gate)
```
cd cam-rust
cargo fmt --all -- --check
cargo clippy -p cam-isp --lib --features rectifier -- -D warnings
cargo test -p cam-isp --lib --features rectifier --no-fail-fast   # 813 lib tests pass
```

## Zero-copy ISP → Android display
- `ProcessParams.input_buffer`/`output_buffer` → `mnn_run_true_zero_copy()` (no heap `Vec`, no f32→u8 pass)
- `mnn_import_ahb_as_device_memory()`/`mnn_release_ahb_device_memory()` — AHardwareBuffer → VkDeviceMemory import via Tensor::setDevicePtr(MNN_MEMORY_AHARDWAREBUFFER)
- Remaining: bind imported VkDeviceMemory to MNN Vulkan session input/output tensors for true host↔GPU zero-copy on Vulkan

## Performance (MNN Vulkan, Snapdragon-class)
- HEAVY 4K→4K: **79.8 FPS** avg; UNIFIED 4K→4K tiled 2×2: **141.2 FPS** avg
- Primitive chain: 1.9 / 4.0 FPS at 4K/FHD
