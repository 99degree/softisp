# SoftISP — CLAUDE.md

## Project summary
SoftISP is a complete camera ISP pipeline in Rust with Vulkan GPU acceleration via MNN. Converts raw Bayer sensor data into display-ready images in real time:

```
Raw → Unpack → BLC → WB → Demosaic → CCM → Tone → Denoise → Display
```

Pipeline stages are `IspBlock`s composable into ONNX, fused into GPU `isp.*` ops, executed on CPU (`CpuEngine`) or GPU (`MnnEngine`/Vulkan).

## MNN repo relationship
- MNN source is a separate local checkout, optionally linked at `third_party/MNN`; build scripts fall back to `MNN_DIR` or `$HOME/MNN`.
- SoftISP only consumes prebuilt MNN libs: `cam-rust/lib/aarch64-v8a/libMNN.so`, `libMNN_Vulkan.so`, `libvulkan.so`, `libMNNConvertDeps.so`.
- MNN C++ FFI wrapper lives in `cam-rust/cam-isp/mnn_sys/` (`mnn_wrapper.cpp/h`), compiled against `libMNN.so`/`libMNN_Vulkan.so`.

## Where to find more info (docs/)

| Dev stage | Doc | What it covers |
|-----------|-----|----------------|
| Overview / getting started | `README.md` | project summary, key features, quick start, APK build |
| Project status / roadmap | `docs/overview/` — PROJECT_STATUS.md, ROADMAP.md, SUMMARY.md, PROGRESS.md, TECH_SUMMARY.md | where the project is, what's next, tech summary |
| Pipeline / block design | `docs/architecture/` — PIPELINE_BLOCKS.md, CUSTOM_OPSET_DESIGN.md, ISP_OPS_PLAN.md, ARCHITECTURE.md | ISP block I/O, custom opset design, architecture |
| ISP / shader work | `docs/isp/` — ISP_OP_AUDIT.md, ISP_SHADER_FIXES.md, KEY_DESIGN_DECISIONS.md, FHD_BENCHMARK.md | ISP math, shader fixes, benchmarks |
| MNN rebuild / convert | `docs/mnn/` — MNN_STATUS.md, MNN_CONVERT.md, MNN_VULKAN_BACKEND.md, MNN_ZERO_COPY_OPTIMIZATION.md, ARCH_ISP_OUTPUT_ZERO_COPY.md, MNN_REBUILD_BLOCKER.md | MNN state, convert flow, Vulkan backend, zero-copy |
| MNN Vulkan deep dive | `docs/guides/MNN_VULKAN_GUIDE.md` | MNN + Vulkan on Android |
| Memfd / zero-copy buffers | `docs/guides/MEMFD_MNN_GUIDE.md` | memfd/CMA buffer handling for MNN |
| NDK camera capture | `docs/guides/NDK_CAMERA_CAPTURE_STUDY.md` | Android Camera2 HAL3 capture study |
| **Build the APK** | `docs/build/README_BUILD_APK.md` | toolchain, MNN subproject build first, env vars, cargo-apk2, CI gate, troubleshooting |
| **Debug APK crash/fail** | `docs/build/DEBUG_APK.md` | user installs `target/debug/apk/softisp-camera.apk` directly (no `install-apk.sh`/`pm` in Termux), launch via launcher/am start, read crash log (`/sdcard/download/softisp-app.log` or fallback), run `logcat` directly on device — no `adb` |
| **Shim fast-debug (no reinstall)** | `docs/build/SHIM_DEBUG_WORKFLOW.md` | build `softisp-test.apk` once (shim-only launcher, no MNN/core bundled), push core+MNN libs to `/sdcard/download/`, relaunch to iterate. `./shim-debug.sh core` → `stop` → `launch` → `logs` → `verify`. MD5 fingerprints prove device runs exact local binaries. §9 covers Vulkan "probe session create fail" debugging. |
| CI gate / lint / test | `cam-rust/README.md` | build/test/lint commands (same as below) |
| Test plan / results | `docs/testing/` — TESTING.md, TEST_PLAN_MNN_VULKAN.md, TEST_RESULTS.md | how tests are organized and results |
| Other guides | `docs/guides/` — MNN_SOLUTION_SUMMARY.md, profiles-technical.md, mnn-inference-guide.md | MNN solution summary, profile tech, inference guide |

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

## Dual-GPU engine
- `DualGpuEngine` alternates between two `MnnEngine` instances for ping-pong inference (frame N on A, N+1 on B)
- Both engines share `IspController` for consistent 3A state
- Registered as `mnn_vulkan_a` and `mnn_vulkan_b` factories

## Performance (MNN Vulkan, Snapdragon-class)
- HEAVY 4K→4K: **79.8 FPS** avg; UNIFIED 4K→4K tiled 2×2: **141.2 FPS** avg
- Primitive chain: 1.9 / 4.0 FPS at 4K/FHD