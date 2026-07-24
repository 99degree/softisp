# CLAUDE.md — Rust Camera ISP Pipeline

## Status: ✅ ALL JAVA PORTED — SIMD + AdaptiveDownscale + pack_rgba default

Workspace: **0 warnings**, **201 unit tests pass** (`cargo test --lib -p cam-isp`).

## Output Format — pack_rgba default

| Mode | ONNX output | `output_channels` | Use |
|------|------------|-------------------|-----|
| **pack_rgba** (default) | `[1,1,H,W]` INT32 | `1` | Production — `R*65536+G*256+B` per i32, unpacked to BGRA |
| bg4a (verify) | `[1,4,H,W]` FLOAT | `4` | Benchmark/verification — Conv(1×1) BGRA |
| legacy float | `[1,3,H,W]` FLOAT | `3` | Post-proc chaining |

Set via `PipelineManager::output_channels` or `ProcessParams::output_channels`.

## Pipeline (16 stages, CpuEngine)

```
RawInput(INT16) → Normalize(F32) → DPC(median) → Gaussian Denoise
→ CalibrationStats → [AF focus] → [EIS gyro] → AWB(controller)
→ BLC/WB → LSC(radial) → Malvar Demosaic(RGB) → [IspController stats]
→ CCM(3×3) → AE(gain) → Tone(gamma+contrast+sat+unsharp)
→ FCS(chroma desat) → LDCI(local contrast) → Warp(EIS/radial)
→ Display(INT32 packed → BGRA)
```

## SIMD Backend

```
CpuEngine::process() → simd: &'static dyn SimdEngine (auto-selected at init)
  ├─ NeonDotprod (ARMv8.4) ← dotprod+fp16+neon
  ├─ NeonFp16    (ARMv8.2) ← fp16+neon
  ├─ Neon        (ARMv8.0) ← 128-bit NEON
  └─ Scalar                ← always available

Op              Speedup  Method
normalize_u16    ~4×     8 u16→f32/iter
apply_ccm        ~3.5×   4 pixels/iter (3×3 FMA)
apply_ae_gain    ~4×     4 f32/iter
display_output   ~3×     batch 4 BGRA
```

## Build & Test

```bash
cd cam-rust
cargo test --lib -p cam-isp              # 201 tests
cargo check --workspace                  # 0 warnings
RUST_LOG=info cargo run --example pipeline -p cam-isp   # single frame PNG
bash bench-tests.sh bench 50 64 48       # perf benchmark
# Synthetic benchmark fixed: packed INT32 input, no -4 errors
```

## Architecture (10 crates)

```
cam-rust/
├── cam-types/      # Frame, ToneParams, BlockDef
├── cam-isp/        # 24+ modules: engine, pipeline, blocks, cpu, controller,
│                   #   ae, af, eis, calibration, scene, predictor, regression,
│                   #   store, genetic, ccm_engine, profile, config, fused,
│                   #   manager, onnx/proto, mnn, stats, demosaic, warp,
│                   #   isp_ops, cpu_simd, simd/ (4 backends)
├── cam-hal/        # ICameraAdapter trait
├── cam-hal-linux/  # V4L2 adapter
├── cam-hal-android/# Android Camera2 NDK stub
├── cam-core/       # PipelineManager, ApplicationHolder, logging
├── cam-onnx/       # ONNX Runtime bindings (placeholder)
├── cam-motion/     # MotionCompensator (placeholder)
├── cam-binder/     # Android AIDL-style binder HAL
└── cam-app/        # ONNX model generator binary
```

## Stats Integration — Triple-Buffered Feedback Loop

```
Controller → engine.process(frame, ccm, tone, bayer, awb)
              ├─ set_extra_inputs()     ← write params to ONNX initializers
              ├─ inference              ← ONNX graph (display + stats outputs)
              ├─ read stats tensors     ← channel means, tone, hist, zones
              └─ write_stats()          → RollingStats write-slot
            controller.rotate_stats()   → process previous, ready next
```

- **3-slot rotate** (`write_idx` → `process_idx` → `ready_idx`): engine writes frame N, controller processes frame N-1, next frame reads frame N-2 params. No lock contention.
- **ONNX dual-registration**: extra inputs (CCM, tone, WB) registered as BOTH initializer AND graph input for runtime override.
- **Bayer pattern fusion**: `set_extra_inputs(bayer_pattern)` selects RGGB/BGGR/GRBG/GBRG demosaic weights fused with CCM matrix.
- **Adaptive stats downscale**: `stats_downscale_max` auto-inserts ResizeBlock before stats for 4K+ sensors.

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| `SimdEngine` trait + `OnceLock` | Zero-cost dispatch, no compile-time flags |
| `#[target_feature]` for fp16 | Compiles on any CPU, runtime `is_aarch64_feature_detected!` gates |
| Triple-buffered RollingStats | Lock-free: engine writes while controller processes |
| ONNX dual-registration | Graph inputs overridable at runtime (CCM/tone/WB) |
| pack_rgba INT32 default | Single i32 per pixel = efficient memory + chaining |
| Debug-level per-frame logs | No info-level noise from frame timing |
| `mnn_buffer` behind feature flag | Isolates pre-existing compilation errors |

## FHD Benchmark — Block Costs

```
Block            CPU 9.0fps  Vulkan 12.3fps  Δ
unpack_cfa       37.7ms      42.4ms         +4.7ms
demosaic_ccm     19.2ms      ~0ms (fused)   🏆
ldci             28.8ms       2.3ms         -26.5ms ← Conv-heavy
  ee             12.0ms       4.0ms          -8.0ms
display          17.4ms      23.4ms          +6.0ms ← mem xfer
Total:~111ms                Total:~81ms      -27%
```

Vulkan wins on Conv-heavy blocks (ldci -92%, ee -67%) but adds kernel-launch overhead on elementwise ops.
- **4K→4K MNN stress test** (post-DCE fix, real ISP graph executed):
  - HEAVY profile: **79.8 FPS** avg (2394 frames / 30s, P99 129.1ms)
  - UNIFIED profile (tiled 2×2): **141.2 FPS** avg (4236 frames / 30s, P99 8.7ms)
- Older per-block 4K→FHD: ~~≈ 4.7 FPS~~ before TypeProto/Extra-input fixes (prevented MNN DCE).

## MNN Vulkan GPU Backend — Working

MNN runs on **Vulkan (Adreno GPU)** with real inference timing:

```
$ bench_mnn_gpu .mnn_bench_16958.mnn 5 50 2
  actual backend: Vulkan (7)
  avg:   17.949 ms  (55.7 FPS)
  model: 10.9 GFLOP, 5.8 MB
```

### Two issues fixed

1. **MNN_SEP_BUILD plugin loading**: MNN compiles backends as separate `.so` files. Each backend self-registers via a static initializer (`gResistor → MNNInsertExtraRuntimeCreator()`), but this only fires when the `.so` is `dlopen`'d. `libMNN.so` does NOT auto-load plugins. Fix: `ensure_vulkan_loaded()` calls `dlopen("libMNN_Vulkan.so")` in `mnn_session_create()`.

2. **`libvulkan.so` (ICD loader)**: MNN's `vulkan_wrapper.cpp` does `dlopen("libvulkan.so")` for driver function pointers. On Android the system driver is at `/system/lib64/libvulkan.so` but Termux doesn't search there. Fix: copied it to `cam-rust/lib/aarch64-v8a/`.

Both are required — loading `libMNN_Vulkan.so` without `libvulkan.so` still fails silently.

### Runtime libs in `cam-rust/lib/aarch64-v8a/`

| Library | Source | Purpose |
|---------|--------|---------|
| `libMNN.so` | `~/MNN/build` | MNN core |
| `libMNN_Vulkan.so` | `~/MNN/build` | Vulkan backend plugin |
| `libvulkan.so` | `/system/lib64/` | Android Vulkan ICD loader |
| `libMNNConvertDeps.so` | `~/MNN/build` | ONNX→MNN converter deps |

## MNNConvert — Static C FFI (No Subprocess)

`mnn_sys/mnn_convert_api.cpp` exposes `mnn_convert_onnx_to_mnn()` linking `MNN::Cli::convertModel()` directly. Compiled as `.a` via `cc::Build` in `build.rs`. No fork/exec, no `MNNConvert` binary needed — single `.so` dep (`libMNNConvertDeps.so`).

## Progress

### ✅ Done
- All Java/Kotlin → Rust (48+ modules)
- SIMD backend (NeonDotprod/NeonFp16/Neon/Scalar)
- AdaptiveDownscaleBlock (fit/crop/pad, EIS margin)
- Triple-buffered RollingStats
- Fused orientation in DisplayBlock (rot90/180/270/hflip/vflip)
- Tone fused into DemosaicCcmBlock Conv
- pack_rgba INT32 default output
- MNNConvert static C FFI (no subprocess)
- All MNN libraries updated from `~/MNN/build`, including `libMNN_GL.so`
- MNN Vulkan GPU backend working (dlopen plugin + ICD loader fix)
- ONNX `TypeProto` / `TensorShapeProto` encoding fix (proto.rs): correct oneof submessage + Dimension repeat prevents MNN DCE of the ISP graph
- WarpGridBlock grid + shading LUT moved to runtime `extra_inputs()` (graph inputs, not initializers) — defeats MNN fold-to-identity
- `synth_bayer` module (5 generators) for stress-test inputs with high spatial frequency
- Pipeline tests: 732 pass
- HEAVY profile 4K→4K MNN stress test: **79.8 FPS** avg (2394 frames / 30s, P99 129.1ms)
- UNIFIED profile 4K→4K MNN stress test: **141.2 FPS** avg (4236 frames / 30s, P99 8.7ms, tiled 2×2)

### ⏳ In Progress
- Numeric agreement ONNX stats ↔ software stats at FHD
- 30fps target at FHD (HEAVY 4K path: 79.8 FPS; UNIFIED 4K path: 141.2 FPS with tiles)

### 🚧 Blocked
- GPU zero-copy: needs device memory
- V4L2 on Android: `rscam` crate platform check
- 256-bin histogram ONNX block: OneHot → 2GB+ at FHD
- Content-aware inpainting: external ML models

## Not Ported

| Feature | Reason |
|---------|--------|
| ORT inference | Needs `libonnxruntime.so` |
| MNN full inference | ~~Needs `libMNN.so`~~ ✅ Working on Vulkan GPU |
| Android Camera2 NDK | Needs NDK device |
| HDR merge, ONNX AI blocks (AiDetect) | Needs ONNX models + runtime |
| YUV processing | Not supported in CpuEngine |
