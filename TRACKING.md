# SoftISP — Project Tracking

## Current State: Vulkan ISP Pipeline (MNN backend)

GPU-accelerated camera ISP pipeline running on MNN's Vulkan backend.
3-dispatch fused pipeline: unpack_demosaic_fcs → ee_ldci → display.

### Performance (Vulkan, Snapdragon)
| Resolution | Latency | FPS |
|---|---|---|
| HD 1280×720 | 8.77 ms | **114** |
| FHD 1920×1080 | 20.09 ms | **50** |
| 4K 3840×2160 | 73.25 ms | **14** |

### Pipeline Profiles
| Profile | Stages | Ops |
|---|---|---|
| LITE | unpack→display | 4 |
| MED | unpack→ee→display | 5 |
| HEAVY | unpack→fcs→ee→ldci→display | 7 |
| PRO | All stages + extras | 10+ |

## Repo Layout

### `softisp/` (root)
- `vulkan_isp/` — GLSL compute shaders, Python ONNX generators, C++ test harnesses
- `cam-rust/` — Rust workspace with 10 crates

### `cam-rust/` crates
| Crate | Purpose |
|-------|---------|
| `cam-types` | Core types: Frame, FrameFormat, ToneParams, BlockDef |
| `cam-isp` | ISP engine: CpuEngine, MnnEngine, OnnxEngine, blocks, ONNX proto, profiles, ML engines |
| `cam-hal` | Hardware abstraction: ICameraAdapter trait, buffer management |
| `cam-hal-linux` | V4L2 adapter via `rscam` |
| `cam-hal-android` | Android Camera2 HAL3 implementation (camera3.h FFI) |
| `cam-core` | PipelineManager, ApplicationHolder, HAL bridge |
| `cam-onnx` | ONNX Runtime bindings (ort v2.0.0-rc.12) |
| `cam-motion` | MotionCompensator (placeholder) |
| `cam-binder` | Android AIDL-style binder HAL + ISP integration |
| `cam-app` | Binary entry, ONNX model generator |

### `MNN/` (fork)
- `tools/converter/source/optimizer/postconvert/IspChainFusion.cpp` — ISP fusion pass
- `tools/converter/source/optimizer/postconvert/isp_spirv_embedded.h` — Embedded SPIR-V
- `source/backend/vulkan/` — Vulkan backend (with concurrent session mutex fix)

## Converter Fusion Rules
| Rule | Pattern | Result |
|---|---|---|
| R1 | unpack_blc (Python Cast→Conv + Rust Concat→Conv) | `isp.unpack_blc` |
| R1b | Rust packed-int16 Concat→Conv | `isp.unpack_blc` (fallback) |
| R2 | Conv1x1 4ch→3ch | `isp.demosaic_ccm` |
| R3a | Conv3x3 group=3 | `isp.ee` |
| R3b | Mul+Add | `isp.fcs` |
| R4 | AvgPool+Sub+Mul+Add | `isp.ldci` |
| R5 | Pow+Clip | `isp.display` |
| R6 | Pow+Clip (alt pattern) | `isp.display` |
| R7 | fcs+display | `isp.fcs_display` |
| R8 | fcs+display (adjacent) | `isp.fcs_display` |
| R9 | ee+ldci → ee_ldci | `isp.ee_ldci` |
| R10 | unpack_blc+demosaic_ccm | `isp.unpack_demosaic` |
| R12 | unpack_demosaic+fcs | `isp.unpack_demosaic` (FCS fused in) |

## Key Files
| File | Purpose |
|---|---|
| `IspChainFusion.cpp` | Pass 1 (standard→Extra) + Pass 2 (chain→fused) |
| `isp_spirv_embedded.h` | SPIR-V for each Extra op type |
| `shader_unpack_demosaic.comp` | Bayer→RGB with BLC+WB+CCM+FCS |
| `shader_ee_ldci_fused.comp` | Edge enhancement + local contrast |
| `shader6_display_simple.comp` | sRGB gamma |
| `gen_isp_onnx_standard.py` | Python ONNX generator (standard ops) |
| `mnn_wrapper.cpp` | C FFI for MNN session/tensor ops |
| `test_e2e_isp_pipeline.rs` | E2E: ONNX→MNN→Vulkan→verify |

## Completed

### Core ISP Pipeline
- [x] CpuEngine — 11-stage software ISP (RawInput→Normalize→DPC→Gaussian→AWB→BLC→LSC→Demosaic→CCM→AE→Tone→Display)
- [x] MnnEngine — Vulkan GPU acceleration (3-dispatch fused pipeline)
- [x] OnnxEngine — ONNX Runtime inference (ort v2.0.0-rc.12)
- [x] Scene-adaptive ISP — Dark/Indoor/Sunset/Outdoor/Bright auto-classification
- [x] IspController — AWB/AE/CCM/Tone/Zone stats feedback

### GPU Acceleration
- [x] Vulkan ISP pipeline — HD 114 FPS, FHD 50 FPS, 4K 14 FPS
- [x] IspChainFusion converter pass — 12 fusion rules (R1-R12)
- [x] Concurrent Vulkan sessions — mutex on vkQueueSubmit
- [x] Named ISP params — blc/wb/ccm/fcs/ee/ldci/display attributes
- [x] FP16 output — Float16Rgb/Float16Bgra (halves GPU→CPU bandwidth)
- [x] Bayer pattern configurability — RGGB/GRBG/GBRG/BGGR
- [x] 8K support — ONNX generation + MNN conversion at 7680×4320
- [x] HDR merge block — Multi-exposure fusion with luminance weight maps
- [x] Vulkan→CPU auto-fallback — Graceful backend degradation

### SIMD Optimization
- [x] AVX2 backend — 8-wide f32 operations (normalize, CCM, AE gain, display)
- [x] SSE2 backend — 4-wide f32 operations
- [x] NEON/NEON-FP16/NEON-DOTPROD — ARM64 backends
- [x] Runtime detection — automatic best backend selection

### Camera HAL
- [x] V4L2 adapter — Linux capture via rscam
- [x] Android Camera2 HAL3 — camera3.h FFI, AHardwareBuffer
- [x] Binder HAL — AIDL-style provider/device/session/callbacks
- [x] ISP integration — IspCameraSession bridges camera → ISP pipeline
- [x] Android NDK ABI support — build.rs with NDK detection
- [x] Camera detection — V4L2 device enumeration
- [x] Platform-aware registration — Android/local mode

### ONNX & Model
- [x] ONNX model generator — Pure Rust proto encoder
- [x] ONNX Runtime wrapper — cam-onnx with ort v2.0.0-rc.12
- [x] E2E test harness — ONNX→MNN→Vulkan→verify
- [x] Pipeline profiles — LITE/MED/HEAVY/PRO presets

### ML & Control
- [x] GeneticOptimizer — GA for ISP parameter calibration
- [x] FastPredictor — Per-CCT-bin averaging
- [x] RegressionModel — 4-feature OLS regression
- [x] LearnerStore — Ring buffer + disk persistence
- [x] EIS — Gyro stabilization with warp grid
- [x] AF — Autofocus state machine
- [x] CCM Engine — Quadratic CCT interpolation

### Timestamp & Latency
- [x] Timestamp passthrough — ProcessParams.timestamp_ns → IspFrame
- [x] Latency tracking — prep/inference/total durations

### Examples
- [x] camera_isp.rs — Single-shot V4L2/test pattern → ISP
- [x] stream_isp.rs — Continuous streaming with FPS stats

## Remaining Work
- (none — all TODO items resolved)

## Testing
```bash
# Run all tests (224)
cargo test --lib

# E2E (serial, requires Vulkan)
cargo test --test test_e2e_isp_pipeline --features mnn -- --ignored --test-threads=1

# Profiles
cargo test --test test_profile_onnx --features mnn

# Binder tests
cargo test -p cam-binder

# Camera ISP example
cargo run --example camera_isp --features mnn -p cam-isp -- --width 640 --height 480

# Streaming example
cargo run --example stream_isp --features mnn -p cam-isp -- --width 640 --height 480 --fps 30
```
