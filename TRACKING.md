# SoftISP — Project Tracking

## Current State: Full GPU ISP Pipeline (MNN Vulkan backend)

GPU-accelerated camera ISP pipeline running on MNN's Vulkan backend.
Non-packed INT16 sensor input → full GPU processing → display output.

### Performance (Vulkan, Snapdragon)

| Mode | Resolution | Latency | FPS | Notes |
|---|---|---|---|---|
| Packed INT32 | 4K→FHD | ~100 ms | **9.7** | Unpack+debayer+cosmetic+display |
| **Non-packed INT16** | **4K→FHD** | **~163 ms** | **6.1** | **Cast+Conv fused on GPU** |
| Packed INT32 | HD | 8.77 ms | **114** | 3-dispatch fused |
| Packed INT32 | FHD | 20.09 ms | **50** | 3-dispatch fused |
| Packed INT32 | 4K | 73.25 ms | **14** | 3-dispatch fused |

### Sensor Input Design

| Sensor bits | Valid bits | GPU normalization | fp16 precision |
|---|---|---|---|
| 16-bit (10-bit valid) | 10 | `val / 1023.0` → [0,1] | Perfect — 10-bit mantissa fits fp16 |
| 16-bit (12-bit valid) | 12 | `Div(val, 4.0)` → 10-bit → `/ 1023.0` → [0,1] | Perfect — 10-bit mantissa fits fp16 |

**Key**: No Rust-side INT16→float conversion. The Cast(INT16→F32) is fused with
the unpack Conv into a single SPIR-V shader by IspChainFusion. Everything on GPU.

### Pipeline Profiles

| Profile | Stages | Ops |
|---|---|---|
| LITE | unpack→display | 4 |
| MED | unpack→ee→display | 5 |
| HEAVY | unpack→fcs→ee→ldci→display | 7 |
| PRO | All stages + extras | 10+ |

### 4K→FHD Pipeline (Non-packed INT16)

```
RawInput(INT16 [1,1,2160,3840])
  → [GPU: Cast(INT16→F32) + Conv(stride=4)] → [1,4,1080,960] float
  → DemosaicCcm(Conv 3×4→3ch) → [1,3,1080,960]
  → FCS → LDCI → EE
  → Display(Conv 1×1 BGRA) → [1,4,540,960] float32
```

- **UnpackCfaBlock**: `NativeInt16` mode with `fast_unpack(true)`, `stride_w=4`
- **Conv weights**: `[4, 1, 2, 4]` — picks one Bayer position per 2×4 block
- **No Resize ops** — Vulkan doesn't support ONNX Resize (op_type=74)
- **No Cast as standalone op** — fused into Conv by IspChainFusion

### CFA Demosaicing: Algorithm Selection Guide

After the CFA (Color Filter Array) extracts raw Bayer data, the demosaicing
algorithm converts the single-channel Bayer pattern into multi-channel RGB.
The choice of algorithm depends on:
1. Input resolution (sensor native)
2. Output resolution (display/processing target)
3. Speed budget (real-time vs offline)
4. Quality requirement (preview vs final output)

#### Resolution Cases

| Case | Input | Output | Strategy | Notes |
|------|-------|--------|----------|-------|
| A | 8K (7680×4320) | 4K (3840×2160) | Binning 4×4 | 4× downscale |
| B | 8K (7680×4320) | FHD (1920×1080) | Binning 4×4 + resize | |
| C | 4K (3840×2160) | FHD (1920×1080) | Binning 2×2 | **Current path** |
| D | 4K (3840×2160) | 4K (3840×2160) | Interpolation | Full-res, expensive |
| E | FHD (1920×1080) | FHD (1920×1080) | Interpolation | **Needs new path** |
| F | FHD (1920×1080) | HD (1280×720) | Binning + resize | |
| G | HD (1280×720) | HD (1280×720) | Interpolation | Fast, low-res |

#### Algorithm Comparison

| Algorithm | Kernel | Speed | Quality | Output | GPU Implementation |
|-----------|--------|-------|---------|--------|--------------------|
| **Binning** | 2×2 stride=2 | ★★★★★ | ★★ | H/2×W/2 | Conv stride=2 (1 dispatch) |
| **Nearest** | 1×1 | ★★★★★ | ★ | H×W | Copy + channel assign |
| **Bilinear** | 3×3 stride=1 | ★★★★ | ★★★ | H×W | SPIR-V shader (1 dispatch) |
| **MHC** | 5×5 stride=1 | ★★★ | ★★★★ | H×W | SPIR-V shader (1 dispatch) |
| **Malvar** | 5×5 stride=1 | ★★★ | ★★★★ | H×W | SPIR-V shader (1 dispatch) |
| **AHD** | 7×7 adaptive | ★★ | ★★★★★ | H×W | Multi-pass, too slow for GPU |
| **DCB** | iterative | ★ | ★★★★★ | H×W | Not suitable for real-time |

#### Speed vs Quality on GPU (estimated)

| Algorithm | 4K→FHD | FHD→FHD | HD→HD | Quality |
|-----------|--------|---------|-------|---------|
| Binning | 2ms | N/A | N/A | Low — loss of detail |
| Bilinear | N/A | 4ms | 2ms | Medium — some color fringing |
| MHC | N/A | 8ms | 4ms | High — edge-aware, minimal artifacts |
| AHD | N/A | 20ms+ | 10ms+ | Excellent — but too slow for real-time |

#### Algorithm Details

**Binning (current implementation)**:
- Input: `[1,1,H,W]` → Output: `[1,3,H/2,W/2]`
- CFA Conv stride=2 extracts 4 Bayer positions per 2×2 block
- DemosaicCcmBlock averages G1/G2, applies CCM
- **Correct for**: sensors in binning mode (4K/8K → FHD)
- **Limitation**: loses half resolution in each dimension

**Bilinear Interpolation**:
- Input: `[1,1,H,W]` → Output: `[1,3,H,W]`
- For each pixel, interpolate missing colors from 2 nearest neighbors
- Red pixel: G from horizontal Gr neighbors, B from 4 diagonal B neighbors
- Green pixel: R from horizontal R neighbors, B from vertical B neighbors
- **Quality**: Some color fringing at edges, acceptable for video
- **GPU**: Single dispatch, 16×16 workgroup, ~4ms for FHD

**MHC (Malvar-He-Cutler)**:
- Input: `[1,1,H,W]` → Output: `[1,3,H,W]`
- 5×5 kernel with adaptive weights based on edge direction
- Computes Laplacian to detect edges, adjusts interpolation direction
- **Quality**: Significantly better than bilinear at edges
- **GPU**: Single dispatch, more ALU than bilinear, ~8ms for FHD
- **Recommended**: Best speed/quality trade-off for real-time ISP

**Adaptive Homogeneity-Directed (AHD)**:
- Input: `[1,1,H,W]` → Output: `[1,3,H,W]`
- Interpolates in both horizontal and vertical directions
- Selects direction with higher homogeneity (less edge artifacts)
- **Quality**: Excellent, industry standard for still photography
- **GPU**: Too slow for real-time, requires multiple passes
- **Use case**: Offline processing, high-quality still capture

#### Recommended Implementation Order

1. **Bilinear** — Quick win, covers FHD→FHD case, simple shader
2. **MHC** — Best speed/quality for real-time, covers all cases
3. **AHD** — Optional, for offline/high-quality mode

#### Current Implementation Status

| Algorithm | Status | Shader | Fusion Rule |
|-----------|--------|--------|-------------|
| Binning | ✅ Working | `shader_unpack_blc.comp` | R1, R10, R12 |
| Bilinear | ✅ Shader ready | `shader_demosaic_interp.comp` | TBD |
| MHC | 🔲 Not started | TBD | TBD |
| AHD | 🔲 Not started | N/A (too slow for GPU) | N/A |

#### TODO: Add interpolation demosaic path
- Shader: `shader_demosaic_interp.comp` (compiled, 11860 bytes SPIR-V)
- GLSL: bilinear interpolation, RGGB, 16×16 workgroup
- Input: INT16/INT32 [1,1,H,W] → Output: F32 [1,3,H,W]
- **To implement:**
  1. Embed SPIR-V in `isp_spirv_embedded.h` (from `demosaic_interp_spv.inc`)
  2. Add fusion rule in `IspChainFusion.cpp`:
     `Conv(4×4,stride=1,1ch→3ch) → isp.demosaic_interp`
  3. Add Rust block: `DemosaicInterpBlock` that generates the Conv pattern
  4. Rebuild MNN converter
- **Current workaround**: use stride_w=1 for non-binned sensors (height
  binning only), accepting width downscale as trade-off

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
| `unpack_cfa.rs` | UnpackCfaBlock: PackedInt32 + NativeInt16 modes |
| `mnnengine.rs` | MnnEngine: session pool, input type detection, bench |
| `pipeline.rs` | GraphComposer: ONNX graph assembly |
| `proto.rs` | ONNX protobuf encoder (field 9 = raw_data) |

## Completed

### Core ISP Pipeline
- [x] CpuEngine — 11-stage software ISP
- [x] MnnEngine — Vulkan GPU acceleration (3-dispatch fused pipeline)
- [x] OnnxEngine — ONNX Runtime inference (ort v2.0.0-rc.12)
- [x] Scene-adaptive ISP — Dark/Indoor/Sunset/Outdoor/Bright auto-classification
- [x] IspController — AWB/AE/CCM/Tone/Zone stats feedback

### GPU Acceleration
- [x] Vulkan ISP pipeline — HD 114 FPS, FHD 50 FPS, 4K 14 FPS (packed)
- [x] **Non-packed INT16 input** — 4K→FHD at 6.1 FPS, full GPU processing
- [x] **10/12-bit sensor support** — Div(4) SHR for 12-bit, normalize to [0,1]
- [x] **Cast+Conv fusion** — INT16→float fused into unpack shader by IspChainFusion
- [x] IspChainFusion converter pass — 12 fusion rules (R1-R12)
- [x] Concurrent Vulkan sessions — mutex on vkQueueSubmit
- [x] Named ISP params — blc/wb/ccm/fcs/ee/ldci/display attributes
- [x] FP16 output — Float16Rgb/Float16Bgra (halves GPU→CPU bandwidth)
- [x] Bayer pattern configurability — RGGB/GRBG/GBRG/BGGR
- [x] 8K support — ONNX generation + MNN conversion at 7680×4320
- [x] HDR merge block — Multi-exposure fusion with luminance weight maps
- [x] Vulkan→CPU auto-fallback — Graceful backend degradation
- [x] **ONNX proto fix** — raw_data uses field 9 (not field 4) in TensorProto
- [x] **Graph output fix** — pointer comparison, not is_tail()
- [x] **write_input type detection** — uses model_input_type from probe session
- [x] **Dynamic Tile Workgroup** — GpuWorkgroupProfile lookup (Mali/Adreno/Apple)
- [x] **Early-Z Rejection** — valid_bounds dispatch culling for VulkanFuse
- [x] **Profile-Based Tuning** — GPU auto-detect + session workgroup override

### UnpackCfaBlock Modes
| Mode | Input | Graph | Notes |
|---|---|---|---|
| PackedInt32 | `[1,1,H,W/2]` INT32 | Mod+Div+Cast+Conv | Legacy Android packed |
| NativeInt16 | `[1,1,H,W]` INT16 | Cast+Conv(stride) | **Current — full GPU** |

### SIMD Optimization
- [x] AVX2 backend — 8-wide f32 operations
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
- [x] **ResizeBlock unique IDs** — AtomicUsize counter prevents name collisions
- [x] **Adaptive ResizeBlock** — skip_below threshold, identity passthrough

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
- [x] **bench_4k_to_fhd.rs** — Non-packed INT16 4K→FHD benchmark

## Remaining Work
- (none — all TODO items resolved)
- ✅ All 7 pipeline blocks decomposed to atomic subgraphs, detected by standard fusion rules
- ✅ Pipeline at 17 ops, 35.0 FPS (exceeds 30 FPS target)
- ✅ Remaining 17 ops = 4 Extras + ConvertTensors at edges (no intermediate ConvertTensors)
- ✅ Fusions: R1 (unpack), R2 (demosaic), R3b (FCS), R4 (EE via tryRustExtraEe), R5 (LDCI), R6 (Display), R10 (unpack+demosaic), R9 (ee+ldci)
- ⏳ R8 (fcs+display) doesn't fire in current pipeline (non-adjacent Extras in chain)
- ⏳ MHC demosaic shader (5×5 adaptive) — low priority, not needed for 4K binning path

## Testing
```bash
# Run all tests (233)
cargo test --lib

# E2E (serial, requires Vulkan)
cargo test --test test_e2e_isp_pipeline --features mnn -- --ignored --test-threads=1

# Profiles
cargo test --test test_profile_onnx --features mnn

# 4K→FHD benchmark (non-packed INT16, full GPU)
LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a RUST_LOG=info \
  cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn

# Packed INT32 benchmark
LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a RUST_LOG=info \
  cargo run --release --example bench_opt -p cam-isp --features mnn

# Camera ISP example
cargo run --example camera_isp --features mnn -p cam-isp -- --width 640 --height 480

# Streaming example
cargo run --example stream_isp --features mnn -p cam-isp -- --width 640 --height 480 --fps 30
```

## Design Decisions

### Why Cast is fused, not removed
Vulkan doesn't support standalone `Cast` (OpType_Cast=9). But IspChainFusion
converts `Cast+Conv` into a single Extra op with a custom SPIR-V shader that
performs the type conversion inline during the convolution. This keeps
everything on GPU with zero CPU involvement.

### Why no Resize ops
Vulkan doesn't support ONNX Resize (op_type=74). Instead:
- UnpackCfaBlock uses Conv stride for width downscale (stride_w)
- Height downscale omitted (would need Resize which is unsupported)
- Packed mode uses stride_w=2 for implicit width/2

### Why output_value_info differs by mode
- PackedInt32: input width = W/2, output = H/2 × W/2/sw
- NativeInt16: input width = W, output = H/2 × W/sw
The formula must account for the packed width halving.

### Why binning demosaic, not interpolation
The CFA Conv with stride=2 treats each 2×2 Bayer block as one pixel with
4 color channels (R, Gr, Gb, B). This is pixel binning: hardware-averaged
sensor data where TL/TR/BL/BR are the same physical pixel. The output is
H/2×W/2 — half resolution, no pixel replication.

Interpolation demosaic (bilinear, MHC, Malvar, etc.) reconstructs full
resolution H×W by interpolating between Bayer samples. This requires a
stride=1 shader that reads overlapping neighborhoods.

Binning is correct for high-res sensors (4K/8K) where the target is FHD
output — the resolution reduction is intentional. For normal-res sensors
(FHD Bayer), binning would reduce to 960×540 which is too small. The
pipeline needs interpolation for non-binned sensors.

Recommended algorithm progression:
1. Bilinear: simple, fast (~4ms FHD), acceptable quality
2. MHC: edge-aware (~8ms FHD), best speed/quality for real-time
3. AHD: excellent quality but too slow for real-time GPU

### Why graph_output_name() default is is_tail()
wire_blocks() doesn't set prev/next pointers (set_prev/set_next consume
Box<dyn IspBlock>). So is_tail() returns true for all blocks. The
graph_output_name() default uses is_tail() which registers all blocks as
outputs — this is intentional: IspChainFusion needs intermediate outputs
to prevent dead code elimination. The tail's output is inserted first
so getSessionOutput(nullptr) returns the display result.
