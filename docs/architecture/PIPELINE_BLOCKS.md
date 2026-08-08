# Pipeline Blocks — ONNX Input/Output Reference

All ISP pipeline blocks implement the `IspBlock` trait and contribute ONNX graph fragments that get fused by `GraphComposer` into a single ONNX model executed by MNN/Vulkan.

## Tensor Convention

- **NCHW layout**: All tensors are `[N, C, H, W]` where `N=1` (batch).
- **Float32**: All intermediate tensors are `float32` (1).
- **Channel order**: RGB = `[0:R, 1:G, 2:B]`. Bayer = `[R, Gr, Gb, B]` or `[Gr, R, B, Gb]` depending on pattern.
- **Pixel range**: `[0.0, 1.0]` for float RGB unless noted otherwise.
- **INT16 raw**: 16-bit unsigned, only lower 10 or 12 bits valid, **not packed**.
- **INT32 packed**: True zero-copy input, 4×16-bit values packed per 32-bit word.

---

## Block Categories

### 1. Input & Unpack

#### `RawInputBlock`
| | |
|---|---|
| **ID** | `raw_input` |
| **Role** | Pipeline head — feeds raw sensor data into ONNX graph |
| **Input** | External buffer (no ONNX input tensor) |
| **Output** | `{ns}/out` → `[1, 1, H, W]` or `[1, 1, H, W/4]` |
| **Elem type** | `1` (FLOAT) or `6` (INT32) for packed |
| **Notes** | `concrete_h` = `(target_width × 9/16)` for 16:9. Packed mode uses half-width. |

#### `UnpackBlock`
| | |
|---|---|
| **ID** | `unpack` |
| **Role** | Unpack INT32→FLOAT, extract 10/12-bit values |
| **Input** | `{prev}/out` → `[1, 1, H, W/4]` INT32 |
| **Output** | `{ns}/out` → `[1, 4, H, W]` FLOAT |
| **ONNX ops** | `Reshape → Div(1023) → Clip(0,1) → Reshape` |
| **Notes** | Each INT32 word contains 4×10-bit pixel values. |

#### `UnpackCfaBlock`
| | |
|---|---|
| **ID** | `unpack_cfa` |
| **Role** | Fused unpack + normalize + CFA + BLC (all-in-one) |
| **Input** | `{prev}/out` → `[1, 1, H, W/4]` INT32 |
| **Output** | `{ns}/out` → `[1, 4, H/2, W/2]` FLOAT (Bayer) |
| **Notes** | Replaces Unpack + Normalize + CFA + BlcBlock. Saves 3 sessions. |

#### `NormalizeBlock`
| | |
|---|---|
| **ID** | `normalize` |
| **Role** | Normalize raw pixel values to [0,1] |
| **Input** | `{prev}/out` → `[1, 1, H, W]` FLOAT (raw) |
| **Output** | `{ns}/out` → `[1, 1, H, W]` FLOAT `[0,1]` |
| **ONNX ops** | `Div(1023)` or `Div(4095)` depending on bit depth |

---

### 2. Black Level & White Balance

#### `BlcBlock`
| | |
|---|---|
| **ID** | `blc` |
| **Role** | Subtract black level per Bayer channel |
| **Input** | `{prev}/out` → `[1, 4, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, 4, H, W]` FLOAT |
| **ONNX ops** | `Sub(black_levels)` |
| **Params** | `black_levels: [f32; 4]` — per-channel (R, Gr, Gb, B) |

#### `Blc50Block`
| | |
|---|---|
| **ID** | `blc50` |
| **Role** | Dark frame subtraction (BLC50) |
| **Input** | `{prev}/out` → `[1, 4, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, 4, H, W]` FLOAT |
| **ONNX ops** | `Sub(dark_frame) → Max(0) → Min(1)` |
| **Params** | `dark_frame: [f32; 4]` — dark frame reference values |

#### `BayerWbBlock`
| | |
|---|---|
| **ID** | `bayer_wb` |
| **Role** | Apply white balance gains in Bayer domain |
| **Input** | `{prev}/out` → `[1, 4, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, 4, H, W]` FLOAT |
| **ONNX ops** | `Mul(wb_gains) → Clip(0,1)` (identity passthrough when gains=[1,1,1,1]) |
| **Params** | `gains: [f32; 4]` — per-channel (R, Gr, Gb, B) |

---

### 3. Color Filter Array & Demosaic

#### `CfaBlock`
| | |
|---|---|
| **ID** | `cfa` |
| **Role** | Arrange raw pixels into 4-channel Bayer pattern |
| **Input** | `{prev}/out` → `[1, 1, H, W]` FLOAT (raw) |
| **Output** | `{ns}/out` → `[1, 4, H/2, W/2]` FLOAT |
| **ONNX ops** | `SpaceToDepth(2) → Reshape` |
| **Notes** | Output channels: [R, Gr, Gb, B] based on Bayer pattern. |

#### `DemosaicBlock`
| | |
|---|---|
| **ID** | `demosaic` |
| **Role** | Convert Bayer to RGB via learned interpolation |
| **Input** | `{prev}/out` → `[1, 4, H, W]` FLOAT (Bayer) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Conv(1×1, 4→3)` — learned demosaic weights |
| **Notes** | Output height = input height (2× upsample built into Conv). |

#### `DemosaicCcmBlock`
| | |
|---|---|
| **ID** | `demosaic_ccm` |
| **Role** | Fused demosaic + color correction matrix |
| **Input** | `{prev}/out` → `[1, 4, H, W]` FLOAT (Bayer) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Conv(1×1, 4→3)` — fused demosaic + CCM weights |
| **Notes** | Saves one full-frame Mul+Add pass vs separate DemosaicBlock + CcmBlock. |

#### `DemosaicInterpBlock`
| | |
|---|---|
| **ID** | `demosaic_interp` |
| **Role** | Bilinear interpolation demosaic (no learning) |
| **Input** | `{prev}/out` → `[1, 4, H, W]` FLOAT (Bayer) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Conv(1×1, 4→3)` — fixed bilinear interpolation weights |

#### `BayerDemosaicBlock`
| | |
|---|---|
| **ID** | `bayer_demosaic` |
| **Role** | Multi-quality Bayer demosaic |
| **Input** | `{prev}/out` → `[1, 4, H, W]` FLOAT (Bayer) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Conv(1×1, 4→3)` — quality-dependent weights |

---

### 4. Color Correction

#### `CcmBlock`
| | |
|---|---|
| **ID** | `ccm` |
| **Role** | Apply 3×3 color correction matrix |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Reshape → MatMul(ccm_matrix) → Reshape → Clip(0,1)` |
| **Params** | `matrix: [[f32; 3]; 3]` — 3×3 CCM, `offsets: [f32; 3]` |

---

### 5. Tone & Contrast

#### `ToneBlock`
| | |
|---|---|
| **ID** | `tone` |
| **Role** | Tone mapping (contrast + brightness) |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Mul(contrast) → Add(brightness) → Clip(0,1)` |
| **Params** | `contrast: f32`, `brightness: f32`, `gamma: f32` |

#### `GammaBlock`
| | |
|---|---|
| **ID** | `gamma` |
| **Role** | Gamma correction (power curve) |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Max(0) → Min(1) → Add(eps) → Log → Mul(1/gamma) → Exp` |
| **Params** | `gamma: f32` (typically 2.2) |

#### `HdrToneBlock`
| | |
|---|---|
| **ID** | `hdr_tone` |
| **Role** | HDR tone mapping with local adaptation |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT |
| **ONNX ops** | `ReduceMean → Sub → Mul → Add → Clip` |

---

### 6. Noise Reduction

#### `FcsBlock`
| | |
|---|---|
| **ID** | `fcs` |
| **Role** | False color suppression |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Sub → ReLU6 → Mul` |
| **Notes** | Fused as `isp.fcs` opset (Sub+ReLU6) by MNN fusion rule R3c. |

#### `LdciBlock`
| | |
|---|---|
| **ID** | `ldci` |
| **Role** | Local dynamic contrast improvement |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `AveragePool → Mul → Add → Clip` |

#### `EeBlock`
| | |
|---|---|
| **ID** | `ee` |
| **Role** | Edge enhancement / unsharp mask |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Sub → Mul(strength) → Add` |
| **Params** | `strength: f32` |

#### `BilateralBlock`
| | |
|---|---|
| **ID** | `bilateral` |
| **Role** | Edge-preserving noise reduction |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `AveragePool (spatial) → Mul → Add` (simplified bilateral) |
| **Params** | `sigma_spatial: f32`, `sigma_range: f32`, `kernel_size: u32` |

#### `GrayscaleBlock`
| | |
|---|---|
| **ID** | `grayscale` |
| **Role** | Convert RGB to grayscale for pyramid/EIS |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 1, H, W]` FLOAT |
| **ONNX ops** | `Mul(weights) → ReduceSum` |

---

### 7. Saturation & Color

#### `SaturationBlock`
| | |
|---|---|
| **ID** | `saturation` |
| **Role** | Adjust color saturation + vibrance |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Grayscale → Sub → Mul(saturation) → Add` |
| **Params** | `saturation: f32` (1.0 = neutral), `vibrance: f32` |

#### `ColorSpaceBlock`
| | |
|---|---|
| **ID** | `rgb_to_hsv` / `hsv_to_rgb` / etc. |
| **Role** | Color space conversion |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT |
| **Conversions** | `RgbToHsv`, `HsvToRgb`, `RgbToLab`, `LabToRgb`, `RgbToYCbCr`, `YCbCrToRgb` |
| **ONNX ops** | `ReduceMax → ReduceMin → Sub → Div → Add` (max/min approach) |

#### `AutoContrastBlock`
| | |
|---|---|
| **ID** | `auto_contrast` |
| **Role** | Automatic contrast stretching |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT |
| **ONNX ops** | `ReduceMin → ReduceMax → Sub → Div → Mul → Add` |

---

### 8. Lens Corrections

#### `VignettingBlock`
| | |
|---|---|
| **ID** | `vignetting` |
| **Role** | Correct lens brightness falloff |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Mul(gain_map)` |
| **Params** | Pre-computed radial gain map `[1, 1, H, W]` |

#### `ChromaticAberrationBlock`
| | |
|---|---|
| **ID** | `chromatic_aberration` |
| **Role** | Correct color fringing at edges |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `GridSample` (per-channel warp) |

#### `LensShadingBlock`
| | |
|---|---|
| **ID** | `lens_shading` |
| **Role** | Correct lens shading (LSC) |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `Mul(lsc_table)` |

---

### 9. Geometric Transforms

#### `WarpBlock`
| | |
|---|---|
| **ID** | `warp` |
| **Role** | Geometric distortion correction (GDC) + EIS |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Input (grid)** | `{grid}` → `[1, 2, H, W]` FLOAT (flow field) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `GridSample(mode=bilinear, border=border)` |

#### `GpuWarpBlock`
| | |
|---|---|
| **ID** | `gpu_warp` |
| **Role** | GPU-accelerated warp with runtime parameters |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **Input (params)** | `k1, k2, k3` (distortion coefficients) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (RGB) |
| **ONNX ops** | `GridSample` (grid computed on GPU) |

#### `RotateBlock`
| | |
|---|---|
| **ID** | `rotate` |
| **Role** | 90°/180°/270° rotation or flip |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, 3, H', W']` FLOAT |
| **ONNX ops** | `Transpose(perm)` |
| **Modes** | 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip |

#### `FlipBlock`
| | |
|---|---|
| **ID** | `flip` |
| **Role** | Horizontal/vertical flip |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT |
| **ONNX ops** | `Transpose` or `GridSample` |

---

### 10. Resize & Scale

#### `AdaptiveDownscaleBlock`
| | |
|---|---|
| **ID** | `adaptive_downscale` |
| **Role** | Smart resize with optional padding/cropping |
| **Input** | `{prev}/out` → `[1, C, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, C, H', W']` FLOAT |
| **ONNX ops** | `Resize(mode=nearest)` or `Crop + Resize` |
| **Modes** | `"edge"` (pad with black), `"crop"` (crop to fit) |

#### `DynResizeBlock`
| | |
|---|---|
| **Id** | `dyn_resize` |
| **Role** | Dynamic resize to target dimensions |
| **Input** | `{prev}/out` → `[1, C, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, C, H', W']` FLOAT |
| **ONNX ops** | `Resize(mode=bilinear)` |

#### `ResizeBlock`
| | |
|---|---|
| **ID** | `resize` |
| **Role** | Fixed-factor resize |
| **Input** | `{prev}/out` → `[1, C, H, W]` FLOAT |
| **Output** | `{ns}/out` → `[1, C, H', W']` FLOAT |
| **ONNX ops** | `Resize` |

---

### 11. Display & Output

#### `DisplayBlock`
| | |
|---|---|
| **ID** | `display` |
| **Role** | Final output format conversion |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT RGB `[0,1]` |
| **Output** | `{ns}/out` — format-dependent: |

| Output Format | Output Shape | Description |
|---|---|---|
| `FloatRgb` | `[1, 3, H, W]` f32 | Identity Mul(1.0) + optional gamma |
| `FloatBgra` | `[1, 4, H, W]` f32 | Conv(1×1) BGR permutation + alpha=255 |
| `Rgba` | `[1, 4, H, W]` f32 | Conv(1×1) RGBA permutation + alpha=255 |
| `Argb` | `[1, 4, H, W]` f32 | Conv(1×1) ARGB permutation + alpha=255 |
| `Abgr` | `[1, 4, H, W]` f32 | Conv(1×1) ABGR permutation + alpha=255 |
| `Rgb` | `[1, 3, H, W]` f32 | Conv(1×1) identity, no alpha |
| `Bgr` | `[1, 3, H, W]` f32 | Conv(1×1) BGR permutation |
| `PackedRgb` | `[1, 1, H, W]` INT32 | Mul(255) → Pack per-pixel |
| `Argb8888` | `[1, 1, H, W]` INT32 | `isp.argb_convert` opset |
| `Yuv420` | 3-plane | `isp.yuv420_convert` opset |

**ONNX ops**: `Pow(1/2.4) → Clip(0,1)` (gamma), `Conv(1×1) → Mul(255) → Add(alpha)` (format)

---

### 12. Stats & Analysis (Aux Blocks)

#### `IdentityBlock`
| | |
|---|---|
| **ID** | `identity_{name}` |
| **Role** | Passthrough — no-op, used for aux hooks |
| **Input** | `{prev}/out` |
| **Output** | `{ns}/out` — same tensor |
| **ONNX ops** | None (wired directly) |

#### `ToneStatsBlock` (stats)
| | |
|---|---|
| **ID** | `tone_stats` |
| **Role** | Luminance statistics for tone mapping feedback |
| **Input** | `{aux_hook}/out` → `[1, 1, H, W]` FLOAT (luma) |
| **Output** | `{ns}/out` → `[1, 1, 16]` FLOAT (mean/min/max/clip/shadow stats) |
| **ONNX ops** | `ConvertTensor → Conv(1×1) → ConvertTensor → ReduceMean → ReduceMin → ReduceMax → Const → BinaryOp → ConvertTensor → ReduceSum → Const → BinaryOp → ConvertTensor → ReduceSum → Size → Concat` (16 ops) |
| **Matcher** | `isp.tone_stats` exact pattern |

#### `CoarseHistogramBlock` (stats)
| | |
|---|---|
| **ID** | `histogram` |
| **Role** | Coarse luminance histogram for AE exposure estimation |
| **Input** | `{aux_hook}/out` → `[1, 1, H, W]` FLOAT (luma) |
| **Output** | `{ns}/out` → `[1, 1, N]` FLOAT (N-bin histogram) |
| **ONNX ops** | Variable-length: `ConvertTensor → Conv(1×1) → [Const → BinaryOp → ConvertTensor → Reduction] × N` → `Const → Concat` |
| **Matcher** | `isp.histogram` prefix-match (6-ops prefix + bin pattern + 5-ops tail) |
| **Notes** | Default N=16 bins → 95 ops. Bin count is configurable at build time. |

#### `CalibrationBlock` (stats)
| | |
|---|---|
| **ID** | `calibration` |
| **Role** | Sensor calibration statistics for AWB/AE/AF |
| **Input** | `{aux_hook}/out` → `[1, 4, H, W]` FLOAT (Bayer) |
| **Output** | `{ns}/out` → `[1, 16]` FLOAT (variance/range/lum/noise stats) |
| **ONNX ops** | `ReduceMean → UnaryOp(Square) → ReduceMean → UnaryOp → BinaryOp(Sub) → ReduceMin → ReduceMax → BinaryOp → Const → BinaryOp → BinaryOp → ReduceMean × 4 → Concat → Const → Reshape` (18 ops) |
| **Matcher** | `isp.calibration` exact pattern |

#### `ZoneStatsBlock` (stats)
| | |
|---|---|
| **ID** | `zone_stats` |
| **Role** | Spatial zone statistics for AF/AE |
| **Input** | `{aux_hook}/out` → `[1, 1, H, W]` FLOAT (luma) |
| **Output** | `{ns}/out` → `[1, 1, Z]` FLOAT (per-zone means) |
| **ONNX ops** | `AveragePool` (single op, zone-based pooling) |

#### `ChannelMeansBlock` (stats)
| | |
|---|---|
| **ID** | `channel_means` |
| **Role** | Per-channel mean values for AWB |
| **Input** | `{aux_hook}/out` → `[1, 4, H, W]` FLOAT (Bayer) |
| **Output** | `{ns}/out` → `[1, 4]` FLOAT (per-channel means) |
| **ONNX ops** | `ReduceMean` (single op, global reduce) |

---

### 13. Multi-Frame

#### `HdrMergeBlock`
| | |
|---|---|
| **ID** | `hdr_merge` |
| **Role** | Merge multiple exposures for HDR |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (current) |
| **Input (neutral)** | `{neutral}` → `[1, 3, H, W]` FLOAT (reference) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (merged) |
| **ONNX ops** | `Sub → Abs → Mul(weight) → Add` |

#### `TemporalDenoiseBlock`
| | |
|---|---|
| **ID** | `temporal_denoise` |
| **Role** | Temporal noise reduction across frames |
| **Input** | `{prev}/out` → `[1, 3, H, W]` FLOAT (current) |
| **Input (prev)** | `{prev_frame}` → `[1, 3, H, W]` FLOAT (previous) |
| **Output** | `{ns}/out` → `[1, 3, H, W]` FLOAT (denoised) |
| **ONNX ops** | `Sub → Abs → Sub(noise_level) → Max(0) → Add` |

---

## Pipeline Profiles

| Profile | Blocks | Notes |
|---|---|---|
| **LITE** | 16 | Minimal: unpack→norm→CFA→BLC→WB→demosaic+CCM→tone→identity×5→display |
| **MED** | 16 | +EE, +Saturation |
| **HEAVY** | 16 | +FCS, +LDCI, +Bilateral, +Vignetting |
| **PRO** | 16 | +Colorspace |
| **UNIFIED** | 16 | Full pipeline, all blocks |

---

## MNN Fusion Rules (Pass 1)

| Rule | Pattern | Fused Opset |
|---|---|---|
| R1c | Mod+Cast+Div+Stack+Conv | `isp.unpack_packed` |
| R1 | Div+Stack+Conv | `isp.unpack` |
| R6c | Conv+ReLU6 | ReLU6 absorbed into Conv |
| R3c | Sub+ReLU6 | `isp.fcs` |
| R3d | Sub+Mul | Difference scaling |
| R3e | Mul+Add | Channel bias |
| R6d | Mul+ReLU6 | ReLU6 absorbed |
| R3f | Sub+Max+Min | BLC50 dark subtraction |
| R4c | Conv(3×3)+Sub | Unsharp sharpen |
| R2 | Reshape+Conv | Demosaic |
| R4 | Add+Relu | Bias activation |
| R7 | Pow+Clip | Gamma (display) |
| R7b | Conv(1×1,3→4) | `isp.argb_convert` |
| R7c | Conv+Reshape+Mul+Add | `isp.yuv420_convert` |
| R8 | GridSample | Warp |
| R7d | Max+Min+Add+Log+Mul+Exp | `isp.gamma` (disabled) |

## Opset Matcher (Pass 0)

The opset matcher in `mnn_opset_matcher.rs` maps MNN op type sequences to ISP block names for pipeline analysis. Three matching mechanisms:

1. **Exact pattern** (`EXACT_MATCH_TABLE`): Maps fixed op-type sequences to block names. Used for most blocks.
2. **Prefix match** (`match_isp_histogram_prefix`): Handles variable-length patterns (e.g., `CoarseHistogramBlock` with N bins).
3. **Disambiguation**: When multiple blocks share the same op signature (ambiguous pairs), resolve by:
   - `OP_NAME_PREFIXES`: MNN operator name prefix → block name
   - `ATTR_DISAMBIG_RULES`: Per-op attribute values (e.g., `BinaryOp.opType=SUB` vs `MUL`)

### ISP Extra Opset Entries

Complex HEAVY-profile blocks (>3 ops) use the `isp.*` naming convention:

| Block | Pattern | Length | Matcher |
|---|---|---|---|
| `isp.tone_stats` | ConvertTensor→Conv→ConvertTensor→Reduction×3→Const→BinaryOp→ConvertTensor→Reduction→Const→BinaryOp→ConvertTensor→Reduction→Size→Concat | 16 | Exact |
| `isp.calibration` | Reduction→UnaryOp→Reduction→UnaryOp→BinaryOp→Reduction×2→BinaryOp→Const→BinaryOp×2→Reduction×4→Concat→Const→Reshape | 18 | Exact |
| `isp.histogram` | ConvertTensor→Conv→Const→BinaryOp→ConvertTensor→Reduction → [bins] → Const→Concat | 23–95 | Prefix |

---

## Data Flow Example (HEAVY Profile)

```
RawInput(INT32)
    │ [1,1,H,W/4]
    ▼
UnpackCfa(fused)
    │ [1,4,H/2,W/2] float Bayer
    ▼
Identity("aux_hook_src") ──────────────────► CalibrationBlock (stats)
    │ [1,4,H/2,W/2]                              [1,4,Z] zone means
    ▼
BayerWbBlock
    │ [1,4,H/2,W/2] float Bayer
    ▼
DemosaicCcmBlock(fused)
    │ [1,3,H,W] float RGB [0,1]
    ▼
ToneBlock
    │ [1,3,H,W] float RGB
    ▼
FcsBlock → LdciBlock → EeBlock
    │ [1,3,H,W] float RGB
    ▼
BilateralBlock
    │ [1,3,H,W] float RGB
    ▼
VignettingBlock
    │ [1,3,H,W] float RGB
    ▼
SaturationBlock
    │ [1,3,H,W] float RGB
    ▼
DisplayBlock
    │ [1,3,H,W] float RGB (or format-converted)
    ▼
Output
```
