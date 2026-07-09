# ISP Parameters — Full Reference for Neural ISP Tuning

**Purpose**: Complete catalog of all parameters that the Neural ISP Tuning model can predict, based on the `PIPELINE_BLOCKS.md` specification.

---

## 📋 Summary

| Category | Parameters | Output Dim | Priority |
|----------|-----------|------------|----------|
| **White Balance** | WB gains [R,Gb,Gr,B] | 4 | ✅ Critical |
| **Color Correction** | CCM 3×3 matrix | 9 | ✅ Critical |
| **Tone Mapping** | Tone curve LUT | 7 | ✅ Critical |
| **Zoom** | Scale factor | 1 | ✅ Critical |
| **Noise Reduction** | Bilateral filter params | 3 | ⚠️ High |
| **Sharpening** | Edge enhancement | 1 | ⚠️ High |
| **Saturation** | Saturation + Vibrance | 2 | ⚠️ High |
| **Lens Corrections** | Vignetting, CA, LSC | 3+ | ⚠️ Medium |
| **Geometric** | Distortion coefficients | 3 | ⚠️ Medium |
| **Advanced** | HDR tone, Local contrast | 4+ | 💡 Optional |
| **Total** | — | **~42** | — |

---

## 🎯 Current Implementation (v1.2)

The current distilled model outputs **20 parameters**:

| Output | Index | Shape | Description |
|--------|-------|-------|-------------|
| **WB Gains** | 0-2 | [3] | [R, G, B] white balance |
| **CCM** | 3-11 | [9] | 3×3 color correction matrix |
| **Tone Curve** | 12-18 | [7] | 7-point LUT |
| **Zoom** | 19 | [1] | Digital zoom factor |

**Gap**: 22 additional parameters from PIPELINE_BLOCKS.md are not yet modeled.

---

## 🔧 Full Parameter Catalog

### 1. White Balance (`BayerWbBlock`)

```rust
pub struct WhiteBalanceParams {
    pub gains: [f32; 4],  // [R, Gr, Gb, B]
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `gain_r` | [0.1, 5.0] | 1.0 | Red channel gain |
| `gain_gr` | [0.8, 1.2] | 1.0 | Green (red row) gain |
| `gain_gb` | [0.8, 1.2] | 1.0 | Green (blue row) gain |
| `gain_b` | [0.1, 5.0] | 1.0 | Blue channel gain |

**Relationship to CCT**:
```
CCT < 3500K → warm (gain_r > gain_b)
CCT > 6500K → cool (gain_b > gain_r)
CCT ≈ 5500K → neutral (all ≈ 1.0)
```

---

### 2. Color Correction (`CcmBlock`)

```rust
pub struct CCMParams {
    pub matrix: [[f32; 3]; 3],  // Row-major 3×3
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `ccm[0][0]` | [-3.0, 3.0] | 1.0 | R→R weight |
| `ccm[0][1]` | [-3.0, 3.0] | 0.0 | G→R weight |
| `ccm[0][2]` | [-3.0, 3.0] | 0.0 | B→R weight |
| `ccm[1][0]` | [-3.0, 3.0] | 0.0 | R→G weight |
| `ccm[1][1]` | [-3.0, 3.0] | 1.0 | G→G weight |
| `ccm[1][2]` | [-3.0, 3.0] | 0.0 | B→G weight |
| `ccm[2][0]` | [-3.0, 3.0] | 0.0 | R→B weight |
| `ccm[2][1]` | [-3.0, 3.0] | 0.0 | G→B weight |
| `ccm[2][2]` | [-3.0, 3.0] | 1.0 | B→B weight |

**Constraints**:
- Diagonal ≈ 1.0 for neutral
- Row sums ≈ 1.0 (energy preservation)
- Determinant > 0 (no color inversion)

---

### 3. Tone Mapping (`ToneBlock` / `ToneLutBlock`)

```rust
pub struct ToneParams {
    pub lut: [f32; 7],  // Piecewise linear LUT
}
```

| Index | X Position | Description |
|-------|------------|-------------|
| 0 | 0.0 | Black point (must be 0.0) |
| 1 | 1/6 | Shadow detail |
| 2 | 2/6 | Dark midtones |
| 3 | 3/6 | Midtones |
| 4 | 4/6 | Light midtones |
| 5 | 5/6 | Highlight detail |
| 6 | 1.0 | White point (must be 1.0) |

**Constraints**:
- Monotonic: `lut[i] ≤ lut[i+1]`
- Range: [0.0, 1.0]
- Linear default: `lut[i] = i/6`

---

### 4. Zoom (`AdaptiveDownscaleBlock`)

```rust
pub struct ZoomParams {
    pub factor: f32,  // Scale factor
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `factor` | [1.0, 4.0] | 1.0 | Digital zoom multiplier |

**Relationship**:
- `factor = 1.0` → No zoom (full FOV)
- `factor = 2.0` → 2× zoom (center crop)
- `factor = 4.0` → 4× zoom (heavy crop)

---

### 5. Noise Reduction (`BilateralBlock`)

```rust
pub struct DenoiseParams {
    pub sigma_spatial: f32,   // Spatial sigma
    pub sigma_range: f32,     // Range sigma
    pub kernel_size: u32,     // Filter kernel size
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `sigma_spatial` | [1.0, 20.0] | 5.0 | Spatial smoothing strength |
| `sigma_range` | [0.01, 0.5] | 0.1 | Edge preservation |
| `kernel_size` | [3, 15] | 5 | Filter window size |

**Trade-off**:
- High sigma_spatial → more smoothing, less detail
- High sigma_range → better edge preservation
- Large kernel → slower, stronger effect

---

### 6. Sharpening (`EeBlock`)

```rust
pub struct SharpenParams {
    pub strength: f32,  // Edge enhancement strength
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `strength` | [0.0, 2.0] | 0.5 | Unsharp mask strength |

**Note**:过高 strength creates halos around edges.

---

### 7. Saturation (`SaturationBlock`)

```rust
pub struct SaturationParams {
    pub saturation: f32,  // Global saturation
    pub vibrance: f32,    // Selective saturation boost
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `saturation` | [0.0, 3.0] | 1.0 | Global color saturation |
| `vibrance` | [0.0, 1.0] | 0.0 | Boost muted colors only |

**Effect**:
- `saturation = 0.0` → Grayscale
- `saturation = 1.0` → Neutral
- `saturation > 1.0` → Vivid colors
- `vibrance > 0` → Boost desaturated colors without oversaturating skin tones

---

### 8. Lens Corrections

#### Vignetting (`VignettingBlock`)
```rust
pub struct VignettingParams {
    pub strength: f32,   // Correction strength [0,1]
    pub center_x: f32,   // Center X offset [-0.5, 0.5]
    pub center_y: f32,   // Center Y offset [-0.5, 0.5]
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `strength` | [0.0, 1.0] | 0.0 | Correction amount |
| `center_x` | [-0.5, 0.5] | 0.0 | Lens center X offset |
| `center_y` | [-0.5, 0.5] | 0.0 | Lens center Y offset |

#### Chromatic Aberration (`ChromaticAberrationBlock`)
```rust
pub struct CAParams {
    pub k1: f32,  // Radial distortion coefficient 1
    pub k2: f32,  // Radial distortion coefficient 2
    pub k3: f32,  // Radial distortion coefficient 3
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `k1` | [-0.5, 0.5] | 0.0 | First-order CA |
| `k2` | [-0.5, 0.5] | 0.0 | Second-order CA |
| `k3` | [-0.5, 0.5] | 0.0 | Third-order CA |

#### Lens Shading (`LensShadingBlock`)
```rust
pub struct LSCParams {
    pub table: [[f32; 4]; 4],  // 4×4 shading gain table
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `table[i][j]` | [0.5, 2.0] | 1.0 | Per-zone shading gain |

---

### 9. Geometric Transforms

#### Distortion (`GpuWarpBlock`)
```rust
pub struct DistortionParams {
    pub k1: f32,  // Barrel/pincushion coefficient
    pub k2: f32,  // Higher-order distortion
    pub k3: f32,  // Cubic distortion
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `k1` | [-0.5, 0.5] | 0.0 | Barrel (negative) / Pincushion (positive) |
| `k2` | [-0.3, 0.3] | 0.0 | Higher-order term |
| `k3` | [-0.1, 0.1] | 0.0 | Cubic term |

---

### 10. Advanced Parameters

#### HDR Tone Mapping (`HdrToneBlock`)
```rust
pub struct HDRToneParams {
    pub strength: f32,     // HDR blend strength [0,1]
    pub local_adapt: f32,  // Local adaptation radius [0,1]
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `strength` | [0.0, 1.0] | 0.0 | HDR effect strength |
| `local_adapt` | [0.0, 1.0] | 0.5 | Local contrast adaptation |

#### Local Contrast (`LdciBlock`)
```rust
pub struct ContrastParams {
    pub strength: f32,  // Local contrast enhancement [0,1]
}
```

| Param | Range | Default | Description |
|-------|-------|---------|-------------|
| `strength` | [0.0, 1.0] | 0.0 | Micro-contrast boost |

---

## 📊 Complete Output Vector

### Minimal (v1.2): 20 params
```python
output = {
    "wb": [3],      # WB gains
    "ccm": [9],     # CCM matrix
    "tone": [7],    # Tone curve
    "zoom": [1],    # Zoom factor
}
# Total: 20
```

### Full: 42 params
```python
output = {
    # Core (20)
    "wb": [4],           # WB gains [R, Gr, Gb, B]
    "ccm": [9],          # CCM 3×3
    "tone": [7],         # Tone LUT
    "zoom": [1],         # Zoom factor
    
    # Noise (3)
    "denoise_spatial": [1],
    "denoise_range": [1],
    "denoise_kernel": [1],
    
    # Detail (1)
    "sharpen": [1],
    
    # Color (2)
    "saturation": [1],
    "vibrance": [1],
    
    # Lens (4)
    "vignetting": [1],
    "ca_k1": [1],
    "ca_k2": [1],
    "lsc_strength": [1],
    
    # Geometric (3)
    "distortion_k1": [1],
    "distortion_k2": [1],
    "distortion_k3": [1],
    
    # Advanced (4)
    "hdr_strength": [1],
    "hdr_local": [1],
    "local_contrast": [1],
    "brightness": [1],
}
# Total: 42
```

---

## 🎚️ Parameter Groups for Distillation

| Group | Params | Teacher Model | Weight |
|-------|--------|---------------|--------|
| **White Balance** | WB[4] | Time-Aware AWB | 0.30 |
| **Color** | CCM[9] | CCMNet | 0.30 |
| **Tone** | Tone[7] | Adobe curves / X-Rite | 0.20 |
| **Detail** | Sharpen + Denoise[4] | Neural ISP Tuning | 0.10 |
| **Geometry** | Zoom + Distortion[4] | Synthetic | 0.05 |
| **Lens** | Vignetting + CA + LSC[4] | Calibrated | 0.05 |

---

## 🔌 Integration with PIPELINE_BLOCKS.md

| Our Param | PIPELINE Block | Injection Point |
|-----------|----------------|-----------------|
| `wb[0:3]` | `BayerWbBlock` | After CFA, before Demosaic |
| `ccm[0:9]` | `CcmBlock` | After Demosaic, before Tone |
| `tone[0:7]` | `ToneBlock` | After CCM, before Saturation |
| `zoom` | `AdaptiveDownscaleBlock` | After Tone, before Display |
| `denoise[0:3]` | `BilateralBlock` | After Demosaic, before CCM |
| `sharpen` | `EeBlock` | After Tone, before Display |
| `saturation[0:1]` | `SaturationBlock` | After Tone, before Display |
| `vignetting` | `VignettingBlock` | After Demosaic, before Tone |
| `ca[0:2]` | `ChromaticAberrationBlock` | After Demosaic, before Tone |
| `lsc` | `LensShadingBlock` | After CFA, before Demosaic |
| `distortion[0:2]` | `GpuWarpBlock` | Before Display |
| `hdr[0:1]` | `HdrToneBlock` | After Tone, before Display |
| `local_contrast` | `LdciBlock` | After Tone, before Display |

---

## 📈 Output Shape Evolution

| Version | Output Dim | Parameters |
|---------|------------|------------|
| **v1.2** (current) | 20 | WB + CCM + Tone + Zoom |
| **v1.3** (planned) | 30 | + Denoise + Sharpen + Saturation |
| **v1.4** (planned) | 42 | + Lens + Geometric + Advanced |

---

## ⚠️ Safety Constraints

| Param | Min | Max | Fallback |
|-------|-----|-----|----------|
| WB gains | 0.1 | 5.0 | 1.0 |
| CCM element | -3.0 | 3.0 | Identity |
| Tone LUT | 0.0 | 1.0 | Linear |
| Zoom | 0.5 | 4.0 | 1.0 |
| Denoise sigma | 0.1 | 20.0 | 5.0 |
| Sharpen | 0.0 | 2.0 | 0.5 |
| Saturation | 0.0 | 3.0 | 1.0 |
| Distortion | -0.5 | 0.5 | 0.0 |

---

© 2026 ISP Rectifier Team
Based on `PIPELINE_BLOCKS.md` from cam-rust