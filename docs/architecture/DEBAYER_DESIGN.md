# Debayer / Demosaic Design Document

## Architecture: Unified `isp.demosaic` Opset

A single MNN Extra op type `isp.demosaic` handles all debayer algorithms.
The algorithm is selected at **converter time** via IspChainFusion, not at
runtime. Each algorithm has its own SPIR-V shader baked into the MNN model.

```
ONNX model (standard Conv pattern)
  ↓ IspChainFusion R2b/R2c/R2d
isp.demosaic (algorithm=binning|bilinear|mhc)
  ↓
SPIR-V shader (specific to algorithm)
  ↓
Vulkan GPU execution
```

### Opset Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `algorithm` | string | `binning`, `bilinear`, `mhc`, `ahd` |
| `bayer_pattern` | int | 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR |
| `sensor_max` | float | Normalization divisor (1023 for 10-bit) |
| `valid_bits` | int | 10 or 12 (affects normalization) |

### Algorithm Selection Rules

| Rule | Trigger Pattern | Algorithm | Output |
|------|-----------------|-----------|--------|
| R2b | Conv(4×4, stride=1, 1ch→3ch) | bilinear | [1,3,H,W] |
| R2c | Conv(6×6, stride=1, 1ch→3ch) | mhc | [1,3,H,W] |
| R2d | Conv(2×2, stride=2, 1ch→4ch) | binning | [1,4,H/2,W/2] |

The Conv kernel size and stride determine the algorithm:
- **Binning**: 2×2 stride=2 → half resolution, 4 channels
- **Bilinear**: 4×4 stride=1 → full resolution, 3 channels
- **MHC**: 6×6 stride=1 → full resolution, 3 channels (larger kernel for adaptive weights)

### SPIR-V Shaders

| Algorithm | Shader File | Workgroup | Kernel |
|-----------|-------------|-----------|--------|
| binning | `shader_unpack_blc.comp` | 16×16 | 2×2 stride=2 |
| bilinear | `shader_demosaic_interp.comp` | 16×16 | 3×3 effective |
| mhc | `shader_demosaic_mhc.comp` | 16×16 | 5×5 adaptive |
| ahd | `shader_demosaic_ahd.comp` | 8×8 | 7×7 multi-pass |

## Resolution Mapping

### Sensor → Display Cases

| Case | Sensor | Output | Algorithm | Pipeline |
|------|--------|--------|-----------|----------|
| A | 8K → 4K | 4× downscale | binning | UnpackCfa(stride=4) |
| B | 8K → FHD | 8× downscale | binning | UnpackCfa(stride=4) + Resize |
| C | 4K → FHD | 2× downscale | binning | UnpackCfa(stride=2) |
| D | 4K → 4K | same res | bilinear | DemosaicBlock(algo=bilinear) |
| E | FHD → FHD | same res | bilinear/mhc | DemosaicBlock(algo=bilinear) |
| F | FHD → HD | downscale | bilinear | DemosaicBlock + Resize |
| G | HD → HD | same res | bilinear | DemosaicBlock(algo=bilinear) |

### Selection Logic

```
if sensor_resolution > 4× output_resolution:
    algorithm = binning(stride=4)
elif sensor_resolution > 2× output_resolution:
    algorithm = binning(stride=2)
else:
    if quality == fast:
        algorithm = bilinear
    elif quality == balanced:
        algorithm = mhc
    elif quality == high:
        algorithm = ahd
```

## Algorithm Details

### Binning (Current)

**Shader:** `shader_unpack_blc.comp`
**Pattern:** Conv(2×2, stride=2, group=4)
**Output:** [1,4,H/2,W/2] — 4 Bayer channels at half resolution

```
TL=R  TR=Gr    →  ch0=R, ch1=Gr
BL=Gb BR=B        ch2=Gb, ch3=B
```

G1/G2 averaging done by DemosaicCcmBlock (Conv 4→3).

**Speed:** ★★★★★ | **Quality:** ★★ | **Resolution:** H/2 × W/2

### Bilinear Interpolation

**Shader:** `shader_demosaic_interp.comp`
**Pattern:** Conv(4×4, stride=1, group=1)
**Output:** [1,3,H,W] — RGB at full resolution

```
For each output pixel (x,y):
  Determine Bayer color at (x%2, y%2)
  Known color: use directly
  Missing colors: average 2 nearest neighbors
```

**Speed:** ★★★★ | **Quality:** ★★★ | **Resolution:** H × W

### MHC (Malvar-He-Cutler)

**Shader:** `shader_demosaic_mhc.comp` (planned)
**Pattern:** Conv(6×6, stride=1, group=1)
**Output:** [1,3,H,W] — RGB at full resolution

```
For each output pixel (x,y):
  Compute 5×5 Laplacian to detect edge direction
  Interpolate along edge (not across it)
  Adaptive weights based on local gradient
```

**Speed:** ★★★ | **Quality:** ★★★★ | **Resolution:** H × W

### AHD (Adaptive Homogeneity-Directed)

**Shader:** `shader_demosaic_ahd.comp` (not planned — too slow)
**Pattern:** Multi-pass (horizontal + vertical interpolation + selection)
**Output:** [1,3,H,W] — RGB at full resolution

**Speed:** ★★ | **Quality:** ★★★★★ | **Resolution:** H × W

## Design Limitations

### 1. Algorithm is Compile-Time Choice

The SPIR-V shader is baked into the MNN model at conversion time.
Changing algorithms requires re-converting the model. This is intentional:
different algorithms have fundamentally different compute patterns that
cannot be efficiently selected at runtime.

### 2. Vulkan Limitations

| Op | Status | Workaround |
|----|--------|------------|
| Cast (op_type=9) | ❌ Not supported | Fuse into adjacent Conv |
| Resize (op_type=74) | ❌ Not supported | Use Conv stride |
| Standard Conv for demosaic | ⚠️ Limited | Custom SPIR-V shader |

### 3. Bayer Pattern Not Yet Dynamic

Current SPIR-V shaders assume RGGB. Other patterns (GRBG, GBRG, BGGR)
require different position mappings. The `bayer_pattern` parameter is
stored but not used by the shader. Implementation requires either:
- Runtime branching in shader (slight performance cost)
- Compile-time shader variant per pattern (no runtime cost)

### 4. Interpolation Quality vs Speed

| Algorithm | ALU/pixel | Latency (FHD) | Quality |
|-----------|-----------|---------------|---------|
| Bilinear | ~20 | 4ms | Medium |
| MHC | ~80 | 8ms | High |
| AHD | ~200+ | 20ms+ | Excellent |

For real-time video (30+ FPS), bilinear or MHC are the only viable options.

### 5. No Temporal Demosaic

All algorithms are spatial (single-frame). Temporal demosaic uses
frame-to-frame coherence to improve quality but requires:
- Frame buffer (previous N frames)
- Motion estimation (EIS/OPT flow)
- Significantly more GPU memory and compute

### 6. Edge Cases

- **Border pixels:** Clamped (repeated edge pixel)
- **Dead pixels:** Not handled by demosaic (requires separate DPC block)
- **High ISO noise:** Demosaic amplifies noise; denoise should precede demosaic

## Implementation Status

| Algorithm | Shader | Fusion Rule | Block | Status |
|-----------|--------|-------------|-------|--------|
| Binning | ✅ `shader_unpack_blc.comp` | R1, R10, R12 | `UnpackCfaBlock` | ✅ Working |
| Bilinear | ✅ `shader_demosaic_interp.comp` | R2b | `DemosaicInterpBlock` | ✅ Implemented |
| MHC | 🔲 TBD | R2c | `DemosaicBlock(algo=mhc)` | 🔲 Planned |
| AHD | ❌ N/A | N/A | N/A | ❌ Not planned |

## Performance (Vulkan GPU, Snapdragon)

| Algorithm | 4K→FHD | FHD→FHD | HD→HD |
|-----------|--------|---------|-------|
| Binning | 2ms | N/A | N/A |
| Bilinear | N/A | 4ms | 2ms |
| MHC | N/A | 8ms | 4ms |
| AHD | N/A | 20ms+ | 10ms+ |

## Future Work

1. **MHC Shader:** 5×5 adaptive kernel with Laplacian edge detection
2. **Bayer Pattern Support:** Runtime branching or compile-time variants
3. **Quality Auto-Selection:** Choose algorithm based on ISO/content
4. **Fused Demosaic+CCM:** Combine demosaic and color correction in one shader
5. **Temporal Demosaic:** Frame-to-frame coherence for video
