# Pipeline Profiles: REFERENCE and INFINITE

## Overview

Two new pipeline profiles were added to the ISP block chain system, extending the
existing 5 profiles (LITE, MED, HEAVY, PRO, TEST) to 7 with **REFERENCE** and
**INFINITE**. These profiles use **proper block implementations** — `EeBlock`,
`FcsBlock`, `LdciBlock` — instead of the `CcmBlock` placeholders used by HEAVY/PRO
for unsharp mask, false color suppression, and local contrast enhancement.

## Profile Comparison

| Profile     | Level | Blocks | DPC | Demosaic | LSC | FCS | LDCI | Unsharp | Warp | HDR |
|-------------|-------|-------:|:---:|:---------|:---:|:---:|:----:|:-------:|:----:|:---:|
| LITE        | 0     | 9      |     | HqLinear |     |     |      |         |      |     |
| MED         | 1     | 11     | ✅  | Standard |     |     |      | 🟡      |      |     |
| HEAVY       | 2     | 13     | ✅  | Edge     | ✅  |     | 🟡   | 🟡      |      |     |
| PRO         | 3     | 15     | ✅  | Edge     | ✅  |     | 🟡   | 🟡      | 🟡   | 🟡  |
| **REFERENCE** | 4   | **14** | ✅ | Edge     | ✅  | ✅  | ✅   | ✅       |      |     |
| **INFINITE**  | 5   | **15+** | ✅ | Edge   | ✅  | ✅  | ✅   | ✅       | 🟡   | 🟡  |
| TEST        | —     | 2      |     | Standard |     |     |      |         |      |     |

**Legend:** ✅ = real implementation, 🟡 = placeholder (`CcmBlock`), blank = disabled.

### Block Pipeline Order

```
RawInput(INT16) → Normalize → [DPC] → CFA → [LSC(4ch)] → WB → Demosaic
→ [Warp] → CCM → Tone → [FCS] → [LDCI] → [Unsharp] → Display(UINT8 BGRA)
```

- **REFERENCE**: FCS (real), LDCI (real), Unsharp (real) — 14 blocks
- **INFINITE**: same as REFERENCE + Warp + HDR placeholders — 16 blocks
- **PRO**: same blocks but FCS is absent (proposed as placeholder for PRO but PRO
  keeps its existing block set for backward compatibility)

## Technical Details

### Block Implementations

#### EeBlock (`blocks/ee.rs`) — Edge Enhancement (Unsharp Mask)

```
Input → Lapalacian 3×3 edge detect → Y-channel separation → gain_scaled(ee_gain/256)
→ boost_raw = edge_y × gain_scaled → Clip[0,1] → Output = Input + boost → Clip[0,1]
```

- **Extra input**: `ee_gain` (FLOAT scalar) — runtime edge enhancement strength
- **Weights**: 3×15=45 Laplace filter coefficients (replicated across R,G,B channels)
- **Initializers**: `ee_gain`→`gain_scaled` multiplier (1/256), zero/one clip bounds
- **Replaces**: `CcmBlock::with_instance("unsharp")` placeholder

#### FcsBlock (`blocks/fcs.rs`) — False Color Suppression

```
Input → Laplacian → Abs → gain_scaled(fcs_gain/64) → fcs_raw = edge × gain_scaled
→ Clip[0,1] → fcs_attn = 1 − fcs_raw → uv_gain = fcs_attn × uv_mask
→ Y = broadcast(fcs_attn) → Output = Input × Y + UV × uv_gain
```

- **Extra input**: `fcs_gain` (FLOAT scalar) — runtime FCS strength
- **Design**: Edge-aware chroma desaturation — suppresses color at high-frequency
  edges to prevent false color artifacts from demosaic
- **UV mask**: `[1,0,1]` applied to U/V channels only

#### LdciBlock (`blocks/ldci.rs`) — Local Dynamic Contrast Enhancement

```
Input → Box blur 3×3 → diff = Input − blur → y_mask(3,1,1) = [1,0,0]
→ boost = diff_y × strength_scaled(ldci_strength/16) → Output = Input + boost → Clip[0,1]
```

- **Extra input**: `ldci_strength` (FLOAT scalar) — runtime local contrast amount
- **Design**: Local contrast via low-pass subtraction (similar to unsharp masking
  but operating on luminance channel only)

### Extra Inputs and ORT Validation

A critical fix was required for ORT validation to pass: **extra inputs** (runtime
parameters like `ee_gain`, `fcs_gain`, `ldci_strength`, `warp_grid`) were being
added to `value_info` (field 13) but **not** to `graph_inputs` (field 11). ONNX
Runtime requires all node inputs that aren't produced by a previous node to be
declared as graph inputs.

The fix in `pipeline.rs`:
```rust
// Before (validation failure):
value_infos.push(Proto::value_info(&name, &shape_dims, elem_type as i32));

// After (ORT-compatible):
let vi = Proto::value_info(&name, &shape_dims, elem_type as i32);
value_infos.push(vi.clone());
graph_inputs.push(vi);  // ← Also add as graph input
```

Without this fix, ORT rejects the model with:
```
Node input 'FcsBlock/fcs_gain' is not a graph input, initializer,
or output of a previous node.
```

### WarpBlock GridSampler Limitation

`WarpBlock` uses the **`GridSampler`** op, which is **not a standard ONNX
operator**. It's an ONNX contrib op (`ai.onnx.contrib`) or a custom op that
requires specific registration in the runtime. ORT 1.26.0 reports:

```
No Op registered for GridSampler with domain_version of 16
```

Short-term fix: INFINITE uses a `CcmBlock::with_instance("warp")` placeholder
for now. The real warp can be applied in the CPU backend. Long-term fix would
be to register a custom `GridSampler` op in ONNX Runtime, or use an alternative
composition of standard ONNX ops.

### PipelineLevel Enum

```rust
pub enum PipelineLevel {
    Lite = 0,
    Medium = 1,
    Heavy = 2,
    Pro = 3,
    Reference = 4,  // NEW
    Infinite = 5,   // NEW
}
```

Used for conditional dispatching in `build_blocks`:
- `level >= Reference`: real blocks for FCS, LDCI, Unsharp
- `level < Reference`: CcmBlock placeholders

### PipelineProfile Flags

```rust
pub struct PipelineProfile {
    pub label: &'static str,
    pub level: PipelineLevel,
    pub use_bad_pixel: bool,
    pub input_elem_type: i32,      // 1=FLOAT, 4=UINT16, 5=INT16
    pub demosaic_quality: DemosaicQuality,
    pub use_local_contrast: bool,
    pub use_unsharp: bool,
    pub use_lsc: bool,
    pub use_warp: bool,
    pub use_hdr: bool,
    pub use_fcs: bool,              // NEW
}
```

### Block Counts

| Profile   | Base | DPC | LSC | FCS | LDCI | Unsharp | Warp | HDR | **Total** |
|-----------|:----:|:---:|:---:|:---:|:----:|:-------:|:----:|:---:|:---------:|
| LITE      | 9    |     |     |     |      |         |      |     | **9**     |
| MED       | 9    | +1  |     |     |      | +1      |      |     | **11**    |
| HEAVY     | 9    | +1  | +1  |     | +1   | +1      |      |     | **13**    |
| PRO       | 9    | +1  | +1  |     | +1   | +1      | +1   | +1  | **15**    |
| REFERENCE | 9    | +1  | +1  | +1  | +1   | +1      |      |     | **14**    |
| INFINITE  | 9    | +1  | +1  | +1  | +1   | +1      | +1   | +1  | **16**    |

Base blocks: RawInput, Normalize, CFA, BLC, WB, Demosaic, CCM, Tone, Display (9).

## Test Results

All 148 tests pass:

```
Unit tests:           133 passed, 0 failed, 2 ignored
Profile integration:    8 passed, 0 failed
ORT validation:         7 passed, 0 failed   (LITE, MED, HEAVY, PRO, TEST, REFERENCE, INFINITE)
```

ORT validation runs against `libonnxruntime.so 1.26.0` via the `ort` crate's
`SessionBuilder::with_model_from_memory()` which performs full structural
validation (ir_version, opset compatibility, graph connectivity, type inference,
shape inference, op registry).

## Blocks Reference (REFERENCE+)

| Block     | ONNX Nodes | Inputs                     | Extra Inputs       | Description                          |
|-----------|:----------:|:---------------------------|:-------------------|--------------------------------------|
| EeBlock   | 7          | 3ch image                  | `ee_gain` (float)  | Laplace edge detection + boost       |
| FcsBlock  | 9          | 3ch image                  | `fcs_gain` (float) | Edge-aware chroma desaturation       |
| LdciBlock | 7          | 3ch image                  | `ldci_strength` (float) | Box blur + luminance boost      |
| WarpBlock*| 2          | image + grid               | `warp_grid`        | GridSampler (non-standard ONNX op)   |

*WarpBlock limited to CcmBlock placeholder in ONNX path due to GridSampler not
being a standard ONNX op.

## Build & Test Commands

```bash
# Unit + profile tests
cd cam-rust
cargo test --lib -p cam-isp
cargo test --test test_profile_onnx -p cam-isp

# ORT validation (requires libonnxruntime.so)
ORT_LIB_PATH=$PWD/lib/arm64-v8a \
ORT_PREFER_DYNAMIC_LINK=1 \
LD_LIBRARY_PATH=$PWD/lib/arm64-v8a \
cargo test --test test_ort_validation -p cam-isp --features ort

# Verify default profile example
cargo run --example pipeline -p cam-isp
```
