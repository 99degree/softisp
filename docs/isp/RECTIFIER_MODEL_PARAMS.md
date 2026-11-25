# ISP Rectifier Model — Input / Output Params & Gap Analysis vs `IspController`

This document details the distilled neural ISP controller model (the "rectifier"):
its exact input and output parameters, how each maps to pipeline blocks, and a
field-by-field gap analysis against the rule-based `IspController`. It is the
single source of truth for what the neural model provides and what it lacks.

---

## 1. Model Interface at a Glance

| Property | Value |
|----------|-------|
| Crate | `isp-rectifier` (`isp_rectifier::OptimizedInference`) |
| Backend | MNN CPU (default `mnn` feature) or tract-onnx (`onnx-runtime` feature) |
| Input | 267 dims = histogram[256] + metadata[52] |
| Output | 20 dims = wb[3] + ccm[9] + tone[7] + zoom[1] |
| Pre-trained model | `fusedispcontroller_light_int8.onnx` (INT8, 490 KB, ~1.2 ms) |
| Variants | light (FP32/INT8/FP16), medium (FP32/INT8/FP16), full (FP32/INT8/FP16) |
| Safety clamping | WB [0.1, 10], CCM [-2, 2], tone [0, 1], zoom [1, 4] |

The model lives in `isp-rectifier/models/` and is compiled into the APK via
`include_bytes!` (see `NeuralController::EMBEDDED_MODEL_INT8`/`EMBEDDED_MODEL_FP32`).

---

## 2. Input Specification — 267 Dimensions

Input is split into two tensors: `hist` [1, 256] F32 and `meta` [1, 52] F32.
The conversion lives in `isp-rectifier/src/types.rs: FrameMetadata::to_feature_vector()`.

### 2.1 Histogram — `hist` [1, 256]

256-bin luminance histogram, normalized to a fixed sum of 10,000:

```
hist_norm[i] = hist_raw[i] / sum(hist_raw) * 10,000
```

Validation requires `9800 <= sum <= 10200` (see `MODEL_SPECIFICATION.md`).

### 2.2 Metadata — `meta` [1, 52]

Only indices 0–10 are populated; indices 11–51 are reserved zeros for forward
compatibility.

| Index | Field | Source | Normalization | Range | Notes |
|-------|-------|--------|---------------|-------|-------|
| 0 | CCT | Correlated color temperature (K) | `/ 10,000` | [0.2, 1.2] | 2000 K → 0.2, 12000 K → 1.2 |
| 1 | WB gain R | White balance red gain | identity | [0.1, 5.0] | Raw gain value |
| 2 | WB gain G | White balance green gain | identity | [0.8, 1.2] | Reference channel |
| 3 | WB gain B | White balance blue gain | identity | [0.1, 5.0] | Raw gain value |
| 4 | Exposure time | Auto-exposure time (s) | clamp [0, 1] | [0, 1] | Clamped to [0,1] |
| 5 | ISO gain | ISO / analog gain | `log2(iso) / 14` | [0.0, 1.0] | ISO 50 → ~0.0, ISO 12800 → ~1.0 |
| 6 | AF position | Auto-focus actuator position | identity | [0.0, 1.0] | 0 = infinity, 1 = macro |
| 7 | AF sharpness | Focus sharpness metric | identity | [0.0, 1.0] | Higher = sharper |
| 8 | Brightness | Mean luminance | identity | [0.0, 1.0] | Luma mean 0–1 |
| 9 | Contrast | Luminance std dev | identity | [0.0, 1.0] | Luma std dev 0–1 |
| 10 | Noise level | Estimated sensor noise | `/ 0.3` | [0.0, 1.0] | Scaled by 0.3 |
| 11–51 | Reserved | — | zeros | 0.0 | Forward compatibility |

The metadata is computed by `NeuralController::extract_metadata()` from the
frame histogram (see `cam-rust/cam-isp/src/neural_controller.rs:273`).

---

## 3. Output Specification — 20 Dimensions

Outputs are four named tensors: `wb` [1,3], `ccm` [1,9], `tone` [1,7], `zoom` [1,1].

| Output | Index | Shape | Range | Activation | Semantics |
|--------|-------|-------|-------|------------|-----------|
| **WB gains** | 0–2 | [3] | [0.2, 5.0] | `exp(TanhClamp)` | White balance gains [R, G, B] |
| **CCM matrix** | 3–11 | [9] | [-3.0, 3.0] | `TanhClamp` | Color correction matrix (row-major 3×3) |
| **Tone curve** | 12–18 | [7] | [0.0, 1.0] | `Sigmoid` | 7-point tone curve LUT (monotonic) |
| **Zoom factor** | 19 | [1] | [1.0, 4.0] | `ReLU + 1` | Digital zoom / crop factor |

### 3.1 Output → Runtime Param Mapping

| Model Output | Pipeline Block | Runtime Param (extra input) | Status | Notes |
|--------------|---------------|----------------------------|--------|-------|
| `wb[3]` | `BayerWbBlock` | `bayer_wb.gains` [1,4,1,1] | ✅ Applied | `[wb_r, 1.0, wb_b, 1.0]` (RGGB, G=1.0 ref) |
| `ccm[9]` | `DemosaicCcmBlock` | `demosaic_ccm.w` [3,4,1,1] + `b` [3] | ✅ Applied | 3×3 → padded 3×4, bias = 0 |
| `tone_curve_lut[7]` | `ToneBlock` | `tone.contrast/brightness/gamma_recip` | ❌ NOT supported | ToneBlock is parametric only; ignores `curve_lut` |
| `tone_curve_lut[7]` | `ToneLutBlock` | — | ⬜ NOT BUILT | Needed to consume 7-point LUT directly |
| `zoom[1]` | Scale / AdaptiveDownscale | `scale_blocks.scale_factor` | ⚠️ Partially | Stored in `IspParams.zoom`, wiring partial |

See `docs/api/RUNTIME_PARAMS.md` for the full extra_inputs catalog.

---

## 4. Gap Analysis: Neural Model vs `IspController`

### 4.1 What the Neural Model Provides (20 params)

The neural model predicts **four** ISP parameter groups:

| Param Group | Output Dim | Blocks Affected |
|-------------|------------|-----------------|
| White balance gains | 3 | `BayerWbBlock` |
| Color correction matrix | 9 | `DemosaicCcmBlock` |
| Tone curve LUT (7-point) | 7 | `ToneBlock` (dropped — see gap below) |
| Digital zoom factor | 1 | Scale/AdaptiveDownscale |

These are **learned from data** (distilled from CCMNet, Time-Aware AWB, Neural ISP
Tuning teachers) and generalize better across scenes than hand-tuned heuristics.

### 4.2 What `IspController` Provides (everything else)

The rule-based `IspController` (`cam-rust/cam-isp/src/isp_controller.rs`) computes
traditional 3A heuristics from frame statistics. It covers **all** parameter groups
the neural model does **not** predict:

| Param Group | Blocks Affected | Neural Model Covers? |
|-------------|-----------------|----------------------|
| Black level correction (BLC) | `BlcBlock` / `Blc50Block` | ❌ No |
| White balance gains | `BayerWbBlock` | ✅ Yes (overrides) |
| Color correction matrix | `DemosaicCcmBlock` | ✅ Yes (overrides) |
| Tone mapping (contrast/brightness/gamma) | `ToneBlock` | ❌ Partially (only LUT, not parametric) |
| Saturation | `SaturationBlock` | ❌ No |
| Sharpening (edge enhancement) | `EeBlock` | ❌ No |
| Denoise (bilateral/temporal) | `BilateralBlock`, `TemporalDenoiseBlock` | ❌ No |
| Lens corrections (vignetting, CA, LSC, distortion) | `VignettingBlock`, `ChromaticAberrationBlock`, `LensShadingBlock`, `GpuWarpBlock` | ❌ No |
| Digital zoom | Scale/AdaptiveDownscale | ✅ Yes (overrides) |
| False-color suppression | `FcsBlock` | ❌ No |
| Auto-contrast | `AutoContrastBlock` | ❌ No |
| Local dynamic contrast | `LdciBlock` | ❌ No |
| HDR tone | `HdrToneBlock` | ❌ No |
| VCM focus position | GDC focus breathing | ❌ No |

### 4.3 The Hybrid Merge Pattern (Neural + Rule-Based, Complementary)

`NeuralController` does **not** replace `IspController` wholesale. It runs both
complementarily:

1. **Neural model** predicts WB, CCM, tone LUT, zoom.
2. **Rule-based `IspController`** fills BLC, denoise, sharpen, lens, saturation,
   parametric tone contrast/brightness/gamma, and everything else.
3. Neural outputs **override** rule-based defaults per-field in
   `params_from_optimized()` (`cam-rust/cam-isp/src/neural_controller.rs:393`).

```rust
// params.wb / params.ccm / params.tone.curve_lut / params.zoom  ← neural model
// params.blc / params.denoise / params.sharpen / params.saturation / params.tone.contrast
//   / params.tone.brightness / params.tone.gamma  ← rule-based IspController
```

The rule-based defaults are used as the starting point (`self.fallback.params()`),
then neural outputs overwrite only the fields they predict. This ensures no field
is ever zeroed out.

### 4.4 The Tone Curve LUT Gap

`NeuralController::params_from_optimized()` writes the 7-point `tone_curve_lut`
into `ToneParams.curve_lut`, but `ToneBlock` (the ONNX graph block that actually
applies tone mapping) reads only `contrast`, `brightness`, and `gamma` — it
**ignores** `curve_lut`.

Until `ToneLutBlock` is built, you have two options:

1. **Apply what maps, ignore the rest**: WB, CCM, and zoom flow from the neural
   model; the tone curve LUT is dropped and `ToneBlock` uses its default
   parametric curve (contrast=1.0, brightness=0.0, gamma=1.0).

2. **Convert LUT → parametric**: Derive contrast/brightness/gamma from the
   7-point LUT so `ToneBlock` gets something derived from the neural output:
   - `contrast` ≈ (tone[6] - tone[0]) — overall slope
   - `brightness` ≈ tone[3] - 0.5 — midpoint offset
   - `gamma` ≈ log-based fit from curve shape

This is lossy but gives the neural model *some* tone influence.

**What needs to be built:**
- [ ] `ToneLutBlock` — new ISP block that applies a 7-point tone curve LUT per-pixel
- [ ] LUT → parametric contrast/brightness/gamma conversion as fallback
- [ ] Wire `NeuralController` into `cam-ipa` `IpaConfig.neural_enabled`
- [ ] Benchmark end-to-end latency with real model

---

## 5. Register Mapping (Fixed-Point Conversion)

`ISPRegisters::from_params()` (`isp-rectifier/src/types.rs:241`) converts the
floating-point `ISPOptimizedParams` to hardware-register fixed-point values:

| Param | Float Range | Fixed-Point Format | Conversion |
|-------|-------------|-------------------|------------|
| WB gains | [0.1, 10.0] | Q4.12 (`u16`) | `val * 4096.0` |
| CCM elements | [-2.0, 2.0] | Q4.12 signed (`i16`) | `val * 4096.0` |
| Tone LUT | [0.0, 1.0] | Q0.16 (`u16`) | `val * 65535.0` |
| Zoom scale | [1.0, 4.0] | Q4.12 (`u16`) | `val * 4096.0` |

Default register values (identity / neutral):
- WB gains: `0x1000` (1.0)
- CCM: identity matrix, diagonal `0x1000`
- Tone LUT: linear ramp `[0x0000, 0x2666, 0x5999, 0x8000, 0xA666, 0xD999, 0xFFFF]`
- Zoom: `0x1000` (1.0), full-frame crop `crop_w=1920, crop_h=1080`

---

## 6. Safety Clamping

`ISPOptimizedParams::clamp()` enforces safe ranges before register conversion:

| Param | Min | Max | Fallback |
|-------|-----|-----|----------|
| WB gains | 0.1 | 10.0 | 1.0 |
| CCM element | -2.0 | 2.0 | Identity |
| Tone LUT | 0.0 | 1.0 | Linear ramp |
| Zoom | 1.0 | 4.0 | 1.0 |

Clamping is applied automatically by `MnnInference::run()` when `clamp=true`
(the default), and by `register_injector::inject_registers()`.

---

## 7. Code Locations

```
Neural Controller (rectifier, MNN CPU):
├── isp-rectifier/src/types.rs          # FrameMetadata, to_feature_vector() (52-dim meta)
├── isp-rectifier/src/inference.rs      # OptimizedInference trait dispatch
├── isp-rectifier/src/mnn_inference.rs  # MNN CPU inference (ONNX→MNN convert + session)
├── isp-rectifier/src/mnn_sys.rs        # Minimal MNN C FFI declarations
├── isp-rectifier/src/register_injector.rs  # ISPOptimizedParams → ISPRegisters (Q4.12)
├── isp-rectifier/src/rectifier_model.rs     # Mock ONNX generator (hist[256]+meta[52] → 20 outputs)
├── isp-rectifier/models/fusedispcontroller_light*.onnx  # Pre-trained models
├── cam-rust/cam-isp/src/neural_controller.rs   # NeuralController + fallback + hybrid merge
├── cam-rust/cam-isp/src/controller_api.rs      # ControllerApi trait + Controller enum
├── cam-rust/cam-isp/src/isp_controller.rs      # IspController (rule-based 3A)
├── cam-rust/cam-isp/src/isp_params.rs          # IspParams, WbParams, CcmParams, ToneParams, …
├── cam-rust/cam-isp/src/unified_pipeline.rs    # Wires NeuralController as default; feeds isp_params → ProcessParams
└── cam-rust/cam-isp/src/engine.rs              # IspEngine trait, ProcessParams (isp_params field)

ISP Pipeline (MNN/Vulkan GPU):
├── cam-rust/cam-isp/src/mnnengine.rs     # MnnEngine (Vulkan session, set_extra_inputs)
├── cam-rust/cam-isp/src/mnn_converter.rs # convert_onnx_buffer() ONNX→.mnn
└── cam-rust/cam-isp/src/blocks/          # IspBlock implementations (BayerWb, DemosaicCcm, Tone, …)

Docs:
├── docs/isp/NEURAL_CONTROLLER.md       # Neural controller integration walkthrough
├── docs/api/CONTROLLER_API.md          # ControllerApi trait + Controller enum
├── docs/api/RUNTIME_PARAMS.md          # All runtime-feedable extra_inputs tensors
├── isp-rectifier/MODEL_SPECIFICATION.md # Model architecture, training, validation
├── isp-rectifier/ISP_PARAMS.md          # Full 42-param catalog (20 current + 22 planned)
└── isp-rectifier/README.md              # ISP Rectifier overview, models, benchmarks
```

---

## 8. Verification Commands

```bash
# Neural controller unit tests (MNN CPU, isp-rectifier):
cargo test -p cam-isp --features rectifier --lib
cargo test -p cam-isp --test test_rectifier_integration --features rectifier
cargo test -p isp-rectifier --features mnn --lib

# Controller API tests:
cargo test -p cam-isp --lib controller_api

# Full workspace (all crates, mnn feature):
cargo test --workspace --features mnn --no-fail-fast
```
