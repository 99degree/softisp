# Runtime Parameters (extra_inputs) API

This document lists all runtime-feedable input tensors exposed by ISP blocks
via the `extra_inputs()` trait method. These tensors default to initializer values
but can be overridden per-frame through the MNN engine's `set_extra_inputs()`.

> **Note**: Conditional tensors are only present in the ONNX graph when the
> corresponding block parameter is active. Feeding a non-existent tensor is harmless
> — the engine skips it via `Option::None`.

## `auto_contrast` — Adaptive contrast S-curve — center/stretch/uncenter

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `half` | contrast=1.5 | [1] | FLOAT | 0.5 | 0.5 constant for center/uncenter S-curve. |
| `contrast_w` | contrast=1.5 | [1] | FLOAT | 1.5 | Contrast weight factor. >1 = more contrast. |
| `zero` | contrast=1.5 | [1] | FLOAT | 0.0 | Clip lower bound (0.0). Conditional. |
| `one` | contrast=1.5 | [1] | FLOAT | 1.0 | Clip upper bound (1.0). Conditional. |
| `zero` | contrast=1.5, highlight_compress=0.1 | [1] | FLOAT | 0.0 | Clip lower bound. Conditional: only when highlight_compress > 0.01. |
| `one` | contrast=1.5, highlight_compress=0.1 | [1] | FLOAT | 1.0 | Clip upper bound. Conditional: only when highlight_compress > 0.01. |

## `bayer_wb` — Bayer white balance — per-channel gain

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `gains` | rggb | [1,4,1,1] | FLOAT | [1.0, 1.0, 1.0, 1.0] | RGGB channel gains for Bayer white balance |

## `demosaic_ccm` — Fused demosaic + CCM

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `w` | rggb | [3,4,1,1] | FLOAT | 4×3×1×1 weights, 3×1 bias | CCM convolution weights: shape [3,4,1,1] for 3×4 color correction matrix |
| `b` | rggb | [3] | FLOAT | 4×3×1×1 weights, 3×1 bias | CCM bias: shape [3] for per-channel offset |

## `display` — Display format conversion (FloatRgb)

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `scale` | FloatRgb | [1] | FLOAT | 1.0 | Scale factor. Used in Mul for FloatRgb path. |
| `gamma_exp` | FloatRgb | [1] | FLOAT | 0.4167 | Gamma exponent for sRGB (1/2.4 ≈ 0.4167). Used in Pow. |
| `zero` | FloatRgb | [1] | FLOAT | 0.0 | Clip lower bound (0.0) after gamma. |
| `one` | FloatRgb | [1] | FLOAT | 1.0 | Clip upper bound (1.0) after gamma. |
| `(none)` | Argb | scalar | UNKNOWN | - | ARGB uses fixed Conv weights — no runtime params. |
| `(none)` | PackedRgb | scalar | UNKNOWN | - | PackedRgb uses fixed weights — no runtime params. |

## `fcs` — Film contrast stretch — Mul+Add per-channel

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `gain` | default | [3,1,1] | FLOAT | [1.0, 1.0, 1.0] | Per-channel gain [R,G,B]. 1.0 = identity |
| `bias` | default | [3,1,1] | FLOAT | [0.0, 0.0, 0.0] | Per-channel bias [R,G,B]. 0.0 = identity |

## `gamma` — Gamma correction — pow(x, 1/γ) + clamp + optional shadow lift

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `inv_gamma` | gamma=2.2 | [1] | FLOAT | 0.4545 | Inverse gamma: 1/γ for gamma=2.2 → ~0.4545 |
| `min` | gamma=2.2 | [1] | FLOAT | 0.0 | Clamp minimum (0.0) — Max(input, min) |
| `max` | gamma=2.2 | [1] | FLOAT | 1.0 | Clamp maximum (1.0) — Min(input, max) |
| `lift` | gamma=2.2, shadow_lift=0.05 | [1] | FLOAT | 0.05 | Shadow lift offset (~0.01-0.10). Conditional: only when shadow_lift > 0. |
| `norm` | gamma=2.2, shadow_lift=0.05 | [1] | FLOAT | 1.0 | Shadow lift re-normalize factor. Conditional: only when shadow_lift > 0. |

## `ldci` — Local contrast enhancement (adaptive tone mapping)

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `strength` | default | [1] | FLOAT | 1.0 | Local contrast strength. 1.0 = default |

## `normalize` — INT32→FLOAT + Div by sensor max

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `max_val` | default | [1] | FLOAT | 65535.0 | Sensor max value (65535 for 16-bit) |

## `saturation` — Saturation control — per-channel RGB factor

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `scale` | default | [3] | FLOAT | [1.0, 1.0, 1.0] | Per-channel saturation scale [R,G,B]. 1.0 = identity |

## `sharpen` — Unsharp mask sharpening

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `strength` | strength=0.5 | [1] | FLOAT | 0.5 | Sharpening strength. 0=off, 0.5=moderate, 1.0=strong |

## `tone` — Tone mapping — contrast/brightness/gamma S-curve

| Tensor Suffix | Config | Shape | Type | Default | Description |
|---|---|---|---|---|---|
| `contrast` | default | [1] | FLOAT | 1.0 | Contrast factor (1.0 = identity) |
| `brightness` | default | [1] | FLOAT | 0.0 | Brightness offset (-1..1) |
| `gamma_recip` | default | [1] | FLOAT | 1.0 | Gamma reciprocal (1/γ) |

## Notes

- **Feed safety**: All extra_input tensors are also present as ONNX initializers.
  If the engine does not feed a value, the initializer default is used.
- **Conditional tensors**: Documented per-config above. Only created when the
  block's parameter exceeds its activation threshold.
- **Shape convention**: `scalar` = rank-0 tensor; `[N]` = 1D; `[H,W]` = 2D
- **Data type**: All runtime params are FLOAT (32-bit).
- **Tensor naming**: `{Namespace}/{suffix}`. Namespace = block's `tensor_ns()`.
