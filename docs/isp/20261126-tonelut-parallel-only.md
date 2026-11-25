# ToneLutBlock — move scalar (non-parallel) derivation to CPU (2026-11-26)

## Symptom / design concern

`ToneLutBlock` (`cam-rust/cam-isp/src/blocks/tone_lut.rs`) emits ONNX graph
nodes that are **not per-pixel**. For each of the 6 LUT segments it computes:

- `diff_i = y_{i+1} - y_i`  (`Sub`, two `[1]` tensors)
- `slope_i = diff_i / seg_width` (`Div`, two `[1]` tensors)

Those 12 nodes operate only on the 7 scalar control points. They are broadcast
through the frame afterwards, but the derivation itself has zero pixel-level
parallelism — it is the same class of work as a running average or a sum over a
handful of items. Running it as graph nodes wastes GPU dispatch slots and
violates the rule that an ISP block graph should contain only parallel,
per-pixel algorithms.

## Where the values come from

The 7 control points arrive per-frame from the neural controller:

- `NeuralController::params_from_optimized()` writes
  `isp_params.tone.curve_lut = optimized.tone_curve_lut`.
- `MnnEngine::set_extra_inputs()` (`mnnengine.rs`) writes them into
  `ToneLutBlock/y0..y6` immediately before inference.

`set_extra_inputs()` is the CPU-side callback that prepares tensor inputs for
inference — the correct home for small scalar derivations.

## Fix

1. `ToneLutBlock::nodes()` keeps only per-pixel element-wise ops:
   `Sub(x, t_i)` → `Clip` → `Mul(by s_i)` → `Add` accumulate → contrast →
   brightness → clip → gamma. The 6 `diff` + 6 `slope` nodes are removed.
2. `y1..y6` graph inputs are replaced by `s0..s5` (precomputed slopes). `y0`
   stays as the accumulator seed. `t_i`, `seg_width`, `zero`, `one`,
   `contrast`, `brightness`, `inv_gamma` are unchanged constants/defaults.
3. A pure CPU helper `tone_curve_slopes(&[f32; 7]) -> [f32; 6]` is exposed from
   `tone_lut.rs`. `extra_input_defaults()` uses it for the built-in LUT, and
   `mnnengine.rs::set_extra_inputs()` uses it on the per-frame neural curve
   before writing `ToneLutBlock/s0..s5`.
4. Node count 42 → 30; extra-input count stays 19 (7 `y` → 1 `y0` + 6 `s`).

## Invariants preserved

At `x = k/6` the piecewise sum still equals `y_k`, because
`sum_{j<k} slope_j * seg_width = sum_{j<k} (y_{j+1} - y_j) = y_k - y_0` and the
accumulator starts at `y_0`. Identity LUT slopes are all `1.0`.

## Verification

- `cargo fmt --all -- --check`
- `cargo clippy -p cam-isp --lib --features rectifier -- -D warnings`
- `cargo test -p cam-isp --lib --features rectifier --no-fail-fast`
