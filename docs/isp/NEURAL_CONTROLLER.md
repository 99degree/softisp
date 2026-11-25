# Neural Controller — ISP Rectifier Integration

## Overview

The **Neural Controller** (`cam_isp::neural_controller::NeuralController`) is a
distilled neural-network ISP controller that predicts optimal ISP parameters
(white balance gains, color correction matrix, tone curve, digital zoom) from
frame metadata. It wraps `isp_rectifier::OptimizedInference` (tract ONNX
runtime) with automatic fallback to the rule-based `IspController` when the
model is unavailable or inference fails.

## Architecture

```
Frame → NeuralController::analyze_and_update()
  │
  ├─ extract_metadata()          → FrameMetadata (histogram 256 + metadata 52)
  │      │ compute_histogram()   → 256-bin luminance histogram from raw data
  │      │ brightness/contrast   → computed from histogram moments
  │      │ cct/noise_level       → estimated from histogram distribution
  │
  ├─ rectifier.optimize(meta)    → ISPOptimizedParams (wb, ccm, tone, zoom)
  │      │ ONNX model: fusedispcontroller_light*
  │      │ inputs:  hist [1,256] + meta [1,52]
  │      │ outputs: wb [1,3], ccm [1,9], tone [1,7], zoom [1,1]
  │
  └─ fallback → IspController::analyze_and_update()
         (rule-based AE/AWB/denoise/tone)

Output: IspParams (wb, ccm, tone, saturation, sharpen, denoise, lens, zoom, …)
```

## Files changed

| File | Change |
|------|--------|
| `cam-rust/cam-isp/src/neural_controller.rs` | `extract_metadata()` now computes real frame statistics (brightness, contrast, CCT, noise level) instead of hardcoded constants. |
| `cam-rust/cam-isp/src/rectifier_model.rs` | Mock ONNX model rewritten to match the real model interface: 2 inputs (`hist` [1,256], `meta` [1,52]) and 4 outputs (`wb`, `ccm`, `tone`, `zoom`) via per-head MatMul+Add. |
| `cam-rust/cam-isp/src/unified_pipeline.rs` | `NeuralController::with_mock_model()` is now the default controller when the `rectifier` feature is enabled; `IspController` is used otherwise. |
| `isp-rectifier/src/types.rs` | `FrameMetadata::to_feature_vector()` now emits 52-dim metadata (up from 11) to match pre-trained models. Reserved dims [11..52] are zero. |
| `isp-rectifier/src/inference.rs` | `TractInference::run()` and `run_batch()` use `[1,52]` metadata tensors (was `[1,11]`). |
| `isp-rectifier/src/lib.rs` | Test assertion updated for 52-dim metadata. |
| Tests (`test_neural_controller_onnx.rs`, `test_rectifier_integration.rs`, `bench_neural_controller.rs`) | Added missing `external_data: None` field to `IspFrame` initializers. |

## Model interface

The pre-trained `fusedispcontroller_*` ONNX models (in `isp-rectifier/models/`)
expect:

| Input | Shape | Dtype |
|-------|-------|-------|
| `hist` | [1, 256] | F32 |
| `meta` | [1, 52] | F32 |

| Output | Shape | Semantics |
|--------|-------|-----------|
| `wb` | [1, 3] | WB gains [R, G, B] |
| `ccm` | [1, 9] | CCM matrix (row-major) |
| `tone` | [1, 7] | Tone curve LUT |
| `zoom` | [1, 1] | Digital zoom factor [1.0, 4.0] |

Metadata layout (52 dims):

| Index | Field | Normalization |
|-------|-------|---------------|
| 0 | CCT | `/ 10000.0` |
| 1–3 | WB gains [R, G, B] | identity |
| 4 | exposure_time | clamped to [0, 1] |
| 5 | iso_gain | `log2(iso) / 14`, clamped [0, 1] |
| 6 | AF position | identity [0, 1] |
| 7 | AF sharpness | identity [0, 1] |
| 8 | brightness | identity [0, 1] |
| 9 | contrast | identity [0, 1] |
| 10 | noise_level | `/ 0.3`, clamped [0, 1] |
| 11–51 | reserved | zeros |

## Usage

```rust
use cam_isp::neural_controller::NeuralController;
use cam_isp::controller_api::ControllerApi;

// Default: tries pre-trained ONNX models (light_int8 → light → light_fp16,
// both in isp-rectifier/models/ and repo root), falls back to rule-based.
let mut ctrl = NeuralController::with_default_model();

// With a specific pre-trained model:
let mut ctrl = NeuralController::with_model("isp-rectifier/models/fusedispcontroller_light_int8.onnx");

// Mock model for testing/CI (self-contained, matches real interface):
let mut ctrl = NeuralController::with_mock_model();

let params = ctrl.analyze_and_update(&frame);
// params.wb = [R, G, B]
// params.ccm = [[r, r, r], [g, g, g], [b, b, b]]
// params.tone.curve_lut = [7 tone points]
// params.zoom = zoom factor
```

Or via `UnifiedPipeline`:

```rust
use cam_isp::unified_pipeline::{UnifiedPipeline, UnifiedConfig};

let config = UnifiedConfig::fhd();
let pipeline = UnifiedPipeline::new(config)?
    .with_controller(Box::new(NeuralController::with_default_model()));
```

## Model selection order

`with_default_model()` tries, in order:

1. Filesystem candidates (dev/CI from repo root):
   - `isp-rectifier/models/fusedispcontroller_light_int8.onnx`
   - `isp-rectifier/models/fusedispcontroller_light.onnx`
   - `isp-rectifier/models/fusedispcontroller_light_fp16.onnx`
   - `isp-rectifier/fusedispcontroller_light_int8.onnx`
   - `isp-rectifier/fusedispcontroller_light.onnx`
   - `isp-rectifier/fusedispcontroller_light_fp16.onnx`
2. Embedded bytes compiled into the binary (Android APK):
   - `fusedispcontroller_light_int8.onnx` (INT8)
   - `fusedispcontroller_light.onnx` (FP32)
3. Fallback to rule-based `IspController` if nothing loads.

All variants share the same interface: `hist[256] + meta[52] → wb[3] / ccm[9] / tone[7] / zoom[1]`.

## Fallback behavior

If `rectifier` is disabled, or the model fails to load, or inference fails on a
frame, the controller silently falls back to `IspController` (rule-based
AE/AWB/tone/denoise). This guarantees the pipeline never stalls on a bad model.

## Verification

```
cargo test -p cam-isp --features rectifier --lib        # 818 tests pass
cargo test -p cam-isp --test test_neural_controller_onnx --features rectifier  # 6 pass
cargo test -p cam-isp --test test_rectifier_integration --features rectifier   # 9 pass (mock + real ONNX)
cargo test -p isp-rectifier --features onnx-runtime --lib  # 6 pass
cargo clippy -p cam-isp --lib --features rectifier -- -D warnings  # clean
cargo clippy -p isp-rectifier --lib --features onnx-runtime -- -D warnings  # clean
```

Pre-trained model verification (`isp-rectifier/examples/inspect_models.rs`):
`fusedispcontroller_light.onnx` and `_int8` both load with tract and accept
`[256] + [52]` inputs, producing 4 outputs `[1,3] [1,9] [1,7] [1,1]`.

## TODO

- [ ] Wire `NeuralController` into `cam-ipa` `IpaConfig.neural_enabled`
- [ ] Add MNN CPU rectifier engine as 3rd engine (per user request)
- [ ] Add ToneLutBlock support for 7-point LUT output
- [ ] Benchmark end-to-end latency with real model
