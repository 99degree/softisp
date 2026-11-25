# Neural Controller — ISP Rectifier Integration

## Overview

The **Neural Controller** (`cam_isp::neural_controller::NeuralController`) is a
distilled neural-network ISP controller that predicts optimal ISP parameters
(white balance gains, color correction matrix, tone curve, digital zoom) from
frame metadata. It wraps `isp_rectifier::OptimizedInference` with automatic
fallback to the rule-based `IspController` when the model is unavailable or
inference fails.

## Two separate inference paths — do not confuse them

| Path | Purpose | Input | Backend | Device |
|------|---------|-------|---------|--------|
| **Neural Controller (rectifier)** | Predict ISP *parameters* (WB, CCM, tone, zoom) | hist[256] + meta[52] | MNN **CPU** | CPU |
| **ISP Pipeline (MNN/Vulkan)** | Process *pixels* (Bayer → RGB → display) | Full-frame tensors | MNN **Vulkan** | GPU |

They are **complementary**, not alternatives:

```
┌─────────────────────────────────────────────────────────────────┐
│                      Per-frame camera loop                      │
│                                                                   │
│  1. Neural Controller (rectifier, MNN CPU)                      │
│     └─ analyze_and_update(frame) → IspParams (WB, CCM, …)      │
│           MNN CPU, small metadata tensors, ~1 ms                │
│                                                                   │
│  2. ISP Pipeline (MNN/Vulkan GPU)                               │
│     └─ engine.process(ProcessParams { isp_params, … })          │
│           MNN Vulkan, full-frame pixels, ~100 ms at 4K          │
│                                                                   │
│  IspParams from step 1 are fed into step 2 via                  │
│  ProcessParams.isp_params, so the neural model's WB/CCM/tone    │
│  values drive the pixel pipeline blocks.                        │
└─────────────────────────────────────────────────────────────────┘
```

### 1. Neural Controller (isp-rectifier, MNN CPU)

**Role**: Replaces traditional rule-based 3A (AE/AWB/CCM/tone heuristics) with a single distilled ONNX model running **on CPU via MNN**.

**Flow per frame**:

```
Raw Bayer frame
     │
     ▼
NeuralController::analyze_and_update(&frame)
     │
     ├─ extract_metadata()                                          │
     │   ├─ compute_histogram()  → 256-bin luminance histogram      │
     │   ├─ brightness/contrast  → from histogram moments           │
     │   ├─ cct                  → estimated from histogram balance │
     │   └─ noise_level          → from adjacent-bin variation      │
     │
     ├─ FrameMetadata::to_feature_vector()                          │
     │     → hist [f32; 256] (sum-normalized to 10,000)             │
     │     → meta [f32; 52]   (CCT, WB, exposure, ISO, AF, …)       │
     │
     ├─ rectifier.optimize(&metadata)                               │
     │   ├─ MNN CPU session (MnnBackendType::Cpu)                   │
     │   ├─ ONNX → MNN conversion (libMNNConvertDeps.so, in-process)│
     │   ├─ inputs: hist [1,256] F32 + meta [1,52] F32              │
     │   ├─ outputs: wb [1,3], ccm [1,9], tone [1,7], zoom [1,1]   │
     │   └─ clamp to RegisterLimits (WB [0.1,10], CCM [-2,2], …)    │
     │
     └─ params_from_optimized() → IspParams                         │
           wb, ccm, tone.curve_lut, zoom, …                         │
                                                              │
                                                              ▼
                                                      IspParams
                                                      (fed to pipeline)
```

**Key point**: The rectifier model is a **pre-trained ONNX file** (`fusedispcontroller_light.onnx`, `_int8`, `_fp16`). It is converted ONNX→MNN in-process via `libMNNConvertDeps.so` and executed on MNN's **CPU** backend — not Vulkan, not tract-onnx. The model file lives in `isp-rectifier/models/` and is also compiled into the APK binary via `include_bytes!`.

**Interface** (verified on the real pre-trained models):

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

**Fallback**: If the model fails to load or inference fails, `NeuralController` silently falls back to `IspController` (rule-based AE/AWB/tone/denoise). The pipeline never stalls.

### 2. ISP Pipeline (cam-isp MNN/Vulkan)

**Role**: Process raw Bayer pixels into a display-ready image, driven by `IspParams` from the controller.

**Flow per frame**:

```
Raw Bayer INT16 sensor data
     │
     ▼
ProcessParams { buf, width, height, isp_params: Some(IspParams), … }
     │
     ▼
IspEngine::process(&params)  →  MnnEngine (Vulkan) or CpuEngine
     │
     ├─ Build ONNX graph from IspBlock chain                       │
     │   (RawInput → UnpackCfa → BayerWb → DemosaicCcm → … → Display)│
     │   Each block emits ONNX nodes; GraphComposer merges them     │
     │
     ├─ convert_onnx_buffer()  →  .mnn bytes                       │
     │     (libMNNConvertDeps.so via FNN, in-memory, no temp files) │
     │
     ├─ MNN session create + set_extra_inputs                      │
     │     (CCM matrix, tone params, Bayer gains, sensor_max, …     │
     │      from isp_params — this is where controller output       │
     │      reaches the pixel pipeline)                             │
     │
     ├─ MNN Vulkan inference (full-frame pixel tensor)             │
     │
     └─ Output: IspFrame { data (RGBA/BGRA/…), width, height, … }
```

**Key point**: The pipeline graph is **built from `IspBlock`s**, compiled to ONNX by `GraphComposer`, then converted ONNX→MNN by `convert_onnx_buffer()`, then executed by `MnnEngine` on **Vulkan**. The `IspParams` (WB, CCM, tone) from the neural controller are injected as extra-input tensors (`set_extra_inputs`) so the pixel blocks use the neural model's values.

## Code locations

```
Neural Controller (rectifier, MNN CPU):
├── isp-rectifier/src/types.rs          # FrameMetadata, to_feature_vector() (52-dim meta)
├── isp-rectifier/src/inference.rs      # OptimizedInference (dispatches to MNN backend)
├── isp-rectifier/src/mnn_inference.rs  # MNN CPU inference (ONNX→MNN convert + session)
├── isp-rectifier/src/mnn_sys.rs        # Minimal MNN C FFI declarations
├── isp-rectifier/src/register_injector.rs  # ISPOptimizedParams → ISPRegisters
├── isp-rectifier/src/rectifier_model.rs     # Mock ONNX generator (hist[256]+meta[52])
├── cam-rust/cam-isp/src/neural_controller.rs   # NeuralController + fallback
├── cam-rust/cam-isp/src/controller_api.rs      # ControllerApi trait + Controller enum
├── cam-rust/cam-isp/src/unified_pipeline.rs    # Wires NeuralController as default
└── isp-rectifier/models/fusedispcontroller_light*.onnx  # Pre-trained models

ISP Pipeline (MNN/Vulkan GPU):
├── cam-rust/cam-isp/src/mnnengine.rs     # MnnEngine (Vulkan session, process())
├── cam-rust/cam-isp/src/mnn_converter.rs # convert_onnx_buffer() ONNX→.mnn
├── cam-rust/cam-isp/src/mnn_sys.rs       # FFI to libmnn_wrapper.so / libMNN.so
├── cam-rust/cam-isp/src/pipeline/types.rs # GraphComposer (IspBlock → ONNX)
├── cam-rust/cam-isp/src/engine.rs        # IspEngine trait, ProcessParams
├── cam-rust/cam-isp/src/blocks/          # IspBlock implementations
└── cam-rust/cam-isp/src/unified_pipeline.rs # feeds isp_params into ProcessParams
```

## How the two connect

The controller's output (`IspParams`) flows into the pipeline via `ProcessParams`:

```rust
// In UnifiedPipeline::process():
let isp_params = self.controller.analyze_and_update(&frame);
// ...
let mut params = ProcessParams::new(width, height, raw_data);
params.isp_params = Some(isp_params.clone());  // ← controller → pipeline
params.ccm_matrix = Some(isp_params.ccm.matrix);
params.awb_gains = Some([isp_params.wb.r, isp_params.wb.g, isp_params.wb.b]);
// ...
let frame = self.engine.process(&params)?;
```

Inside `MnnEngine::build()`, `set_extra_inputs()` writes these values into the ONNX→MNN graph as runtime input tensors, so blocks like `BayerWbBlock` (WB gains) and `DemosaicCcmBlock` (CCM matrix) use the neural model's predictions instead of rule-based defaults.

## Verification

```
# Neural controller (MNN CPU, isp-rectifier):
cargo test -p cam-isp --features rectifier --lib        # 818 tests pass
cargo test -p cam-isp --test test_neural_controller_onnx --features rectifier  # 6 pass
cargo test -p cam-isp --test test_rectifier_integration --features rectifier   # 9 pass
cargo test -p isp-rectifier --features mnn --lib  # 6 pass

# Dual-GPU 4K→FHD benchmark (MNN Vulkan engines A/B):
cargo run --release --example bench_4k_dual_engine -p cam-isp --features mnn
```

## TODO

- [ ] Wire `NeuralController` into `cam-ipa` `IpaConfig.neural_enabled`
- [ ] Add ToneLutBlock support for 7-point LUT output
- [ ] Benchmark end-to-end latency with real model
