# Neural Controller vs. ISP Pipeline — Two Separate Inference Paths

SoftISP has **two distinct inference subsystems** that are easy to confuse:

| | **Neural Controller (Rectifier)** | **ISP Pipeline (MNN/Vulkan)** |
|---|---|---|
| **Purpose** | Predict ISP *parameters* (WB, CCM, tone, zoom) | Process *pixels* (Bayer → RGB → display) |
| **Input** | Small metadata tensors | Full-frame pixel tensors |
| **Input size** | hist[256] + meta[52] ≈ tiny | [1,1,H,W] or [1,3,H,W] = megapixels |
| **Output** | 20 ISP parameters | Processed image frame |
| **ONNX runtime** | `tract-onnx` (CPU, direct ONNX) | ONNX → `.mnn` via `libMNNConvertDeps`, then MNN/Vulkan |
| **Graph source** | Pre-trained `fusedispcontroller_*.onnx` | Built from `IspBlock` chain via `GraphComposer` |
| **Conversion** | None — loads ONNX directly | `convert_onnx_buffer()` ONNX→MNN |
| **Engine** | `isp_rectifier::OptimizedInference` | `cam_isp::mnnengine::MnnEngine` |
| **Device** | CPU | Vulkan GPU |
| **Per-frame cost** | ~1–2 ms (tiny tensors) | ~50–140 ms at 4K (full-frame) |

They are **complementary**, not alternatives:

```
┌─────────────────────────────────────────────────────────────────┐
│                      Per-frame camera loop                      │
│                                                                   │
│  1. Neural Controller (rectifier)                               │
│     └─ analyze_and_update(frame) → IspParams (WB, CCM, …)      │
│           tract ONNX, small metadata tensors, ~1 ms              │
│                                                                   │
│  2. ISP Pipeline (MNN/Vulkan)                                    │
│     └─ engine.process(ProcessParams { isp_params, … })          │
│           ONNX→MNN converted graph, full-frame pixels, ~100 ms   │
│                                                                   │
│  IspParams from step 1 are fed into step 2 via                  │
│  ProcessParams.isp_params, so the neural model's WB/CCM/tone    │
│  values drive the pixel pipeline blocks.                         │
└─────────────────────────────────────────────────────────────────┘
```

## 1. Neural Controller (isp-rectifier)

**Role**: Replaces traditional rule-based 3A (AE/AWB/CCM/tone heuristics) with a single distilled ONNX model.

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
     │   ├─ noise_level          → from adjacent-bin variation      │
     │   └─ wb_gains/ae/af/awb   → from 3A state (or defaults)      │
     │
     ├─ FrameMetadata::to_feature_vector()                          │
     │     → hist [f32; 256] (sum-normalized to 10,000)             │
     │     → meta [f32; 52]   (CCT, WB, exposure, ISO, AF, …)       │
     │
     ├─ rectifier.optimize(&metadata)                               │
     │   ├─ tract-onnx loads fusedispcontroller_light_int8.onnx     │
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

**Key point**: The rectifier model is a **pre-trained ONNX file** (`fusedispcontroller_light.onnx`, `_int8`, `_fp16`). It is **not** built from `IspBlock`s. It is loaded directly by `tract-onnx` — no `convert_onnx_buffer`, no MNN session, no Vulkan. The model file lives in `isp-rectifier/models/`.

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

## 2. ISP Pipeline (cam-isp MNN/Vulkan)

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

**Key point**: The pipeline graph is **built from `IspBlock`s**, compiled to ONNX by `GraphComposer`, then converted ONNX→MNN by `convert_onnx_buffer()`, then executed by `MnnEngine` on Vulkan. The `IspParams` (WB, CCM, tone) from the neural controller are injected as extra-input tensors (`set_extra_inputs`) so the pixel blocks use the neural model's values.

**This is where `convert_onnx_buffer` is used** — NOT in the neural controller.

## 3. Why they differ

| Question | Answer |
|----------|--------|
| Does the neural controller convert ONNX→MNN? | **No.** It loads the pre-trained ONNX directly via `tract-onnx`. |
| Does the pipeline use `tract-onnx`? | **No.** It uses ONNX→MNN conversion + MNN/Vulkan. |
| Where does `convert_onnx_buffer` fit? | Pipeline only (block chain → ONNX → .mnn → MNN session). |
| Where does `OptimizedInference::new(model_path)` fit? | Neural controller only (pre-trained ONNX → tract runtime). |
| Can the neural controller run on GPU/Vulkan? | Not currently — `tract-onnx` is CPU. The model is tiny (metadata only), so CPU inference (~1 ms) is negligible vs. the pixel pipeline. |
| Can the pipeline run without the neural controller? | Yes — `IspController` (rule-based) provides `IspParams` instead. |

## 4. Code locations

```
Neural Controller (rectifier):
├── isp-rectifier/src/types.rs          # FrameMetadata, to_feature_vector() (52-dim meta)
├── isp-rectifier/src/inference.rs      # OptimizedInference, tract-onnx runner
├── isp-rectifier/src/register_injector.rs  # ISPOptimizedParams → ISPRegisters
├── cam-rust/cam-isp/src/neural_controller.rs   # NeuralController + fallback
├── cam-rust/cam-isp/src/rectifier_model.rs     # Mock ONNX generator (hist[256]+meta[52])
├── cam-rust/cam-isp/src/controller_api.rs      # ControllerApi trait + Controller enum
├── cam-rust/cam-isp/src/unified_pipeline.rs    # Wires NeuralController as default
└── isp-rectifier/models/fusedispcontroller_light*.onnx  # Pre-trained models

ISP Pipeline (MNN/Vulkan):
├── cam-rust/cam-isp/src/mnnengine.rs     # MnnEngine (Vulkan session, process())
├── cam-rust/cam-isp/src/mnn_converter.rs # convert_onnx_buffer() ONNX→.mnn
├── cam-rust/cam-isp/src/mnn_sys.rs       # FFI to libmnn_wrapper.so / libMNN.so
├── cam-rust/cam-isp/src/pipeline/types.rs # GraphComposer (IspBlock → ONNX)
├── cam-rust/cam-isp/src/engine.rs        # IspEngine trait, ProcessParams
├── cam-rust/cam-isp/src/blocks/          # IspBlock implementations
└── cam-rust/cam-isp/src/unified_pipeline.rs # feeds isp_params into ProcessParams
```

## 5. How the two connect

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

## 6. Verification

```
# Neural controller (tract ONNX, CPU):
cargo test -p cam-isp --features rectifier --lib        # 818 pass
cargo test -p cam-isp --test test_neural_controller_onnx --features rectifier  # 6 pass
cargo test -p cam-isp --test test_rectifier_integration --features rectifier   # 6 pass
cargo test -p isp-rectifier --features onnx-runtime --lib  # 6 pass

# Pre-trained model loads & runs (hist[256]+meta[52] → 4 outputs):
cargo run -p isp-rectifier --example inspect_models --features onnx-runtime
#   [256]+[52]: OK, 4 outputs: ["[1, 3]", "[1, 9]", "[1, 7]", "[1, 1]"]
```

## 7. TODO / open questions

- [ ] Add an MNN-based rectifier engine (CPU) so the neural model can run via MNN instead of tract, unifying the inference stack. Currently `tract-onnx` is a separate dependency used only by `isp-rectifier`.
- [ ] Wire `cam-ipa` `IpaConfig.neural_enabled` to select `NeuralController` vs `IspController`.
- [ ] Add `ToneLutBlock` support so the 7-point tone curve LUT from the neural model drives the pixel pipeline's tone block.
- [ ] Benchmark end-to-end: neural controller (~1 ms) + pipeline (~100 ms at 4K) = frame budget.
