# SoftISP Architecture

## Overview

SoftISP is a complete camera ISP (Image Signal Processor) pipeline implemented in Rust, with GPU acceleration via Vulkan/MNN. It processes raw Bayer sensor data into display-ready images.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SoftISP Architecture                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                 │
│  │  Raw Sensor  │───▶│  ISP Block   │───▶│   Display    │                 │
│  │  (Bayer)     │    │  Pipeline    │    │   Output     │                 │
│  └──────────────┘    └──────┬───────┘    └──────────────┘                 │
│                             │                                              │
│                    ┌────────▼────────┐                                    │
│                    │   Controller    │                                    │
│                    │  (Rule/Neural)  │                                    │
│                    └─────────────────┘                                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. Pipeline System

The pipeline is built from composable blocks that process image data:

```rust
// Pipeline building
let mut blocks = profile.build_blocks(width, height);
GraphComposer::wire_blocks(&mut blocks);

// Processing
let frame = engine.process(&params)?;
```

**Block Types:**
- **Input Blocks**: Raw data ingestion (RawInputBlock)
- **Processing Blocks**: ISP operations (BLC, WB, Demosaic, CCM, etc.)
- **Output Blocks**: Format conversion (DisplayBlock)

### 2. Controller System

Controllers analyze scene and compute ISP parameters:

```rust
// Rule-based controller
let mut ctrl = IspController::new();
let params = ctrl.analyze_and_update(&frame);

// Neural controller (ONNX inference)
let mut ctrl = NeuralController::with_model("model.onnx")?;
let params = ctrl.analyze_and_update(&frame);
```

**Controller Output (IspParams):**
- `wb`: White balance gains [R, G, B]
- `ccm`: Color correction matrix (3×3)
- `tone`: Tone mapping (contrast, brightness, gamma)
- `zoom`: Digital zoom factor

### 3. Engine System

Engines execute pipelines on different backends:

```rust
// Vulkan GPU engine
let mut engine = select_engine_by_name("mnn_vulkan").unwrap();

// CPU fallback
let mut engine = select_engine_by_name("mnn_cpu").unwrap();
```

## Data Flow

```
Raw Bayer Sensor (INT16)
        │
        ▼
┌───────────────────┐
│   RawInputBlock   │  Converts INT16 to float
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│    BlcBlock       │  Black level correction
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│  BayerWbBlock     │  White balance
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│ DemosaicCcmBlock  │  Bayer → RGB + color correction
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│    ToneBlock      │  Tone mapping
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│  DisplayBlock     │  Output format (ARGB/RGB/FP16)
└─────────┬─────────┘
          │
          ▼
    Display-Ready Image
```

## Pipeline Profiles

| Profile | Blocks | Use Case |
|---------|--------|----------|
| **LITE** | 12 | Mobile, low power |
| **MED** | 14 | Balanced |
| **HEAVY** | 16 | Quality priority |
| **PRO** | 18 | Professional |
| **UNIFIED** | 18 | Full features |

## GPU Acceleration

### MNN + Vulkan Backend

The pipeline is executed on GPU via MNN's Vulkan backend:

1. **ONNX Model**: Pipeline compiled to ONNX graph
2. **MNN Conversion**: ONNX → MNN format
3. **Vulkan Execution**: GPU compute shaders

### Custom ISP Opsets

Custom SPIR-V shaders for ISP operations:
- `isp.unpack_packed` - INT32 Bayer unpacking
- `isp.argb_convert` - RGB to ARGB conversion
- `isp.yuv420_convert` - YUV420 encoding
- `isp.gamma` - Gamma correction (WIP)

> **The ONNX graph is a dispatch pattern, not an algorithm proof.** softISP emits
> standard ONNX ops specifically so MNN's fusion rules match them and *rewrite*
> them into the custom `isp.*` opsets above. The op sequence is a *trigger* for
> MNN's custom-opset GPU path — it does **not** assert that the numerical result
> of every block has been verified against the pure-Rust `CpuEngine` reference.
> Use the GPU path for speed; `CpuEngine` remains the source of truth for semantics.
>
> **If you add a new algorithm as an `IspBlock`**, confirm that the ONNX pattern
> you emit is recognized by `libMNNConvertDeps` and fused into the intended
> `isp.*` opset. An unrecognized pattern silently falls back to the **standard
> ONNX→MNN opset**; a *partially* matching (but semantically different) pattern
> can be mis-classified as an existing `isp.*` opset and dispatched to the
> **wrong** custom shader — producing **incorrect output with no diagnostic**.
> Always validate a new block's MNN/GPU result numerically against `CpuEngine`.

## Neural Controller (isp-rectifier)

### Purpose

The Neural Controller predicts ISP *parameters* (WB gains, CCM matrix, tone curve,
digital zoom) from small frame-metadata tensors. It is a **separate inference path**
from the pixel pipeline — it does **not** run on MNN/Vulkan and does **not** go
through ONNX→MNN conversion.

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
     │   ├─ tract-onnx loads fusedispcontroller_light_int8.onnx     │
     │   ├─ inputs: hist [1,256] F32 + meta [1,52] F32              │
     │   ├─ outputs: wb [1,3], ccm [1,9], tone [1,7], zoom [1,1]   │
     │   └─ clamp to RegisterLimits                                 │
     │
     └─ → IspParams (wb, ccm, tone.curve_lut, zoom, …)
```

### Interface (verified on pre-trained models)

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

The model file (`isp-rectifier/models/fusedispcontroller_light*.onnx`) is a
pre-trained ONNX loaded directly by `tract-onnx` — no `convert_onnx_buffer`,
no MNN session, no Vulkan. The tiny metadata tensors make CPU inference (~1 ms)
negligible versus the full-frame pixel pipeline.

### Fallback

If the model fails to load or inference fails, `NeuralController` silently falls
back to `IspController` (rule-based AE/AWB/tone/denoise). The pipeline never stalls.

### Connection to the pixel pipeline

`NeuralController` output (`IspParams`) is fed into `ProcessParams.isp_params` in
`UnifiedPipeline::process()`, and `MnnEngine` writes WB/CCM/tone as graph
extra-inputs so the pixel blocks use the neural model's predictions.

## Module Structure

```
cam-isp/src/
├── blocks/           # ISP processing blocks
├── pipeline/         # Pipeline composition
├── mnnengine.rs      # MnnEngine (Vulkan session, process())
├── mnn_converter.rs  # convert_onnx_buffer() ONNX→.mnn
├── mnn_sys.rs        # FFI to libmnn_wrapper.so / libMNN.so
├── engine.rs         # IspEngine trait, ProcessParams
├── isp_params.rs     # Controller parameters
├── isp_controller.rs # Rule-based controller (IspController)
├── neural_controller.rs # NeuralController (tract ONNX + fallback)
├── controller_api.rs # ControllerApi trait + Controller enum
├── rectifier_model.rs # Mock ONNX generator (hist[256]+meta[52])
└── unified_pipeline.rs # Wires controller → ProcessParams.isp_params

isp-rectifier/  (separate crate, rectifier feature)
├── src/types.rs       # FrameMetadata, to_feature_vector() (52-dim meta)
├── src/inference.rs   # OptimizedInference, tract-onnx runner
├── src/register_injector.rs  # ISPOptimizedParams → ISPRegisters
└── models/fusedispcontroller_light*.onnx  # Pre-trained models
```

### Data flow — controller → pipeline

```
Neural Controller (tract ONNX, CPU)        ISP Pipeline (MNN/Vulkan, GPU)
─────────────────────                      ─────────────────────────────
frame → extract_metadata()                 ProcessParams {
   → to_feature_vector()  hist[256]+meta[52]   buf, width, height,
   → tract-onnx optimize()   → IspParams       isp_params: Some(IspParams),
   → IspParams {wb,ccm,tone,zoom}             ccm_matrix, awb_gains, …
                                                          │
                                                          ▼
                                                    engine.process(&params)
                                                    MNN Vulkan session
                                                    (ONNX→MNN converted graph)
                                                    → IspFrame (display image)
```

The controller runs first (~1 ms on CPU), producing `IspParams`. Those params are
injected into `ProcessParams` and reach the pixel pipeline via `MnnEngine`'s
`set_extra_inputs()` as graph input tensors, so `BayerWbBlock`, `DemosaicCcmBlock`,
and `ToneBlock` use the neural model's predictions.
