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

## Neural Controller

### Model Architecture

```
Input: histogram[256] + metadata[11] = 267 dims
        │
        ▼
    ┌───┴───┐
    │  MLP  │ (256 → 128 → 64 → 20)
    └───┬───┘
        │
        ▼
Output: wb[3] + ccm[9] + tone[7] + zoom[1] = 20 dims
```

### Model Variants

| Model | Size | Latency | Use Case |
|-------|------|---------|----------|
| Light INT8 | 490 KB | 11ms | Production |
| Medium FP32 | 1.3 MB | 20ms | Development |
| Full FP32 | 5.6 MB | 23ms | Accuracy |

### Teacher Models

Distilled from:
- **CCMNet** - Color correction
- **Time-Aware AWB** - White balance
- **Neural ISP Tuning** - Tone mapping

**License:** CC BY-NC 4.0 (non-commercial)

## Performance

### Vulkan GPU (Snapdragon 8 Gen 2)

| Resolution | HEAVY | LITE |
|------------|-------|------|
| HD | 1.0ms | 4.9ms |
| FHD | 2.0ms | 8.0ms |
| 4K | 8.0ms | 24ms |

### Neural Controller

| Model | FPS (Tract) |
|-------|-------------|
| Light INT8 | 93 |
| Medium FP32 | 55 |
| Full FP32 | 43 |

## Module Structure

```
cam-isp/src/
├── blocks/           # ISP processing blocks
├── pipeline/         # Pipeline composition
├── mnn/             # MNN backend
├── simd/            # SIMD optimizations
├── isp_params.rs    # Controller parameters
├── isp_controller.rs # Rule-based controller
├── neural_controller.rs # Neural controller
├── controller_api.rs # Unified controller trait
├── engine.rs        # Engine abstraction
└── unified_pipeline.rs # Main pipeline
```
