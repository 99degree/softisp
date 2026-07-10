# SoftISP Project Summary

## What Is SoftISP?

SoftISP is a **complete camera ISP (Image Signal Processor) pipeline** implemented in Rust, designed for real-time image processing on mobile and embedded devices. It converts raw Bayer sensor data into display-ready images using GPU acceleration via Vulkan.

## Key Features

### 1. Full ISP Pipeline

```
Raw Bayer → BLC → White Balance → Demosaic → CCM → Tone → Display
```

- **40+ ISP blocks** covering all standard camera processing
- **Multiple pipeline profiles** (LITE/MED/HEAVY/PRO/UNIFIED)
- **Real-time processing** at HD/FHD/4K resolutions

### 2. GPU Acceleration

- **Vulkan backend** via MNN framework
- **Custom SPIR-V shaders** for ISP operations
- **Fused operations** to minimize GPU dispatches

Performance on Snapdragon 8 Gen 2:
| Resolution | HEAVY | LITE |
|------------|-------|------|
| HD (720p) | 1.0ms | 4.9ms |
| FHD (1080p) | 2.0ms | 8.0ms |
| 4K (2160p) | 8.0ms | 24ms |

### 3. Neural Controller

A distilled neural network that predicts ISP parameters:

- **267 inputs** (histogram + metadata)
- **20 outputs** (WB, CCM, tone, zoom)
- **93 FPS** on Light INT8 model
- **Distilled from** CCMNet, Time-Aware AWB, Neural ISP Tuning

### 4. Dual Controller System

```rust
// Rule-based (fast, no model needed)
let ctrl = IspController::new();

// Neural (accurate, needs ONNX model)
let ctrl = NeuralController::with_model("model.onnx")?;
```

Both implement `ControllerApi` trait for seamless switching.

### 5. AOSP HAL Integration

Complete Android Camera HAL3 implementation:
- AIDL binderized interface
- V4L2 backend for Linux
- NDK camera backend for Android

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      SoftISP Architecture                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │
│  │  Raw Sensor  │───▶│  ISP Block   │───▶│   Display    │     │
│  │  (Bayer)     │    │  Pipeline    │    │   Output     │     │
│  └──────────────┘    └──────┬───────┘    └──────────────┘     │
│                             │                                  │
│                    ┌────────▼────────┐                        │
│                    │   Controller    │                        │
│                    │  (Rule/Neural)  │                        │
│                    └─────────────────┘                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Project Structure

```
softisp/
├── cam-rust/           # Main Rust workspace
│   ├── cam-isp/        # Core ISP library
│   ├── cam-binder/     # AIDL HAL
│   ├── cam-core/       # Camera core
│   ├── cam-hal-linux/  # V4L2 backend
│   └── cam-hal-android/# NDK backend
│
├── isp-rectifier/      # Neural controller
│   ├── models/         # ONNX models (9 variants)
│   └── src/            # Inference code
│
├── vulkan_isp/         # Vulkan shaders
│   ├── *.comp          # GLSL compute shaders
│   └── tests/          # C++ tests
│
├── cpp/                # Legacy C++ code
├── teachers/           # Teacher models
└── docs/               # Documentation
```

## Quick Start

### Rust (Library)

```rust
use cam_isp::engine::{select_engine_by_name, ProcessParams};
use cam_isp::profile::PipelineProfile;

// Create engine
let mut engine = select_engine_by_name("mnn_vulkan")?;

// Build pipeline
let blocks = PipelineProfile::HEAVY.build_blocks(1920, 1080);

// Process frame
let params = ProcessParams::new(3840, 2160, &raw_bayer);
let frame = engine.process(&params)?;
```

### Python (Model Training)

```bash
# Install dependencies
pip install torch onnx numpy

# Train model
python distill_model.py --train --dataset data.npz

# Export ONNX
python distill_model.py --export --output model.onnx
```

## Model Downloads

Pre-trained ONNX models available at:
```
https://github.com/99degree/softisp/releases
```

| Model | Size | FPS | Best For |
|-------|------|-----|----------|
| Light INT8 | 490 KB | 93 | Production |
| Medium FP32 | 1.3 MB | 55 | Development |
| Full FP32 | 5.6 MB | 43 | Accuracy |

**License:** CC BY-NC 4.0 (non-commercial)

## Testing

```bash
# Run all tests
cargo test -p cam-isp

# Run with GPU
cargo test -p cam-isp --features mnn

# Run benchmarks
cargo run --example bench_heavy --release
```

## Build Status

- ✅ **706+ unit tests passing**
- ✅ **Vulkan GPU backend working**
- ✅ **Neural controller integrated**
- ✅ **4K processing verified**
- ✅ **AOSP HAL implemented**

## Documentation

See `docs/` folder:
- `docs/architecture/ARCHITECTURE.md` - System design
- `docs/architecture/PIPELINE_BLOCKS.md` - Block reference
- `docs/api/CONTROLLER_API.md` - Controller API
- `docs/guides/MNN_VULKAN_GUIDE.md` - GPU setup
- `docs/performance/` - Benchmarks
- `docs/testing/TESTING.md` - Test guide

## License

**Code:** MIT OR Apache-2.0

**Models:** CC BY-NC 4.0 (inherited from teacher models)
- Non-commercial use only
- Attribution required
- See `TEACHER_ANALYSIS.md` for details
