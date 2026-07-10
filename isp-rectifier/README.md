# ISP Rectifier - Distilled Controller for ISP Pipeline

A complete **teacher-student distillation pipeline** that fuses CCMNet, Time-Aware AWB, and Neural ISP Tuning into a single lightweight ONNX model for real-time ISP parameter prediction.

## 🎯 What Is This?

This is a **distilled neural network** that predicts ISP parameters (White Balance, Color Correction Matrix, Tone Mapping, Zoom) from camera sensor data. It replaces multiple traditional 3A algorithms with a single fast inference.

**Distilled from:**
- **CCMNet** - Color Correction Matrix prediction
- **Time-Aware AWB** - Auto White Balance with temporal consistency
- **Neural ISP Tuning** - Tone mapping and zoom factor prediction

## 📦 Pre-trained Models

### Download from GitHub Releases

```bash
# Light models (fastest, recommended for production)
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller_light.onnx      # 490 KB
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller_light_int8.onnx  # 490 KB
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller_light_fp16.onnx  # 277 KB

# Medium models (balanced)
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller_medium.onnx      # 1.3 MB
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller_medium_int8.onnx  # 1.3 MB
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller_medium_fp16.onnx  # 0.7 MB

# Full models (highest accuracy)
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller.onnx             # 5.6 MB
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller_int8.onnx        # 5.6 MB
wget https://github.com/99degree/softisp/releases/download/models/fusedispcontroller_fp16.onnx        # 2.9 MB
```

### Model Variants

| Model | Size | Best For | Accuracy |
|-------|------|----------|----------|
| **Light** | 490 KB | Mobile, embedded | Good |
| **Medium** | 1.3 MB | Balanced | Better |
| **Full** | 5.6 MB | Desktop, server | Best |

### Quantization Formats

| Format | Description | Use Case |
|--------|-------------|----------|
| **FP32** | Full precision | Development, debugging |
| **INT8** | 8-bit integers | Production (fastest) |
| **FP16** | Half precision | GPU inference |

## 📊 Performance Benchmarks

### Tract ONNX Runtime (Debug Mode)

| Model | Size | Load Time | Inference | FPS |
|-------|------|-----------|-----------|-----|
| Light INT8 | 490 KB | 180ms | 11ms | **93** |
| Light FP32 | 490 KB | 303ms | 18ms | 57 |
| Medium FP32 | 1.3 MB | 199ms | 20ms | 55 |
| Medium INT8 | 1.3 MB | 209ms | 20ms | 55 |
| Full FP32 | 5.6 MB | 347ms | 23ms | 43 |
| Full INT8 | 5.6 MB | 343ms | 23ms | 43 |

**Recommendation:** Use **Light INT8** for production (93 FPS, 490 KB)

### Model Architecture

```
Input: histogram[256] + metadata[11] = 267 dims
        ↓
    ┌───┴───┐
    │ CNN   │ (1D convolution on histogram)
    │ MLP   │ (metadata encoding)
    └───┬───┘
        ↓
    Fusion Layer (256 dims)
        ↓
    ┌───┼───┬───┐
    ↓   ↓   ↓   ↓
   WB CCM Tone Zoom
  (3) (9) (7)  (1)  = 20 dims output
```

## 🚀 Quick Start

### Python (Training)

```bash
# Install dependencies
pip install torch onnx numpy onnxruntime

# Generate mock model (for testing)
python gen_mock.py --all --benchmark

# Train with real data
python distill_model.py --train \
    --dataset teacher_dataset/teacher_dataset.npz \
    --epochs 100

# Export ONNX
python distill_model.py --export \
    --model checkpoints/best_model.pth \
    --output fusedispcontroller.onnx \
    --quantize
```

### Rust (Inference)

```rust
use tract_onnx::prelude::*;

// Load model
let model = tract_onnx::onnx()
    .model_for_path("fusedispcontroller_light_int8.onnx")?
    .into_optimized()?
    .into_runnable()?;

// Create input tensors
let hist = Tensor::from_shape(&[1, 256], &histogram_data)?;
let meta = Tensor::from_shape(&[1, 11], &metadata_data)?;

// Run inference
let outputs = model.run(tvec![hist.into(), meta.into()])?;

// Parse outputs
let wb = outputs[0].to_array_view::<f32>()?;    // [1, 3] WB gains
let ccm = outputs[1].to_array_view::<f32>()?;   // [1, 9] CCM matrix
let tone = outputs[2].to_array_view::<f32>()?;   // [1, 7] Tone curve
let zoom = outputs[3].to_array_view::<f32>()?;   // [1, 1] Zoom factor
```

### Integration with cam-rust

```rust
use cam_isp::neural_controller::NeuralController;
use cam_isp::controller_api::ControllerApi;

// Create controller with pre-trained model
let mut ctrl = NeuralController::with_model("models/fusedispcontroller_light_int8.onnx")?;

// Process frame
let frame = IspFrame { ... };
let params = ctrl.analyze_and_update(&frame);

// Apply to pipeline
// params.wb = [R_gain, G_gain, B_gain]
// params.ccm = [[r,r,r], [g,g,g], [b,b,b]]
// params.tone = {contrast, brightness, gamma, ...}
```

## 📁 Project Structure

```
isp-rectifier/
├── models/                              # Pre-trained ONNX models
│   ├── fusedispcontroller.onnx          # Full FP32 (5.6 MB)
│   ├── fusedispcontroller_int8.onnx     # Full INT8 (5.6 MB)
│   ├── fusedispcontroller_fp16.onnx     # Full FP16 (2.9 MB)
│   ├── fusedispcontroller_medium.onnx   # Medium FP32 (1.3 MB)
│   ├── fusedispcontroller_medium_int8.onnx
│   ├── fusedispcontroller_medium_fp16.onnx
│   ├── fusedispcontroller_light.onnx    # Light FP32 (490 KB)
│   ├── fusedispcontroller_light_int8.onnx
│   └── fusedispcontroller_light_fp16.onnx
├── src/                                 # Rust library
│   ├── lib.rs
│   ├── types.rs                         # Input/output schemas
│   ├── inference.rs                     # ONNX inference
│   └── register_injector.rs             # ISP register mapping
├── tests/
│   ├── test_model_loading.rs            # Model validation tests
│   └── bench_inference.rs               # Performance benchmarks
├── distill_model.py                     # Training + export
├── gen_mock.py                          # Mock model generator
├── requirements.txt                     # Python deps
└── Cargo.toml                           # Rust crate config
```

## 🧪 Testing

```bash
# Run model loading tests
cargo test --test test_model_loading

# Run benchmarks
cargo test --test bench_inference -- --nocapture

# Python validation
python batch_validate_quantization.py \
    --fp32_model models/fusedispcontroller.onnx \
    --quant_model models/fusedispcontroller_int8.onnx \
    --samples 1000
```

## 📝 Input/Output Specification

### Input (267 dims)

| Index | Feature | Description |
|-------|---------|-------------|
| 0-255 | histogram | 256-bin luminance histogram |
| 256 | cct | Color temperature (Kelvin) |
| 257-259 | wb_gains | Current WB [R, G, B] |
| 260 | exposure_time | Exposure time (seconds) |
| 261 | iso_gain | ISO/gain value |
| 262 | focus_position | Focus position (0-1) |
| 263 | sharpness | Sharpness metric |
| 264 | brightness | Mean luminance |
| 265 | contrast | Contrast metric |
| 266 | noise_level | Estimated noise |

### Output (20 dims)

| Index | Output | Description |
|-------|--------|-------------|
| 0-2 | wb[3] | White balance gains [R, G, B] |
| 3-11 | ccm[9] | Color correction matrix (3×3) |
| 12-18 | tone[7] | Tone curve control points |
| 19 | zoom[1] | Digital zoom factor |

## 📄 License

### Code License

The Rust/Python code in this repository is licensed under:
- **MIT OR Apache-2.0** (your choice)

### Model License

**⚠️ The pre-trained ONNX models are NOT MIT/Apache licensed.**

The student models are distilled from teacher models with the following licenses:

| Teacher Model | License |
|---------------|--------|
| CCMNet | Creative Commons Attribution-NonCommercial 4.0 (CC BY-NC 4.0) |
| Time-Aware AWB | CC BY-NC 4.0 |
| Neural ISP Tuning | CC BY-NC 4.0 |

**Therefore, the distilled student models inherit the CC BY-NC 4.0 license:**
- ✅ **Free for non-commercial use**
- ✅ **Must give attribution**
- ❌ **No commercial use without permission**

### Attribution Required

If you use these models, you must credit:
```
ISP Rectifier - Distilled ISP Controller
https://github.com/99degree/softisp

Based on teacher models:
- CCMNet (Creative Commons Attribution-NonCommercial 4.0)
- Time-Aware AWB (Creative Commons Attribution-NonCommercial 4.0)
- Neural ISP Tuning (Creative Commons Attribution-NonCommercial 4.0)
```

### Commercial Use

For commercial licensing, contact the original teacher model authors or retrain with your own data.

See [TEACHER_DATASET.md](TEACHER_DATASET.md) and [TEACHER_ANALYSIS.md](TEACHER_ANALYSIS.md) for full license details.

## 🙏 Acknowledgments

- **CCMNet** - Color correction matrix prediction
- **Time-Aware AWB** - Temporal white balance
- **Neural ISP Tuning** - Learned ISP parameter optimization
- **MNN** - Mobile Neural Network inference framework
- **Tract** - ONNX runtime for Rust

---

**Built for:** Real-time ISP parameter prediction on mobile/embedded devices  
**Target:** < 5ms inference on Snapdragon 8 Gen 2  
**Integration:** Rust pipeline via `tract-onnx` or MNN Vulkan
