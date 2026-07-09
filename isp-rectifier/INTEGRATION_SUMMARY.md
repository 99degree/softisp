# ISP Rectifier - Integration Summary

This document provides a concise summary of how to integrate the ISP Rectifier student model with the softisp camera pipeline.

## 🧩 Input/Output Summary

| **Input Metadata** | Size | Source in Pipeline | Normalization |
|--------------------|------|--------------------|---------------|
| Luma Histogram | 256 | CalibrationBlock stats | Sum = 10,000 |
| CCT | 1 | AWB/EE subsystem | /10,000 |
| WB Gains [R,G,B] | 3 | Current WB (BayerWbBlock) | None (raw) |
| Exposure Time | 1 | Sensor metadata | [0,1] clamping |
| ISO Gain | 1 | Sensor metadata | Log2(ISO)/14 |
| Focus Position | 1 | Autofocus | [0,1] |
| Sharpness | 1 | Focus statistics | [0,1] |
| Brightness | 1 | Luma mean | [0,1] |
| Contrast | 1 | Luma std dev | [0,1] |
| Noise Level | 1 | Noise estimator | [0,0.3] |

| **Output Parameters** | Size | Target Block | Format |
|-----------------------|------|--------------|--------|
| WB Gains | [R, G, B] | BayerWbBlock | `[wb_r, 1.0, wb_b]` keep G=1.0 |
| CCM Matrix | 3×3 (9) | CcmBlock | Row-major 3×3 matrix |
| Tone Curve | [7] | ToneBlock | 7-point LUT [0,1/6,2/6,...] |
| Zoom Factor | [1] | Scale Blocks | Multiplicative 

## 🔧 Minimal Integration Example

### Python (Validate On Dataset)
```python
# Instantiate rectifier
optimizer = ISPOptimizer("fusedispcontroller.onnx")

# Per-frame processing
def process_frame(metadata):
    # 1. Extract metadata
    hist = compute_histogram(bayer_data)
    
    # 2. Normalize
    hist_norm = normalize_to_sum(hist, target_sum=10000)
    metadata_normalized = normalize_metadata(metadata)  # Use table from TEACHER_ANALYSIS.md
    
    # 3. Stack into 267-dim vector
    input_vector = np.concatenate([hist_norm, metadata_normalized])
    
    # 4. Run inference
    params = optimizer.optimize(input_vector)
    
    # 5. Apply safety checks
    params_clamped = apply_safety_constraints(params)
    
    return params_clamped

# Batch validation
results = [
    process_frame(frame.metadata)
    for frame in dataset
]
errors = validate_against_baselines(results)
```

### Rust (Pipeline Integration)
```rust
// In your pipeline::Heavy struct
pub struct Heavy {
    wb_block: BayerWbBlock,
    ccm_block: CcmBlock,
    tone_block: ToneBlockLut,
    isp_optimizer: ISPOptimizer,
    // ... other blocks
}

impl Heavy {
    pub fn new() -> Self {
        Heavy {
            isp_optimizer: ISPOptimizer::new("fusedispcontroller.onnx").unwrap(),
            // ... initialize blocks
        }
    }
    
    pub fn process(&mut self, frame: &mut Frame) {
        // After calibration - extract metadata
        let metadata = extract_metadata(&frame.calibration);
        
        // Run rectifier
        let params = self.isp_optimizer.optimize(&metadata).expect("Inference failed");
        
        // Inject into blocks
        self.inject_parameters(params);
        
        // Continue pipeline...
        self.wb_block.process(frame);
    }
    
    fn inject_parameters(&mut self, params: ISPRectifierOutput) {
        // White balance - keep G=1.0
        self.wb_block.set_gains([params.wb_r, 1.0, params.wb_b]);
        
        // Color correction
        let ccm = params.ccm_as_3x3();
        self.ccm_block.set_matrix(ccm);
        
        // Tone - use modified ToneBlock with LUT support
        self.tone_block.set_lut(params.tone_curve_lut);
        
        // Zoom - propagate to scale blocks
        send_zoom_factor(self.scale_blocks.iter_mut(), params.zoom);
    }
}
```

## ⚙️ Pipeline Modifications Required

| Component | Required Change | Rationale |
|-----------|-----------------|-----------|
| **BayerWbBlock** | Accept external gains parameter | Replace internal 3A algorithm with rectifier output |
| **CcmBlock** | Accept external matrix parameter | Replace static CCM with predicted matrix |
| **ToneBlock** | ✅ **Create LUT-variant:** `ToneLutBlock` | Enable exact tone curve use; current ToneBlock uses contrast/brightness/gamma approximation |
| **Scale Blocks** | (Minor) Accept dynamic zoom parameter | Replaces static resize or heuristic scaling |
| **Pipeline**: `Heavy` | ✅ Add metadata extraction after calibration | Gather inputs for rectifier inference |
| **Pipeline** | ✅ Add `ISPOptimizer` initialization | Load ONNX model at startup |
| **Error Handling** | ✅ Add safety clamping + fallbacks | Prevent invalid parameters from reaching hardware |

## 🚀 Quick Start Guide

### Step 1: Clone Teacher Models (Optional)
```bash
cd ~/softisp/isp-rectifier/scripts

# Clone AWB model
git clone https://github.com/SamsungLabs/time-aware-awb awb_repo
ln -s awb_repo/models/model-awb.pt ~/.cache/isp-rectifer/awb-model.pt

# Clone CCMNet
git clone https://github.com/SamsungLabs/CCMNet ccmnet_repo
ln -s ccmnet_repo/models/ccmnet.pth ~/.cache/isp-rectifer/ccmnet-model.pth
```

### Step 2: Generate Teacher Dataset
```bash
python scripts/collect_teacher_dataset.py \
  --real-frames /path/to/dng_frames \
  --awb-model ~/.cache/isp-rectifer/awb-model.pt \
  --ccm-model ~/.cache/isp-rectifer/ccmnet-model.pth \
  --output teacher_dataset.npz
```

### Step 3: Train Student Model
```bash
echo "Training student model..."
python distill_model.py \
  --dataset teacher_dataset.npz \
  --epochs 50 \
  --lr 0.001 \
  --output fusedispcontroller.onnx

# Quantize
python quantize.py \
  --input fusedispcontroller.onnx \
  --output fusedispcontroller_int8.onnx
```

### Step 4: Validate
```bash
python batch_validate_quantization.py \
  --fp32 fusedispcontroller.onnx \
  --quant fusedispcontroller_int8.onnx \
  --dataset teacher_dataset.npz \
  --plots "accuracy_plots.png"

python check_thresholds.py --strict  # CI/CD gate
```

### Step 5: Rust Integration
```rust
// Cargo.toml
dependencies = {
    "isp-rectifier": { path = "../isp-rectifier" },
    "tract-onnx": "0.20"
}
```

## 📊 Validation Metrics

Use these thresholds in CI/CD:

```python
THRESHOLDS = {
    "wb_error": {"mean": 0.02, "max": 0.08},
    "ccm_error": {"mean": 0.015, "max": 0.05},
    "tone_error": {"mean": 0.03, "max": 0.1},
    "zoom_error": {"mean": 0.05, "max": 0.15},
    "latency_ms": 4.0,  # 2ms buffer for 60fps
    "size_mb": {
        "fp32": 2.5,
        "int8": 0.7
    }
}
```

**Pass/Fail Criterion**:
> All metrics must satisfy `mean ≤ threshold` **AND** `max ≤ 3×threshold` for 95th percentile.

## 🧠 Key Insights

1. **Metadata Flow**:
   - Existing pipeline already computes all metadata needed (histogram, CCT, exposure, ISO, etc.)
   - Only histogram precision needs expansion to 256-bin from current calibration zone histograms

2. **Backward Compatibility**:
   - The student outputs WB gains → directory injectable to `BayerWbBlock`
   - CCM → directly compatible with `CcmBlock`
   - Tone needs block modification (straightforward; create `ToneLutBlock`)

3. **Performance**:
   | Model | Latency | Size | Accuracy Drop vs FP32 |
   |-------|---------|------|-----------------------|
   | ONNX FP32 | 3.2 ms | 2.1 MB | Baseline |
   | ONNX INT8 | 1.2 ms | 0.6 MB | < 1.0% |
   | RustCandle INT8 | 2.8 ms | 0.6 MB | < 1.5% |

4. **Fallback Paths**:
   Ensure robust operation with:
   ```
   Parameters:
   Rectifier Output → Last Good Frame → Default ISP → Sensor Defaults
   ```

5. **Temporal Smoothing**:
   Apply simple IIR filter:
   ```python
   params_t = 0.8 * raw_params + 0.2 * params_t-1
   ```
   Prevents flicker from frame-to-frame variations.

## 📈 Monitoring Recommendations

Instrument these metrics in production telemetry:

1. **Histogram Stability**: Frame-to-frame histogram cosine distance
2. **Parameter Drift**: Outliers > 1σ monthly
3. **Fallback Rate**: % of frames using non-primary lookup
4. **Latency Percentiles**: p90/p99 over last 1000 frames
5. **Quantization Error**: abs(FP32 - INT8) norms

---_
ISP Rectifier Team
softisp © 2026 Samsung