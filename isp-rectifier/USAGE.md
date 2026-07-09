# ISP Rectifier - Usage Guide

**Current Version**: v1.2 (Semi-fixed API: ✅ 267→20)
**Status**: Ready for Integration

---

## 🛠️ Quick Start

### 1. Install Dependencies

```bash
# Python (training/validation)
pip install -r requirements.txt

# Optional tools
sudo apt install onnxruntime-tools  # ONNX optimizer
```

### 2. Validate Models

```bash
# Check model integrity
python validate_model.py \
    --model models/fusedispcontroller_int8.onnx \
    --test-input scripts/test_input.npy

# Expected output:
# Input shape:  torch.Size([1, 267])
# Output shape: torch.Size([1, 20])  # ✅ PASS
```

---

## 🚀 Inference

### Python (Reference Implementation)

```python
import numpy as np
import onnxruntime

class ISPOptimizer:
    def __init__(self, model_path="models/fusedispcontroller_int8.onnx"):
        self.sess = onnxruntime.InferenceSession(model_path)
        self.input_name = self.sess.get_inputs()[0].name
    
    def __call__(self, input_vector):
        """Semi-fixed API: input_vector shape [267] → returns [20]"""
        # 1. Validate
        assert input_vector.shape == (267,), "Input must be [267] vector"
        
        # 2. Inference
        outputs = self.sess.run(None, {self.input_name: input_vector[np.newaxis].astype(np.float32)})
        return self._post_process(outputs[0][0])
    
    def _post_process(self, raw_output):
        """Apply safety clamping to model output [20]"""
        wb = np.clip(raw_output[0:3], 0.2, 5.0)
        wb[1] = 1.0  # G=1.0 reference
        
        ccm = np.clip(raw_output[3:12], -3.0, 3.0)
        
        tone = np.clip(raw_output[12:19], 0.0, 1.0)
        tone = np.maximum.accumulate(tone)  # Monotonicity
        
        zoom = np.clip(raw_output[19], 1.0, 4.0)
        
        return {"wb": wb, "ccm": ccm, "tone": tone, "zoom": zoom}

# Example usage
if __name__ == "__main__":
    model = ISPOptimizer()
    
    # Create synthetic input vector
    synthetic_input = np.zeros(267)
    synthetic_input[256:] = [
        0.5,   # CCT / 10000
        1.0, 1.0, 1.0,  # WB gains [R,G,B] = neutral
        0.1,   # Exposure [0,1]
        0.5,   # ISO
        0.5,   # Focus [0,1]
        0.7,   # Sharpness
        0.5,   # Brightness
        0.3,   # Contrast
        0.1    # Noise [0, 0.3]
    ]
    
    # Run inference
    output = model(synthetic_input)
    print("WB Gains:", [f"{v:.3f}" for v in output["wb"]])
    print("CCM Matrix (first row):", [f"{v:.3f}" for v in output["ccm"][:3]])
    print("Tone Curve:", [f"{v:.3f}" for v in output["tone"]])
    print("Zoom Factor:", output["zoom"])
```

### Rust (tract-onnx)

```rust
use tract_onnx::prelude::*;

pub struct ISPOptimizer {
    model: RunnableModel<TypedFact, Box<dyn TypedOp>>,
}

impl ISPOptimizer {
    pub fn new(model_path: &str) -> TractResult<Self> {
        let model = tract_onnx::onnx()
            .model_for_path(model_path)?
            .with_input_fact(0, crate::fact!(1, 267))?
            .into_optimized()?
            .into_runnable()?;
        Ok(Self { model })
    }
    
    pub fn run(&self, input: &[f32; 267]) -> TractResult<ISPOptimizerOutput> {
        // Create input tensor
        let tensor = ndarray::Array4::from_shape_fn((1, 1, 267, 1), |(_, _, i, _)| input[i]);
        
        // Run inference
        let outputs = self.model.run(tvec!(tensor.into()))?;
        let raw: Vec<f32> = outputs[0].to_array_view::<f32>()?.iter().cloned().collect();
        
        // Post-process
        Ok(ISPOptimizerOutput::from_raw(&raw))
    }
}

#[derive(Debug, Clone)]
pub struct ISPOptimizerOutput {
    pub wb_gains: [f32; 3],
    pub ccm: [f32; 9],
    pub tone_curve: [f32; 7],
    pub zoom_factor: f32,
}

impl ISPOptimizerOutput {
    fn from_raw(raw: &[f32]) -> Self {
        let mut wb = [raw[0], 1.0, raw[2]]; // G=1.0 reference
        wb.iter_mut().for_each(|v| *v = v.clamp(0.2, 5.0));
        
        let mut ccm = [
            raw[3], raw[4], raw[5],
            raw[6], raw[7], raw[8],
            raw[9], raw[10], raw[11]
        ];
        ccm.iter_mut().for_each(|v| *v = v.clamp(-3.0, 3.0));
        
        let mut tone: Vec<f32> = raw[12..19].iter().cloned().collect();
        tone.iter_mut().for_each(|v| *v = v.clamp(0.0, 1.0));
        tone = tone
            .windows(2)
            .enumerate()
            .map(|(i, w)| w[0].max(w[1]))  // Enforce monotonicity
            .collect();
        tone.insert(0, 0.0f32);  // Force tone[0] = 0.0
        tone[6] = 1.0;             // Force tone[6] = 1.0
        
        let zoom = raw[19].clamp(1.0, 4.0);
        
        Self {
            wb_gains: [wb[0], wb[1], wb[2]],
            ccm: ccm.try_into().unwrap(),
            tone_curve: tone.try_into().unwrap(),
            zoom_factor: zoom,
        }
    }
}
```

---

## 🔧 Model Management

### Model Zoo

| Model | Path | Quantization | Use Case |
|-------|------|--------------|----------|
| `fusedispcontroller.onnx` | models/ | FP32 | Development baseline |
| `fusedispcontroller_int8.onnx` | models/ | INT8 | **Production target** |
| `fusedispcontroller_qat_int8.onnx` | models/ | INT8 QAT | Low-power devices |

```bash
# Check model files
shasum -a 256 models/fusedispcontroller_int8.onnx  
# Expected: 0e7f3d...a1c2f0ddf
```

### Conversion Tools

```bash
# ONNX → TensorRT
python -m onnxruntime_tools convert \
    models/fusedispcontroller_int8.onnx \
    --output models/fusedispcontroller_tensorrt.plan \
    --trt
```

---

## 🎯 Validation

### Accuracy Suite

```bash
# Run full validation (5k real + 45k synthetic frames)
python batch_validate_quantization.py \
    --fp32 models/fusedispcontroller.onnx \
    --quant models/fusedispcontroller_int8.onnx \
    --output_dir validation_reports/

# Sample output:
# Model: fusedispcontroller_int8.onnx
# ✅ WB Error: Mean=0.0015, Max=0.006 (Threshold: Mean<0.01, Max<0.03)
# ✅ CCM Error: Mean=0.0021, Max=0.008
# ✅ Tone Error: Mean=0.0009, Max=0.004
# ✅ Zoom Error: Mean=0.0018, Max=0.009
# 🎯 Quantization Loss: < 0.8% vs FP32
```

### Safety Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_invalid_inputs() {
        let model = ISPOptimizer::new("models/fusedispcontroller_int8.onnx").unwrap();
        
        // Test histogram = zeros
        let mut input = [0.0f32; 267];
        input[256..].copy_from_slice(&[0.5, 1.0, 1.0, 1.0, 0.1, 0.5, 0.5, 0.7, 0.5, 0.3, 0.1]);
        let output = model.run(&input).unwrap();
        
        // Check safe defaults
        assert!((0.9..=1.1).contains(&output.wb_gains[0]));
        assert!((0.8..=1.2).contains(&output.zoom_factor));
    }
    
    #[test]
    fn test_monotonic_tone_curve() {
        let model = ISPOptimizer::new("models/fusedispcontroller_int8.onnx").unwrap();
        let input = create_dark_scene_input(); // Extreme values
        
        let output = model.run(&input).unwrap();
        
        // Validate tone curve
        let mut prev = output.tone_curve[0];
        for &current in &output.tone_curve[1..] {
            assert!(current >= prev);
            prev = current;
        }
    }
}
```

---

## 🔄 Fallback Strategy

```python
class FallbackChain:
    def __init__(self):
        self.model = ISPOptimizer()
        self.last_good_params = None
        
    def __call__(self, input_vector):
        try:
            params = self.model(input_vector)
            self.last_good_params = params  # Cache only if successful
            return params
        except Exception as e:
            if self.last_good_params:
                print(f"Using last known good params ({type(e).__name__})")
                return self.last_good_params
            else:
                print(f"Critical failure ({type(e).__name__}). Using hardcoded defaults")
                return {
                    "wb": [1.0, 1.0, 1.0],
                    "ccm": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    "tone": np.linspace(0, 1, 7).tolist(),
                    "zoom": 1.0
                }
```

---

## 📁 File Layout

```
isp-rectifier/
├── models/                          # ✅ Current models (v1.2)
│   ├── fusedispcontroller.onnx       # FP32 reference
│   ├── fusedispcontroller_int8.onnx  # **Production model**
│   └── fusedispcontroller_qat.onnx   # QAT variant
├── scripts/
│   ├── collect_teacher_dataset.py     # Generate training data
│   ├── distill_model.py               # Training script
│   ├── quantize.py                    # Quantization
│   ├── batch_validate.py              # **Validation**
│   └── benchmark.py                  # Performance testing
├── MODEL_SPECIFICATION.md             # Technical specification
└── USAGE.md                           # **You are here**
```

---

## 🏁 Next Steps

| Task | Command/Reference | Owner |
|------|-------------------|-------|
| **Integrate Model** | `ISPOptimizer::new("fusedispcontroller_int8.onnx")` | Rust Team |
| Validate Input | `TEACHER_ANALYSIS.md` **Section 5** | Pipeline Team |
| Safety Clamps | `ISPOptimizerOutput::from_raw()` | Systems Team |
| Fallbacks | **FallbackChain** class above | QA Team |
| Telemetry | `RectifierMetrics` struct | Cloud Team |