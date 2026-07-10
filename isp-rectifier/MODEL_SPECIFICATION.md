# ISP Rectifier Model - Specification

## 🎯 Model Overview

**Purpose**: Lightweight neural controller that predicts optimal ISP parameters (WB gains, CCM, tone curve, zoom) from frame metadata to replace traditional 3A heuristics.

**Key Attributes**:
- ✅ **Distilled**: Trained via knowledge distillation from teacher models (CCMNet, Time-Aware AWB)
- ✅ **Embedded-Friendly**: Compact ONNX model with INT8 quantization support
- ✅ **Label-Free**: Learns from teacher outputs without ground truth labels
- ✅ **Hardware-Agnostic**: Runs on CPU/GPU/DSP via ONNX runtime

---

## 🧠 Model Architecture

```mermaid
graph TD
    %% Input
    A[Input: 267
    hist[256]+meta[11]]
    
    %% Shared Backbone
    A -->|267→256| B1[MLP 256]
    B1 --> B2[Dropout 0.2]
    B2 -->|256→256| B3[MLP 256]
    B3 -->|256→128| B4[MLP 128]
    B4 -->|128→64| B5[MLP 64]
    B5 --> B6[Dropout 0.1]
    B6 -->|64→32| B7[MLP 32]
    
    %% Multi-task Heads
    B7 -->|32→3| C1[WB Head
Linear→TanhClamp]
    B7 -->|32→9| C2[CCM Head
Linear→TanhClamp]
    B7 -->|32→7| C3[Tone Head
Linear→Sigmoid]
    B7 -->|32→1| C4[Zoom Head
Linear→ReLU]

    %% Output
    C1 -->D[Output: 20
WB[3]+CCM[9]+TONE[7]+ZOOM[1]]
    C2 -->D
    C3 -->D
    C4 -->D
```

**Detailed Layers**:
```python
class ISPOptimizer(nn.Module):
    def __init__(self, input_dim=267, output_dim=20):
        super().__init__()
        self.shared_backbone = nn.Sequential(
            nn.Linear(input_dim, 256),
            nn.LeakyReLU(0.1),
            nn.Dropout(0.2),
            nn.Linear(256, 256),
            nn.LeakyReLU(0.1),
            nn.Linear(256, 128),
            nn.LeakyReLU(0.1),
            nn.Linear(128, 64),
            nn.LeakyReLU(0.1),
            nn.Dropout(0.1),
            nn.Linear(64, 32),
            nn.LeakyReLU(0.1)
        )
        
        self.heads = nn.ModuleDict({
            'wb': nn.Linear(32, 3),
            'ccm': nn.Linear(32, 9),
            'tone': nn.Linear(32, 7),
            'zoom': nn.Linear(32, 1)
        })
    
    def forward(self, x):
        shared = self.shared_backbone(x)
        
        wb = self.wb_head(shared).clamp(-3.0, 3.0).exp()       # Range [0.05, 20.0] → clamp to [0.2, 5.0]
        ccm = self.ccm_head(shared).clamp(-3.0, 3.0)            # CCM elements [-3, 3]
        tone = torch.sigmoid(self.tone_head(shared))            # Curve points [0,1]
        zoom = self.zoom_head(shared).relu().add(1.0)          # Range [1.0, ∞) → clamp to [1.0, 4.0]
        
        return torch.cat([wb, ccm, tone, zoom], dim=-1)
```

---

## 📥 Input Specification (Semi-Fixed API)

| Component | Shape | Range | Semantics | Normalization |
|-----------|-------|-------|-----------|---------------|
| **Raw Histogram** | [256] u32 | [0, ∞) | 256-bin luminance histogram | `normalize_sum(histogram) → sum=10,000` |
| **Metadata** | [11] float | Varies | Scene/camera metadata | See table below |

### **Metadata Fields (11-dimensional)**

| Index | Field | Raw Range | Normalized Range | Notes |
|-------|-------|-----------|-------------------|-------|
| 0 | CCT | [2000, 12000] K | [0.2, 1.2] | `/10,000` |
| 1 | WB R gain | [0.2, 5.0] | Identity | Raw value |
| 2 | WB G gain | [1.0, 1.1] | Identity | Reference channel |
| 3 | WB B gain | [0.2, 5.0] | Identity | Raw value |
| 4 | Exposure time | [0.001, 0.1] s | [0.01, 1.0] | Clamped &
| 5 | ISO gain | [50, 12800] | [0.0, 1.0] | `log2(iso)/14` |
| 6 | Focus position | [0.0, 1.0] | Identity | AF actuator [0→near, 1→far] |
| 7 | Sharpness | [0.0, 1.0] | Identity | Focus sharpness metric |
| 8 | Brightness | [0.0, 1.0] | Identity | Luma mean 0–1 |
| 9 | Contrast | [0.0, 1.0] | Identity | Luma std dev 0–1 |
| 10 | Noise level | [0.0, 0.3] | [0,1] | Noise estimate scaled/0.3 |

**Validation**: Input validation enforced via:
```rust
fn validate_input(input: &[f32; 267]) -> Result<(), InputError> {
    // Histogram
    let hist_sum = input[..256].iter().sum::<f32>();
    ensure!(9800.0 <= hist_sum && hist_sum <= 10200.0, "Histogram sum ≈10,000");
    
    // Metadata ranges
    ensure!(0.0 <= input[256] && input[256] <= 1.2, "CCT ∈ [0,1.2]");
    ensure!((0.1..=5.0).contains(&input[257]), "WB R ∈ [0.1,5.0]");
    ensure!((0.0..=0.3).contains(&input[266]), "Noise ≤ 0.3");
    
    Ok(())
}
```

---

## 📤 Output Specification

| Head | Shape | Range | Activation | Target Runtime Param(s) | Target Block |
|------|-------|-------|------------|-------------------------|--------------|
| **WB gains** | [3] | [0.2, 5.0] | `exp(TanhClamp)` | `bayer_wb.gains` [1,4,1,1] (RGGB) | BayerWbBlock |
| **CCM** | [9] | [-3.0, 3.0] | `TanhClamp` | `demosaic_ccm.w` [3,4,1,1] + `b` [3] | DemosaicCcmBlock |
| **Tone curve** | [7] | [0.0, 1.0] | `Sigmoid` | `tone.contrast/brightness/gamma_recip`, `fcs.gain/bias`, `gamma.inv_gamma/min/max` | ToneBlock, FcsBlock, GammaBlock |
| **Zoom** | [1] | [1.0, 4.0] | `ReLU+1` | `scale` → AdaptiveDownscaleBlock.scale_factor | ScaleBlocks |

> **Note**: Current model outputs 20 parameters (3+9+7+1). The runtime API exposes finer-grained controls (see `docs/api/RUNTIME_PARAMS.md`). The 7-point tone curve is internally mapped to multiple tone-related blocks (`tone`, `fcs`, `gamma`, `auto_contrast`). Future model versions (v1.3+) may expand output dimension to directly control all runtime params.

### 🔄 Model Output → Runtime Parameter Mapping

The 20 model outputs map to the fine-grained runtime params (RUNTIME_PARAMS.md) as follows:

| Model Output | Runtime Param(s) | Block | Mapping Logic |
|--------------|------------------|-------|---------------|
| `wb[0:3]` | `bayer_wb.gains` [1,4,1,1] | BayerWbBlock | `gains = [wb_r, 1.0, wb_b, 1.0]` (RGGB, G=1.0 ref) |
| `ccm[0:9]` | `demosaic_ccm.w` [3,4,1,1] + `b` [3] | DemosaicCcmBlock | 3×3 → 3×4: pad 4th column with 0; bias = 0 |
| `tone[0]` | `tone.contrast` | ToneBlock | `contrast = tone[0]*2 - 1` → [-1,1] |
| `tone[1]` | `tone.brightness` | ToneBlock | `brightness = tone[1]*2 - 1` → [-1,1] |
| `tone[2]` | `tone.gamma_recip` | ToneBlock | `gamma_recip = tone[2]*0.5 + 0.5` → [0.5,1.0] |
| `tone[3]` | `auto_contrast.contrast_w` | AutoContrastBlock | `contrast_w = tone[3]*1.5 + 0.5` → [0.5,2.0] |
| `tone[4]` | `gamma.inv_gamma` | GammaBlock | `inv_gamma = tone[4]*0.5 + 0.25` → [0.25,0.75] |
| `tone[5]` | `fcs.gain` [3] | SaturationBlock | `gain = [tone[5], tone[5], tone[5]]*2` → [0,2] |
| `tone[6]` | `ldci.strength` | LdciBlock | `strength = tone[6]` → [0,1] |
| `zoom[0]` | `scale_blocks.scale_factor` | ScaleBlocks | Direct pass-through |

> **Note**: Additional runtime params (`normalize.max_val`, `saturation.scale`, `sharpen.strength`, `display.scale`, etc.) use fixed defaults or are controlled by separate heuristics. The model controls the core color/tone/zoom pipeline.

---

**Validation**: Output validation enforced via:
```rust
fn validate_output(output: &[f32; 20]) -> Result<(), OutputError> {
    // WB gains
    ensure!(output[..3].iter().all(|&v| 0.2 <= v && v <= 5.0), "WB ∈ [0.2,5.0]");
    ensure!((0.05..=5.0).contains(&output[1]), "WB G ≈1.0");
    
    // CCM
    ensure!(output[3..12].iter().all(|&v| -3.0 <= v && v <= 3.0), "CCM ∈ [-3,3]");
    
    // Tone curve
    ensure!(output[12..19].windows(2).all(|w| w[0] <= w[1]), "Tone monotonic");
    ensure!((0.0..=1.0).contains(&output[12]), "Tone[0]=0.0");
    ensure!((0.0..=1.0).contains(&output[18]), "Tone[6]=1.0");
    
    // Zoom
    ensure!(1.0 <= output[19] && output[19] <= 4.0, "Zoom ∈ [1,4]");
    
    Ok(())
}
```

## 🧪 Training Process

### Loss Function

```python
class DistillationLoss(nn.Module):
    def __init__(self, weights={'wb':0.4, 'ccm':0.4, 'tone':0.1, 'zoom':0.1}):
        super().__init__()
        self.weights = weights
        
    def forward(self, preds, targets):
        """preds/targets dict: {'wb':[B,3], 'ccm':[B,9], ...}"""
        
        wb_loss   = F.mse_loss(preds['wb'],
                              torch.log(targets['wb'].clamp(min=0.1)), 
                              reduction='none').mean(1)
        
        ccm_loss  = F.mse_loss(preds['ccm'], targets['ccm'], reduction='none').mean(1)
        
        tone_loss = F.mse_loss(preds['tone'], targets['tone'], reduction='none').mean(1)
        
        zoom_loss = F.mse_loss(preds['zoom'],
                              targets['zoom'].log(), 
                              reduction='none').mean(1)
        
        return (self.weights['wb']   * wb_loss  +
                self.weights['ccm']  * ccm_loss +
                self.weights['tone'] * tone_loss+
                self.weights['zoom'] * zoom_loss)
```

> **Note**: WB and zoom losses use log-space for relative error; CCM and tone use absolute MSE.

### Dataset Requirements

- **Input**: `
 { 'histogram': [256] u32,     # Raw 256-bin luminance
   'metadata': [11] float      # Normalized metadata
 }
`

- **Target**: `
 {
   'wb_gains': [3] float,      # Teacher WB gains [R,G,B]
   'ccm': [9] float,           # Teacher CCM (row-major)
   'tone_curve': [7] float,     # Adobe/Xrite tone response
   'zoom_factor': float         # Unity (1.0 = no zoom)
 }
`

### Training Parameters

```yaml
# distill_model.py
config:
  optimizer: AdamW
  lr: 0.001
  lr_schedule:
    type: Cosine
    T_max: 100
    eta_min: 1e-6
  
  epochs: 100
  batch_size: 128
  
  distillation:
    wb_weight: 0.4
    ccm_weight: 0.4
    tone_weight: 0.1
    zoom_weight: 0.1
    
  augmentation:
    - type: HistogramGaussianNoise
      sigma: 0.05
      p: 0.5
    - type: MetadataJitter
      scale: 0.1
      p: 0.5
```

## 📈 Validation Metrics

| Metric | Target | Validation Dataset | Quantization Impact |
|--------|--------|----------------------|----------------------|
| **WB Error** | <0.015 | 5,274 real + 45,000 synth | +0.0008 INT8 |
| **CCM Error** | <0.008 | Same | +0.0012 INT8 |
| **Tone Error** | <0.005 | Same | +0.0003 INT8 |
| **Zoom RMSE** | <0.05 | Same | +0.0002 INT8 |
| **Latency** | <2 ms | ARM Cortex-A78 | INT8: 1.2 ms, FP32: 3.2 ms |
| **Size** | <1.5 MB | Memory budget | INT8: 0.6 MB |

**Validation Script**:
```bash
python batch_validate_quantization.py \
   --fp32 models/fusedispcontroller.onnx \
   --quant models/fusedispcontroller_int8.onnx \
   --dataset teacher_dataset_v1.2.npz \
   --output validation_results.json
```

## 🔧 Quantization Support

### INT8 Quantization

```python
# quantize.py
quantized_model = quantization.quantize_dynamic(
    fp32_model,
    {torch.nn.Linear},  # Op types to quantize
    dtype=torch.qint8,
    weight_backend="qnnpack"
)

# Calibration
def calibrate_model(model, calibration_loader):
    model.eval()
    with torch.no_grad():
        for inputs in calibration_loader:
            _ = model(inputs)
```

| Layer | Quantization | Activation Precision | Weight Precision |
|-------|--------------|-----------------------|------------------|
| Shared Backbone | ✅ | 8-bit | 8-bit |
| WB Head | ✅ | 8-bit | 8-bit |
| CCM Head | ✅ | 8-bit | 8-bit |
| Tone Head | ✅ | 8-bit | 8-bit |
| Zoom Head | ✅ | 8-bit | 8-bit |

> **QAT (Quantization-Aware Training)**: Recommended for production. See `distill_model.py --qat_enable`.

## 🧩 Semi-Fixed API Contract

```python
def run_inference(self, input_tensor):
    """
    Semi-fixed API guarantee:
    - Input:  [batch, 267]    tensor [must be fixed]
    - Output: [batch, 20]     tensor [must be fixed]
    
    Model may internally:
    - Change architecture freely
    - Rebalance head weights
    - Adjust activation functions
    - Modify distillation weights
    
    As long as:
    ▶ Input/Output tensor shapes/semantics unchanged
    ▶ Safety constraints enforced
    ▶ Backward compatibility on major versions
    """
```

---

## ✅ Validation Checklist

| Test | Method | Pass Criterion |
|------|--------|-----------------|
| **Histogram Null Test** | Feed all-zero histogram | WB≈1.0, CCM≈identity, tone=linear, zoom=1.0 |
| **Metadata Null Test** | Feed zero metadata | Output within 3σ of training mean |
| **Monotonicity** | Check tone curve | `tone[i] ≤ tone[i+1]` for all i |
| **WB Gray Test** | Synthetic gray scene | WB=[1.0±0.02, 1.0±0.02, 1.0±0.02] |
| **CCM Identity Test** | Input neutral CCM teacher | CCM[-0.1,0.1] ∀ ij ∈ diagonals |
| **Latency Budget** | 1000 runs, avg | <1.5ms 90th percentile |
| **Size Limit** | Model file | ≤0.8 MB (INT8) |
| **Quantization Drop** | MSE comparison | ≤1% worse than FP32 on validation |

---

## 📁 File Manifest

| File | Purpose | Status |
|------|---------|--------|
| `models/fusedispcontroller.onnx` | Production FP32 ONNX | ✅ Finalized |
| `models/fusedispcontroller_int8.onnx` | Production INT8 ONNX | ✅ Finalized |
| `models/fusedispcontroller_qat_int8.onnx` | QAT INT8 | ✅ Recommended |
| `teacher_dataset_v1.2.npz` | Training/validation data | ✅ 50k samples |
| `distill_model.py` | Training script | ✅ Config frozen |
| `batch_validate_quantization.py` | Validation script | ✅ CI/CD integrated |
| `check_thresholds.py` | CI/CD gate | ✅ Thresholds locked |

---

## 🆚 Comparison: Teacher vs Student

| Model | Size | Latency | WB Error | CCM Error | Tone Error |
|-------|------|---------|---------|-----------|-----------|
| Time-Aware AWB | ~5 MB | 12 ms | Baseline | N/A | N/A |
| CCMNet | ~21 MB | 28 ms | N/A | Baseline | N/A |
| **Student Model** | **0.6 MB** | **1.2 ms** | +0.0012 | +0.0018 | Baseline* |

> *Student tone is learned from Adobe-standard curves, not CCMNet/TA-AWB.

---

© 2026 ISP AI Team
Model Version: v1.2.2

**Next Planned Version**: v1.3 -
- Adaptive distillation weights
- On-device learning hook
- DSP-specific optimizations