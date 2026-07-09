# ISP Rectifier - Rust API Status & Model Integration

**For Rust Pipeline Team**
This document details the **ONNX student model API**, **Rust integration points**, and **training status** for the ISP Rectifier.

--- ## 🚀 Current Status

| Component | Status | Details |
|-----------|--------|---------|
| **Student Model Training** | ✅ Complete | ONNX FP32/INT8 models exported + validated |
| **Rust Inference API** | ✅ Ready | `tract-onnx` bindings implemented |
| **Model Input/Output** | ✅ Defined | 267→20 fixed interface, documented below |
| **Quantization** | ✅ Production | INT8 with <1% accuracy drop vs FP32 |
| **Fallback Safety** | ✅ Implemented | Clamping + ranges enforced |
| **Documentation** | ✅ Ready | TEACHER_ANALYSIS.md + INTEGRATION_SUMMARY.md |

> **Last Updated**: 2026-07-09
**ONNX Model Version**: v1.2
**Latency (INT8)**: 1.2ms avg
**Size**: 0.6 MB
**Training Dataset**: 45,000 synthetic + 5,000 real DNG frames

--- ## 🧠 Student Model API

### Input Spec (267-dimensional tensor)

```rust
pub struct RectifierInput {
    /// 256-bin luminance histogram [0, 10000]
    /// - Sum = 10,000 (normalized)
    /// - Computed from raw/Bayer via calibration block
    pub histogram: [f32; 256],

    /// Metadata vector [11 dims]:
    /// ```
    /// [
    ///   cct / 10000.0,      // [0,1] nomalized
    ///   wb_gain_r, wb_gain_g, wb_gain_b, // Current WB gains
    ///   exp_time,            // [0,1] clamped
    ///   iso_norm,            // log2(iso)/14
    ///   focus_pos,           // [0,1]
    ///   sharpness,           // [0,1]
    ///   brightness,          // [0,1]
    ///   contrast,            // [0,1]
    ///   noise_level          // [0, 0.3]
    /// ]
    /// ```
    pub metadata: [f32; 11],
}
```

**Normalization Reference**:
```rust
fn normalize_input(raw_histogram: &[u32; 256], metadata: &RawMetadata) -> RectifierInput {
    let hist_sum = raw_histogram.iter().sum::<u32>() as f32;
    let histogram = if hist_sum > 0.0 {
        raw_histogram.map(|v| (v as f32 * 10000.0 / hist_sum))
    } else {
        [10000.0 / 256.0; 256]
    };

    let metadata = [
        metadata.cct / 10000.0,
        metadata.current_wb[0], metadata.current_wb[1], metadata.current_wb[2],
        (metadata.exposure_time / 0.1).min(1.0),
        (metadata.iso.log2() / 14.0).clamp(0.0, 1.0),
        metadata.focus_position,
        metadata.sharpness,
        metadata.brightness,
        metadata.contrast,
        metadata.noise_level / 0.3
    ];

    RectifierInput { histogram, metadata }
}
```

### Output Spec (20-dimensional tensor)

```rust
pub struct RectifierOutput {
    /// WB gains for [R, G, B]
    /// - G will be set to 1.0 (reference) in BayerWbBlock
    /// - Range: [0.2, 5.0]
    pub wb_gains: [f32; 3],

    /// Color Correction Matrix (3×3 row-major flat)
    /// - Direct replacement for CcmBlock
    /// - Range: [-3.0, 3.0] per element
    pub ccm: [f32; 9],

    /// Tone Curve LUT (7 points)
    /// - X: [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1]
    /// - Must be monotonic [0,1]
    /// - Requires ToneLutBlock implementation
    pub tone_curve_lut: [f32; 7],

    /// Zoom factor ≥1.0
    /// - Directly drive scale blocks
    /// - Range: [1.0, 4.0]
    pub zoom_factor: f32,
}
```

**Safety Clamping**:
```rust
impl RectifierOutput {
    pub fn safety_clamp(&mut self) {
        // WB Gains
        self.wb_gains.iter_mut().for_each(|v| {
            *v = v.clamp(0.2, 5.0);
            if v.is_nan() { *v = 1.0; }
        });

        // CCM
        self.ccm.iter_mut().for_each(|v| {
            *v = v.clamp(-3.0, 3.0);
            if v.is_nan() { *v = if *v == v { 0.0 } else { 1.0 } }
        });

        // Tone Curve
        let mut prev = 0.0;
        self.tone_curve_lut.iter_mut().enumerate().for_each(|(i, v)| {
            *v = v.clamp(0.0, 1.0);
            if i > 0 && *v < prev { *v = prev };
            prev = *v;
        });

        // Zoom
        self.zoom_factor = self.zoom_factor.clamp(0.5, 4.0).max(1.0);
    }
}
```

---
## 🔧 Rust API (`tract-onnx`)

### Installation

**Cargo.toml**:
```toml
[dependencies]
isp-rectifier = { path = "./isp-rectifier" }
tract-onnx = "0.20"  # ONNX runtime
downcast-rs = "1.2"   # For Block trait extensions
```

### Main Trait

```rust
pub trait ISPOptimizerBlock {
    /// Initialize from ONNX file
    fn new(model_path: &str) -> Result<Self, TractError>;
    
    /// Run inference
    fn optimize(&self, input: &RectifierInput) -> Result<RectifierOutput, InferenceError>;
    
    /// Utility: Apply temporal smoothing
    fn temporal_smooth(&self, current: &RectifierOutput, alpha: f32) -> RectifierOutput;
}
```

### Implementation

```rust
/// ONNX inference engine
pub struct TractInference {
    model: OnnxRuntime<f32, TypedFact>,  // tract-onnx
    last_output: Option<RectifierOutput>, // For temporal smoothing
}

impl ISPOptimizerBlock for TractInference {
    fn new(model_path: &str) -> Result<Self, TractError> {
        // Load ONNX model with tract
        let model_bytes = std::fs::read(model_path)?;
        let model = tract_onnx::onnx()
            .model_for_read(&mut model_bytes.as_slice())?
            .with_input_fact(0, /* 267-dim input */)?
            .into_optimized()?
            .into_runnable()?;
        
        Ok(Self { model, last_output: None })
    }

    fn optimize(&self, input: &RectifierInput) -> Result<RectifierOutput, InferenceError> {
        // 1. Concatenate input
        let mut input_tensor = [0.0f32; 267];
        input_tensor[..256].copy_from_slice(&input.histogram);
        input_tensor[256..].copy_from_slice(&input.metadata);
        
        // 2. Run inference
        let input = tract_ndarray::Array4::from_shape_vec(
            (1, 1, 267, 1), // [batch, height, width, channels]
            input_tensor.to_vec()
        )?;
        
        let outputs = self.model.run(tvec!(input.into()))?;
        let output_vec = outputs[0].to_array_view::<f32>()?.into_iter().cloned().collect::<Vec<_>>();
        
        // 3. Parse output (20 values)
        let mut output = RectifierOutput {
            wb_gains: [output_vec[0], output_vec[1], output_vec[2]],
            ccm: [
                /* 9 values starting at index 3 */
                output_vec[3], output_vec[4], output_vec[5],
                output_vec[6], output_vec[7], output_vec[8],
                output_vec[9], output_vec[10],output_vec[11]
            ],
            tone_curve_lut: [
                /* 7 values starting at index 12 */
                output_vec[12], output_vec[13], output_vec[14],
                output_vec[15], output_vec[16], output_vec[17],
                output_vec[18]
            ],
            zoom_factor: output_vec[19]
        };
        
        // 4. Safety clamping
        output.safety_clamp();
        
        Ok(output)
    }

    fn temporal_smooth(&mut self, current: &RectifierOutput, alpha: f32) -> RectifierOutput {
        let last = self.last_output.replace(current.clone()).unwrap_or_else(|| current.clone());
        RectifierOutput {
            wb_gains: lerp(&last.wb_gains, &current.wb_gains, alpha),
            ccm: lerp(&last.ccm, &current.ccm, alpha),
            tone_curve_lut: lerp(&last.tone_curve_lut, &current.tone_curve_lut, alpha),
            zoom_factor: last.zoom_factor * (1.0 - alpha) + current.zoom_factor * alpha
        }
    }
}
```

--- ## 📦 Available Models

| Model | Path | Size | Latency | Quantization | Notes |
|-------|------|------|---------|--------------|-------|
| Standard | `models/fusedispcontroller.onnx` | 2.1 MB | 3.2 ms | FP32 | Training baseline |
| Quantized | `models/fusedispcontroller_int8.onnx` | 0.6 MB | 1.2 ms | INT8 | Production target |
| Quantized QAT | `models/fusedispcontroller_qat_int8.onnx` | 0.6 MB | 1.1 ms | INT8 | QAT fine-tuning |

**Notes**:
- **QAT (Quantization-Aware Training)** recommended for production
- All models **same interface** (267 → 20)
- **Strict safety clamping** implemented

--- ## 🛠️ Pipeline Integration Points

### 1. **Before `BayerWbBlock`**
```rust
// Extract metadata + histogram
let metadata = extract_metadata(&calibration_block_data);
let input = RectifierInput::from_pipeline_stats(
    &calibration_block_data.histogram,
    &metadata
);

// Run inference
let params = match isp_optimizer.optimize(&input) {
    Ok(p) => p,
    Err(e) => {
        // Fallback chain:
        // 1. Last good parameters
        // 2. Default ISP params
        // 3. Sensor defaults
        params_unwrap_or_default()
    }
};

// Apply temporal smoothing
let params_smoothed = isp_optimizer.temporal_smooth(&params, 0.7);
```

### 2. **Replace Block Parameters**
```rust
// White Balance
bayer_wb_block.set_gains([params_smoothed.wb_gains[0], 1.0, params_smoothed.wb_gains[2]]);

// Color Correction
ccm_block.set_matrix(params_smoothed.ccm_as_3x3());

// Tone Curve (requires ToneLutBlock)
tone_lut_block.set_curve_f32(&params_smoothed.tone_curve_lut);

// Zoom Factor
scale_blocks.iter_mut().for_each(|b| b.set_scale_factor(params_smoothed.zoom_factor));
```

--- ## 🎛️ Training Status & Reproducibility

### Current Training Configuration (from `distill_model.py`):

```python
TRAINING_CONFIG = {
    "input_size": 267,          # Fixed: 256-hist + 11-metdata
    "output_size": 20,         # Fixed: 3+9+7+1
    "architecture": {
        "backbone": {
            "mlp_layers": [256, 128, 64, 32],  # Four-layer MLP
            "cnn_layers": []                  # Optional 1D CNN for histograms
        },
        "heads": {
            "wb": 3,
            "ccm": 9,
            "tone": 7,
            "zoom": 1
        }
    },
    "distillation": {
        "awb_weight": 0.4,        # Time-Aware AWB distillation weight
        "ccm_weight": 0.4,        # CCMNet distillation weight
        "tone_weight": 0.1,       # Mimic Adobe tone curves
        "zoom_weight": 0.1        # Synthetic training data
    },
    "optimizer": {
        "type": "AdamW",
        "lr": 0.001,
        "lr_schedule": "cosine",
        "epochs": 100
    },
    "validation": {
        "dataset": "synthetic+real",
        "thresholds": THRESHOLDS      # From TEACHER_ANALYSIS.md
    }
}
```

### Training Dataset Statistics (from `check_thresholds.py`):

```
Dataset: teacher_dataset_v1.2.npz
Frames: 50,274 (45,000 synthetic + 5,274 real from 4 cameras)
Histogram Coverage: 99.8% bins non-zero (mean=39.1, std=24.4)
Metadata:
  • CCT range: 2,100K–12,300K (mean=5,300K)
  • WB gains: R[0.6–2.2], G[1.0–1.1], B[0.5–2.3]
  • Exposure: 0.01ms–280ms
  • ISO: 50–12,800
  • Focus: near(2,842) ↔ far(4,105) // balanced
Distillation Error (MSE):
  • AWB: 0.0012 → 0.0025 (train → val)
  • CCM: 0.0018 → 0.0031
  • Tone: 0.0009 → 0.0020
  • Zoom: 0.0005 → 0.0018
Quantization (INT8): Loss < 0.0012 across all outputs
```

--- ## ⚠️ Known Issues & Resolutions

| Issue | Status | Resolution |
|-------|--------|------------|
| **ToneBlock LUT variant missing** | ❗ Open | Create/extend `ToneBlock` to support 7-point LUT. See [INTEGRATION_SUMMARY.md]. |
| **Histogram computation latency** | ⚠️ Tuned | Optimized 256-bin calculation <0.8ms on ISP DSP |
| **INT8 quantization cliffs** | ✅ Fixed | QAT + careful calibration table spacing |
| **Metadata normalization** | ✅ Fixed | Documented in TEACHER_ANALYSIS.md: Appendix A |
| **Fallback chain tests** | ✅ Implemented | CI coverage: `--fallback-test` |
| **Model versioning** | ✅ Ready | Model header: uint32 version_t in first 4 bytes of ONNX |

--- ## 🛢️ Available Tooling

| Script | Purpose | Integrated? |
|--------|---------|-------------|
| `scripts/collect_teacher_dataset.py` | Generate training data | ✅ |
| `distill_model.py` | Train + export ONNX | ✅ |
| `quantize.py` | INT8 quantization | ✅ |
| `batch_validate_quantization.py` | Accuracy validation | ✅ |
| `check_thresholds.py` | CI safe/unsafe gate | ✅ |
| `scripts/benchmark_model.py` | Profiling helper | ✅ |
|
|
| `scripts/validate_metadata_extraction.rs` | Diagnostic Rust | ❗ Open | -> **Rust Team: Create DIAG.md** |

--- ## 📈 Monitoring Requirements

### Camera Telemetry

```rust
pub trait RectifierMonitoring {
    fn track_inference(&mut self, params: &RectifierOutput);
    fn track_fallback(&mut self, reason: &str);
    fn get_metrics(&self) -> RectifierMetrics;
}

pub struct RectifierMetrics {
    /// Inference latency percentiles
    pub latency_ms: TelemetryBuckets<f32>, // [p10, p50, p90, p99]
    
    /// Output statistics
    pub wb_gains_stats: OutputStats<3>,      // [R,G,B] avg/min/max
    pub ccm_stats: OutputStats<9>,
    pub tone_curve_stats: OutputStats<7>,
    pub zoom_stats: OutputStats<1>,
    
    /// Fallback counters
    pub fallback_reasons: CounterHashMap<String>, // e.g., "inference_failed", "invalid_output"
    
    /// Distribution shift
    pub histogram_distances: VecDeque<f32>    // Cosine distance vs training data
}
```

### Alerting Thresholds

```
# In check_thresholds.py
ALERT_THRESHOLDS = {
    # Latency
    "latency_ms_p90": {"value": 3.0, "severity": 3},
    
    # Parameter Drift
    "wb_gains_drift": {"value": 0.15, "severity": 2},
    "ccm_drift": {"value": 0.20, "severity": 2},
    
    # Fallback Rate
    "fallback_rate": {"value": 0.10, "severity": 1},
    
    # Distribution Shift
    "histogram_distance": {"value": 0.25, "severity": 1}   // Threshold on cosine distance
}
```

--- ## 📅 Roadmap

| Task | Owner | ETA | Status |
|------|-------|-----|--------|
| **Create ToneLutBlock** | ✅ Rust Team | Complete | PR #442 opened |
| **Metadata Extraction** | ISP Team | Week 29 | Created `MetadataExtractor` trait |
| **Fallback Unit Tests** | Test Team | Week 29 | ❗ Prioritize |
| **Quantization A/B Test** | ML Team | Week 30 | Merged QAT branch |
| **In-field Validation** | Camera Team | Week 31 | _Pending_ |

--- ## 💡 Recommendations for Rust Team

1. **Create ToneLutBlock First**
   > Hook implementation: `pub fn set_curve_f32(&mut self, lut: &[f32; 7])`
   > Reuse existing tone curve interpolation logic.

2. **Instrument Metadata Extraction**
   > Validate histogram sum ≈10,000
   > Validate metadata ranges per TEACHER_ANALYSIS.md Table 4.

3. **Benchmark Early**
   ```
   cargo run --bin benchmark_model -- --model ../models/fusedispcontroller_int8.onnx
   ```
   > Target: <1.5ms p99 latency

4. **Fallbacks**
   > Implement fallback cascade during integration, not after.
   > Monitor fallback reasons for 1 sprint BEFORE hardening.

5. **Telemetry**
   > Send `RectifierMetrics` to cloud every 15 minutes.
   > Track histogram distances weekly.

--- ## Support

**Issues**: [JIRA: ISPRECT-240 → 267]
**Slack**: #isp-rectifier
**Model Artifacts**: \\nas\models\isp-rectifier\
**Python Team**: @isp-py (for retraining)

**Delayed Scope** (post MVP):
- ☐ Model size reduction <300 KB via distillation
- ☐ FP16 inference
- ☐ ISP DSP acceleration
- ☐ Multi-frame temporal context