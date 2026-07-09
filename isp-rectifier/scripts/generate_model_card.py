#!/usr/bin/env python3
"""
Generate MODEL_CARD.md for the distilled ISP controller model.
"""

import argparse
import json
import torch
from datetime import datetime
from pathlib import Path


def generate_model_card(
    model_path: Path,
    int8_path: Path,
    fp16_path: Path,
    checkpoint_path: Path,
    dataset_info_path: Path,
    output_path: Path,
):
    """Generate comprehensive model card."""
    
    # Load metadata
    dataset_info = json.loads(dataset_info_path.read_text())
    checkpoint = torch.load(checkpoint_path, map_location='cpu')
    
    # Model sizes
    fp32_size = model_path.stat().st_size / (1024 * 1024)
    int8_size = int8_path.stat().st_size / (1024 * 1024) if int8_path.exists() else 0
    fp16_size = fp16_path.stat().st_size / (1024 * 1024) if fp16_path.exists() else 0
    
    # Training info
    epoch = checkpoint.get('epoch', 'N/A')
    val_loss = checkpoint.get('val_loss', 'N/A')
    metadata_dim = checkpoint.get('metadata_dim', 'N/A')
    
    card = f"""# Fused ISP Controller Model Card

## Model Overview
- **Name**: `fusedispcontroller`
- **Version**: 1.0.0
- **Date**: {datetime.now().strftime('%Y-%m-%d')}
- **Type**: Distilled student model (multi-head regression)
- **Framework**: PyTorch → ONNX
- **License**: MIT

## Model Architecture
- **Input**: 267-dim feature vector (256-bin histogram + 11 metadata features)
- **Backbone**: 
  - Histogram: 1D CNN (3 conv layers, 2 max-pool, adaptive pool) → 2048-dim
  - Metadata: MLP (2 layers) → 256-dim
  - Fusion: Concat → 2304-dim → Fusion MLP (2 layers) → 256-dim
- **Heads**: 4 independent heads
  - WB Gains: Linear(256→64→3) 
  - CCM: Linear(256→128→9)
  - Tone Curve: Linear(256→64→7)
  - Zoom Factor: Linear(256→32→1)
- **Total Parameters**: ~1.2M
- **Model Size (FP32)**: {fp32_size:.2f} MB

## Training
- **Method**: Knowledge Distillation
- **Teacher Models**:
  - CCMNet (Color Correction Matrix)
  - Time-Aware AWB (White Balance)
  - Neural ISP Tuning (Tone Mapping + Zoom)
- **Loss**: Weighted MSE (WB: 1.0, CCM: 1.0, Tone: 0.5, Zoom: 0.5)
- **Optimizer**: AdamW (lr=1e-3, weight_decay=1e-4)
- **Scheduler**: Cosine Annealing
- **Epochs**: {epoch}
- **Best Val Loss**: {val_loss:.6f}
- **Training Samples**: {dataset_info.get('num_samples', 'N/A')}

## Input Features (267 dimensions)
| Index | Feature | Description | Normalization |
|-------|---------|-------------|---------------|
| 0-255 | histogram | 256-bin luminance histogram | Sum=10000 |
| 256 | cct | Correlated Color Temperature (K) | /10000 |
| 257-259 | wb_gains | Current WB gains [R, G, B] | raw |
| 260 | exposure_time | Exposure time (s) | raw |
| 261 | iso_gain | ISO/gain | raw |
| 262 | focus_position | Normalized focus position | [0, 1] |
| 263 | sharpness | Sharpness metric | [0, 1] |
| 264 | brightness | Mean brightness | [0, 1] |
| 265 | contrast | Contrast (std/mean) | [0, 1] |
| 266 | noise_level | Estimated noise | [0, 0.3] |

## Output Predictions
| Head | Shape | Description | Typical Range |
|------|-------|-------------|---------------|
| wbgains | (3,) | WB gains [R, G, B] | [0.5, 2.5] |
| ccm | (9,) | 3×3 CCM matrix (flattened) | [-1, 2] |
| tonecurve | (7,) | Tone mapping parameters | [0, 1] |
| zoom_factor | (1,) | Digital zoom factor | [1.0, 3.0] |

## Quantized Models
| Format | Size | Reduction | Use Case |
|--------|------|-----------|----------|
| FP32 | {fp32_size:.2f} MB | baseline | Reference |
| FP16 | {fp16_size:.2f} MB | {(1-fp16_size/fp32_size)*100:.1f}% | GPU inference |
| INT8 | {int8_size:.2f} MB | {(1-int8_size/fp32_size)*100:.1f}% | CPU/Embedded |

## Validation Results
Run `python batch_validate_quantization.py` for latest results.

### Thresholds (from check_thresholds.py)
| Head | Mean Diff | Max Diff |
|------|-----------|----------|
| WB (INT8) | < 0.01 | < 0.05 |
| CCM (INT8) | < 0.01 | < 0.05 |
| Tone (INT8) | < 0.02 | < 0.10 |
| Zoom (INT8) | < 0.01 | < 0.05 |

## Deployment

### Rust (tract-onnx)
```toml
[dependencies]
tract-onnx = "0.20"
```

```rust
let model = tract_onnx::onnx()
    .model_for_path("fusedispcontroller_int8.onnx")?
    .into_optimized()?
    .into_runnable()?;

let outputs = model.run(tvec!(hist_tensor, meta_tensor))?;
```

### Python (ONNX Runtime)
```python
import onnxruntime as ort
import numpy as np

session = ort.InferenceSession("fusedispcontroller_int8.onnx")
outputs = session.run(None, {{
    "histogram": histogram.astype(np.float32).reshape(1, -1),
    "metadata": metadata.astype(np.float32).reshape(1, -1),
}})
```

## Register Mapping
| ONNX Output | ISP Register | Format | Clamp Range |
|-------------|--------------|--------|-------------|
| wbgains[0] | WB_R_GAIN | Q4.12 | [0.1, 10.0] |
| wbgains[1] | WB_G_GAIN | Q4.12 | [0.1, 10.0] |
| wbgains[2] | WB_B_GAIN | Q4.12 | [0.1, 10.0] |
| ccm[0:9] | CCM_00...CCM_22 | Q4.12 | [-2.0, 2.0] |
| tonecurve[0:6] | TONE_LUT_0...6 | Q0.16 | [0.0, 1.0] |
| zoom_factor[0] | ZOOM_SCALE | Q4.12 | [1.0, 4.0] |

## Limitations
- Trained on synthetic/distilled data - validate on real hardware
- Assumes 256-bin histogram input format
- Metadata features must match training order exactly
- INT8 quantization may degrade on extreme exposures

## Future Work
- [ ] Train with real camera data
- [ ] Add uncertainty estimation head
- [ ] Support variable histogram bins
- [ ] TensorRT optimization for NVIDIA GPUs
- [ ] ONNX Runtime Web for browser deployment

## Contact
Generated by ISP Rectifier distillation pipeline.
See: https://github.com/your-org/softisp
"""

    output_path.write_text(card)
    print(f"✅ Model card written to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Generate MODEL_CARD.md")
    parser.add_argument("--model", required=True, help="FP32 ONNX model path")
    parser.add_argument("--int8", required=True, help="INT8 ONNX model path")
    parser.add_argument("--fp16", required=True, help="FP16 ONNX model path")
    parser.add_argument("--checkpoint", required=True, help="PyTorch checkpoint path")
    parser.add_argument("--dataset", required=True, help="Dataset info JSON path")
    parser.add_argument("--output", default="MODEL_CARD.md", help="Output path")
    
    args = parser.parse_args()
    
    generate_model_card(
        Path(args.model),
        Path(args.int8),
        Path(args.fp16),
        Path(args.checkpoint),
        Path(args.dataset),
        Path(args.output),
    )


if __name__ == "__main__":
    main()