# Teacher Dataset Collection

This script collects soft labels from the three teacher models (CCMNet, Time-Aware AWB, Neural ISP Tuning) to create a distillation dataset.

## Quick Start

```bash
# Generate synthetic dataset (for testing)
python collect_teacher_dataset.py --samples 10000 --output teacher_dataset

# Use real metadata from your pipeline
python collect_teacher_dataset.py --metadata path/to/metadata.json --output teacher_dataset
```

## Output Files

| File | Format | Description |
|------|--------|-------------|
| `teacher_dataset.json` | JSON | Human-readable dataset with all inputs and teacher outputs |
| `teacher_dataset.npz` | NPZ | Compressed binary format for efficient training |
| `schema.json` | JSON | Dataset schema documentation |

## Dataset Structure

### Input Features (267 dimensions)
```
histogram (256) + cct (1) + wb_gains (3) + exposure_time (1) + 
iso_gain (1) + focus_position (1) + sharpness (1) + 
brightness (1) + contrast (1) + noise_level (1) = 267
```

### Teacher Outputs
| Output | Model | Shape |
|--------|-------|-------|
| `wb_gains` | Time-Aware AWB | (3,) |
| `ccm` | CCMNet | (9,) |
| `tone_curve` | Neural ISP Tuning | (7,) |
| `zoom_factor` | Neural ISP Tuning | (1,) |

## Integration with Real Pipeline

### 1. Export Metadata from Rust Pipeline
Add this to your `cam-rust` pipeline to dump metadata:

```rust
use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize)]
struct FrameMetadata {
    histogram: Vec<f32>,
    cct: f32,
    wb_gains: [f32; 3],
    exposure_time: f32,
    iso_gain: f32,
    focus_position: f32,
    sharpness: f32,
    brightness: f32,
    contrast: f32,
    noise_level: f32,
    timestamp: u64,
}

fn dump_metadata(metadata: &FrameMetadata) {
    let json = serde_json::to_string(metadata).unwrap();
    std::fs::write("metadata.json", json).unwrap();
}
```

### 2. Collect Real Metadata
```bash
# Run your pipeline to generate metadata.json
./your_isp_pipeline --dump-metadata metadata.json

# Use collected metadata for dataset generation
python collect_teacher_dataset.py --metadata metadata.json --output teacher_dataset
```

## Using with Distillation Training

The generated `teacher_dataset.npz` is ready for training the student model:

```python
import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

# Load dataset
data = np.load("teacher_dataset/teacher_dataset.npz")
inputs = torch.from_numpy(data["inputs"])
wb_targets = torch.from_numpy(data["wb_targets"])
ccm_targets = torch.from_numpy(data["ccm_targets"])
tone_targets = torch.from_numpy(data["tone_targets"])
zoom_targets = torch.from_numpy(data["zoom_targets"])

# Create DataLoader
dataset = TensorDataset(inputs, wb_targets, ccm_targets, tone_targets, zoom_targets)
loader = DataLoader(dataset, batch_size=32, shuffle=True)

# Train student model (see distill_model.py)
```

## Next Steps

1. **Replace Placeholder Models**: Update `TeacherModels` class in `collect_teacher_dataset.py` with actual model loading/inference
2. **Collect Real Data**: Run your ISP pipeline to generate real metadata
3. **Train Student**: Use `distill_model.py` with the generated dataset
4. **Validate**: Use `batch_validate_quantization.py` to verify quantized model quality