# ISP Rectifier Training Data Directory

## Directory Structure
```
data/
├── raw/                      # Raw DNG/JPG frames + metadata JSON
├── processed/                # Preprocessed NPZ datasets for training
├── teacher_dataset/          # Teacher model outputs (NPZ format)
│   ├── teacher_dataset.npz   # Main training dataset
│   ├── dataset_info.json     # Dataset metadata
│   └── schema.json           # Data schema
└── synthetic/                # Synthetic data for augmentation
```

## Dataset Schema (teacher_dataset.npz)

```python
{
    'inputs': float32[N, 267],           # histogram(256) + metadata(11) or 308 for v1.3
    'wb_targets': float32[N, 3],         # WB gains [R, G, B]
    'ccm_targets': float32[N, 9],        # CCM matrix flattened (3x3)
    'tone_targets': float32[N, 7],       # Tone curve parameters
    'zoom_targets': float32[N, 1],       # Zoom factor
    'metadata': {
        'num_samples': int,
        'histogram_bins': 256,
        'metadata_dim': 11,  # or 52 for v1.3
        'version': '1.3',
        'created': 'ISO8601 timestamp',
        'teacher_models': ['time_aware_awb', 'ccmnet', 'neural_isp_tuning']
    }
}
```

## Data Collection Commands

```bash
# Collect from DNG frames (production)
python collect_teacher_dataset_production.py \
    --input-dir /path/to/dng_frames \
    --metadata-dir /path/to/metadata_json \
    --output-dir ./data/teacher_dataset \
    --ccmnet-weights ./teachers/CCMNet/weights.pth \
    --awb-weights ./teachers/time-aware-awb/weights.pth \
    --isp-tuning-weights ./teachers/neural-isp-tuning/weights.pth

# Generate synthetic data (for testing)
python collect_teacher_dataset_production.py \
    --synthetic 10000 \
    --output-dir ./data/teacher_dataset

# Validate dataset
python -c "
import numpy as np
data = np.load('data/teacher_dataset/teacher_dataset.npz', allow_pickle=True)
print('Samples:', len(data['inputs']))
print('Input shape:', data['inputs'].shape)
print('WB targets:', data['wb_targets'].shape)
print('CCM targets:', data['ccm_targets'].shape)
"
```

## Data Format for Training (distill_model.py)

The `load_teacher_dataset` function expects:
- `inputs`: concatenated [histogram(256) + metadata(11 or 52)]
- `wb_targets`: [N, 3]
- `ccm_targets`: [N, 9] (flattened 3x3)
- `tone_targets`: [N, 7]
- `zoom_targets`: [N, 1]

## Data Versioning

| Version | Histogram | Metadata | Total Dim | Notes |
|---------|-----------|----------|-----------|-------|
| v1.2    | 256       | 11       | 267       | Original |
| v1.3    | 256       | 52       | 308       | +15 AWB + 26 CCM extras |

## Retraining Checklist

- [ ] Raw DNG frames collected from target sensors
- [ ] Metadata JSON sidecars generated (EXIF + 3A stats)
- [ ] Teacher models produce consistent outputs
- [ ] Dataset NPZ saved with version metadata
- [ ] Validation split created (10-20%)
- [ ] Distillation training completes with target loss
- [ ] Quantization validation passes thresholds
- [ ] ONNX export validates on target hardware
