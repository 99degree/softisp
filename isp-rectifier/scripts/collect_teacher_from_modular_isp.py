#!/usr/bin/env python3
"""
Teacher Output Collection Script for Modular Neural ISP Distillation

This script loads pretrained .pth models from the Samsung Modular Neural ISP repo,
runs inference on metadata samples, and saves teacher outputs for student distillation.

Expected repo structure (from https://github.com/SamsungLabs/modularneuralisp):
modularneuralisp/
├── models/
│   ├── awb/
│   │   ├── model.py          # AWBNet class definition
│   │   └── awb_pretrained.pth
│   ├── ccm/
│   │   ├── model.py          # CCMNet class definition
│   │   └── ccm_pretrained.pth
│   ├── tone/
│   │   ├── model.py          # ToneNet class definition
│   │   └── tone_pretrained.pth
│   └── zoom/ (or in tone)
│       ├── model.py
│       └── zoom_pretrained.pth

Usage:
    python collect_teacher_outputs.py \
        --modular-isp-path /path/to/modularneuralisp \
        --metadata-path /path/to/metadata.json \
        --output-path teacher_dataset/teacher_outputs.npz
"""

import argparse
import json
import sys
import importlib.util
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional
import numpy as np
import torch
from tqdm import tqdm


class TeacherModelLoader:
    """Loads and manages teacher models from the Modular Neural ISP repo."""
    
    def __init__(self, repo_path: Path, device: str = "cuda"):
        self.repo_path = Path(repo_path)
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.models = {}
        
    def load_model(self, model_name: str, model_file: str, weights_file: str, class_name: str) -> torch.nn.Module:
        """Dynamically load a model from the modular neural ISP repo."""
        model_path = self.repo_path / model_file
        weights_path = self.repo_path / weights_file
        
        if not model_path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")
        if not weights_path.exists():
            raise FileNotFoundError(f"Weights file not found: {weights_path}")
        
        # Load module dynamically
        spec = importlib.util.spec_from_file_location(f"models.{model_name}", model_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        
        # Get model class and instantiate
        model_class = getattr(module, class_name)
        model = model_class()
        
        # Load weights
        state_dict = torch.load(weights_path, map_location=self.device)
        # Handle different state dict formats
        if 'state_dict' in state_dict:
            state_dict = state_dict['state_dict']
        if 'model' in state_dict:
            state_dict = state_dict['model']
        model.load_state_dict(state_dict, strict=False)
        
        model.to(self.device)
        model.eval()
        return model
    
    def load_all_teachers(self, config: Dict) -> Dict[str, torch.nn.Module]:
        """Load all teacher models based on config."""
        teachers = {}
        
        for name, cfg in config.items():
            print(f"Loading {name}...")
            try:
                model = self.load_model(
                    model_name=name,
                    model_file=cfg['model_file'],
                    weights_file=cfg['weights_file'],
                    class_name=cfg['class_name']
                )
                teachers[name] = model
                print(f"  ✅ {name} loaded successfully")
            except Exception as e:
                print(f"  ❌ Failed to load {name}: {e}")
                raise
        
        return teachers


def build_feature_vector(metadata: Dict) -> Tuple[np.ndarray, np.ndarray]:
    """
    Build model input from metadata.
    Returns: (histogram_256, metadata_11) matching the student model input format.
    """
    # Histogram (256 bins)
    histogram = np.array(metadata.get("histogram", np.ones(256) * 100), dtype=np.float32)
    if len(histogram) != 256:
        raise ValueError(f"Histogram must have 256 bins, got {len(histogram)}")
    
    # Normalize histogram
    if histogram.sum() > 0:
        histogram = histogram / histogram.sum() * 10000
    
    # Metadata features (11 dims)
    meta = np.array([
        metadata.get("cct", 6500.0) / 10000.0,           # Normalized CCT
        *metadata.get("wb_gains", [1.0, 1.0, 1.0]),       # WB gains [R, G, B]
        metadata.get("exposure_time", 0.033),             # Exposure time
        metadata.get("iso_gain", 1.0),                    # ISO/gain
        metadata.get("focus_position", 0.5),              # Focus position
        metadata.get("sharpness", 0.5),                   # Sharpness
        metadata.get("brightness", 0.5),                  # Brightness
        metadata.get("contrast", 0.5),                    # Contrast
        metadata.get("noise_level", 0.1),                 # Noise level
    ], dtype=np.float32)
    
    return histogram, meta


def load_metadata(metadata_path: Path) -> List[Dict]:
    """Load metadata from JSON or JSONL file."""
    if metadata_path.suffix == ".jsonl":
        with open(metadata_path) as f:
            return [json.loads(line) for line in f]
    else:
        with open(metadata_path) as f:
            data = json.load(f)
            return data if isinstance(data, list) else [data]


def run_teacher_inference(
    teachers: Dict[str, torch.nn.Module],
    histogram: np.ndarray,
    metadata: np.ndarray,
    device: torch.device
) -> Dict[str, np.ndarray]:
    """Run all teacher models on a single sample."""
    # Prepare inputs
    hist_tensor = torch.from_numpy(histogram).unsqueeze(0).to(device)  # [1, 256]
    meta_tensor = torch.from_numpy(metadata).unsqueeze(0).to(device)   # [1, 11]
    
    outputs = {}
    
    with torch.no_grad():
        # AWB: expects histogram + metadata, outputs WB gains [3]
        if 'awb' in teachers:
            awb_out = teachers['awb'](hist_tensor, meta_tensor)
            outputs['wb_gains'] = awb_out.cpu().numpy().flatten()
        
        # CCM: expects histogram + metadata, outputs flattened 3x3 matrix [9]
        if 'ccm' in teachers:
            ccm_out = teachers['ccm'](hist_tensor, meta_tensor)
            outputs['ccm'] = ccm_out.cpu().numpy().flatten()
        
        # Tone: expects histogram + metadata, outputs tone curve params [7]
        if 'tone' in teachers:
            tone_out = teachers['tone'](hist_tensor, meta_tensor)
            outputs['tone_curve'] = tone_out.cpu().numpy().flatten()
        
        # Zoom: expects histogram + metadata, outputs zoom factor [1]
        if 'zoom' in teachers:
            zoom_out = teachers['zoom'](hist_tensor, meta_tensor)
            outputs['zoom_factor'] = zoom_out.cpu().numpy().flatten()
    
    return outputs


def main():
    parser = argparse.ArgumentParser(
        description="Collect teacher outputs from Modular Neural ISP for distillation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example config for teacher models (adjust paths to match your repo):

--teacher-config '{
  "awb": {
    "model_file": "models/awb/model.py",
    "weights_file": "models/awb/awb_pretrained.pth",
    "class_name": "AWBNet"
  },
  "ccm": {
    "model_file": "models/ccm/model.py", 
    "weights_file": "models/ccm/ccm_pretrained.pth",
    "class_name": "CCMNet"
  },
  "tone": {
    "model_file": "models/tone/model.py",
    "weights_file": "models/tone/tone_pretrained.pth", 
    "class_name": "ToneNet"
  },
  "zoom": {
    "model_file": "models/zoom/model.py",
    "weights_file": "models/zoom/zoom_pretrained.pth",
    "class_name": "ZoomNet"
  }
}'
        """
    )
    
    parser.add_argument("--modular-isp-path", type=Path, required=True,
                        help="Path to cloned modularneuralisp repo")
    parser.add_argument("--metadata-path", type=Path, required=True,
                        help="Path to metadata JSON/JSONL file")
    parser.add_argument("--output-path", type=Path, required=True,
                        help="Output NPZ file path")
    parser.add_argument("--teacher-config", type=str, 
                        help="JSON config for teacher models (see epilog)")
    parser.add_argument("--teacher-config-file", type=Path,
                        help="Path to JSON file with teacher config")
    parser.add_argument("--device", type=str, default="cuda",
                        help="Device to use (cuda/cpu)")
    parser.add_argument("--batch-size", type=int, default=32,
                        help="Batch size for inference")
    parser.add_argument("--max-samples", type=int, default=None,
                        help="Maximum samples to process")
    
    args = parser.parse_args()
    
    # Load teacher config
    if args.teacher_config_file:
        with open(args.teacher_config_file) as f:
            teacher_config = json.load(f)
    elif args.teacher_config:
        teacher_config = json.loads(args.teacher_config)
    else:
        # Default config matching typical modularneuralisp structure
        teacher_config = {
            "awb": {
                "model_file": "models/awb/model.py",
                "weights_file": "models/awb/awb_pretrained.pth",
                "class_name": "AWBNet"
            },
            "ccm": {
                "model_file": "models/ccm/model.py",
                "weights_file": "models/ccm/ccm_pretrained.pth",
                "class_name": "CCMNet"
            },
            "tone": {
                "model_file": "models/tone/model.py",
                "weights_file": "models/tone/tone_pretrained.pth",
                "class_name": "ToneNet"
            },
            "zoom": {
                "model_file": "models/zoom/model.py",
                "weights_file": "models/zoom/zoom_pretrained.pth",
                "class_name": "ZoomNet"
            }
        }
    
    # Validate paths
    if not args.modular_isp_path.exists():
        print(f"❌ Modular ISP path not found: {args.modular_isp_path}")
        sys.exit(1)
    
    if not args.metadata_path.exists():
        print(f"❌ Metadata file not found: {args.metadata_path}")
        sys.exit(1)
    
    args.output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Load metadata
    print(f"📂 Loading metadata from {args.metadata_path}")
    metadata_list = load_metadata(args.metadata_path)
    if args.max_samples:
        metadata_list = metadata_list[:args.max_samples]
    print(f"   Loaded {len(metadata_list)} samples")
    
    # Load teacher models
    print(f"🤖 Loading teacher models from {args.modular_isp_path}")
    loader = TeacherModelLoader(args.modular_isp_path, args.device)
    teachers = loader.load_all_teachers(teacher_config)
    
    # Prepare output arrays
    num_samples = len(metadata_list)
    input_dim = 256 + 11  # histogram + metadata
    
    inputs = np.zeros((num_samples, input_dim), dtype=np.float32)
    wb_targets = np.zeros((num_samples, 3), dtype=np.float32)
    ccm_targets = np.zeros((num_samples, 9), dtype=np.float32)
    tone_targets = np.zeros((num_samples, 7), dtype=np.float32)
    zoom_targets = np.zeros((num_samples, 1), dtype=np.float32)
    
    # Process samples
    print(f"🔮 Running teacher inference on {num_samples} samples...")
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    
    for i, metadata in enumerate(tqdm(metadata_list, desc="Processing")):
        histogram, meta = build_feature_vector(metadata)
        inputs[i] = np.concatenate([histogram, meta])
        
        outputs = run_teacher_inference(teachers, histogram, meta, device)
        
        if 'wb_gains' in outputs:
            wb_targets[i] = outputs['wb_gains']
        if 'ccm' in outputs:
            ccm_targets[i] = outputs['ccm']
        if 'tone_curve' in outputs:
            tone_targets[i] = outputs['tone_curve']
        if 'zoom_factor' in outputs:
            zoom_targets[i] = outputs['zoom_factor']
    
    # Save dataset
    print(f"💾 Saving dataset to {args.output_path}")
    np.savez_compressed(
        args.output_path,
        inputs=inputs,
        histograms=inputs[:, :256],
        metadata=inputs[:, 256:],
        wb_targets=wb_targets,
        ccm_targets=ccm_targets,
        tone_targets=tone_targets,
        zoom_targets=zoom_targets,
    )
    
    # Save dataset info
    info = {
        "num_samples": num_samples,
        "input_dim": input_dim,
        "histogram_bins": 256,
        "metadata_features": 11,
        "output_dims": {
            "wb_gains": 3,
            "ccm": 9,
            "tone_curve": 7,
            "zoom_factor": 1,
        },
        "teachers_used": list(teachers.keys()),
        "modular_isp_path": str(args.modular_isp_path),
        "metadata_path": str(args.metadata_path),
    }
    
    info_path = args.output_path.with_suffix('.json')
    with open(info_path, 'w') as f:
        json.dump(info, f, indent=2)
    
    # Print sample outputs
    print(f"\n✅ Dataset saved to {args.output_path}")
    print(f"   Samples: {num_samples}")
    print(f"   Input shape: {inputs.shape}")
    print(f"   WB targets: {wb_targets.shape}")
    print(f"   CCM targets: {ccm_targets.shape}")
    print(f"   Tone targets: {tone_targets.shape}")
    print(f"   Zoom targets: {zoom_targets.shape}")
    print(f"\n📋 Sample outputs (first sample):")
    print(f"   WB gains: {wb_targets[0]}")
    print(f"   CCM: {ccm_targets[0].reshape(3,3)}")
    print(f"   Tone: {tone_targets[0]}")
    print(f"   Zoom: {zoom_targets[0][0]:.3f}")


if __name__ == "__main__":
    main()