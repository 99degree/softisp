#!/usr/bin/env python3
"""
Unified Teacher Output Collection Script for ISP Distillation

This script loads available teacher models (AWB, CCM, Tone, Zoom, etc.) from their
respective repositories, runs inference on metadata samples, and saves all teacher
outputs into a unified NPZ dataset for student distillation.

Currently supported teacher models (based on available repos):
- Time-Aware AWB (SamsungLabs/time-aware-awb) ✅
- CCMNet (SamsungLabs/CCMNet) ✅
- Modular Neural ISP (tone/zoom) - if available
- Additional: HDRNet, SID, DnCNN - if repos found

Usage:
    python collect_all_teacher_outputs.py \
        --metadata-path metadata.json \
        --output-path teacher_dataset/all_teacher_outputs.npz \
        --time-aware-awb-path time-aware-awb \
        --ccmnet-path ccmnet \
        [--modular-isp-path modularneuralisp] \
        [--hdrnet-path hdrnet] \
        [--sid-path sid] \
        [--config teacher_config.json]
"""

import argparse
import json
import sys
import importlib.util
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional, Callable
import numpy as np
import torch
from tqdm import tqdm


class TeacherRegistry:
    """Registry for all teacher models with their loading and inference logic."""
    
    def __init__(self, device: str = "cuda"):
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.teachers = {}
        self.inference_fns = {}
        self.output_shapes = {}
        self.output_keys = {}
    
    def register_teacher(
        self,
        name: str,
        loader_fn: Callable[[], torch.nn.Module],
        inference_fn: Callable[[torch.nn.Module, torch.Tensor, torch.Tensor], torch.Tensor],
        output_key: str,
        output_shape: Tuple[int, ...]
    ):
        """Register a teacher model with its loading and inference functions."""
        self.teachers[name] = loader_fn
        self.inference_fns[name] = inference_fn
        self.output_keys[name] = output_key
        self.output_shapes[name] = output_shape
        print(f"Registered teacher: {name} -> {output_key} (shape: {output_shape})")
    
    def load_all(self) -> Dict[str, torch.nn.Module]:
        """Load all registered teachers."""
        loaded = {}
        for name, loader_fn in self.teachers.items():
            print(f"Loading {name}...")
            try:
                model = loader_fn()
                model.to(self.device)
                model.eval()
                loaded[name] = model
                print(f"  ✅ {name} loaded successfully")
            except Exception as e:
                print(f"  ❌ Failed to load {name}: {e}")
                raise
        return loaded
    
    def run_inference(
        self,
        models: Dict[str, torch.nn.Module],
        hist_tensor: torch.Tensor,
        meta_tensor: torch.Tensor
    ) -> Dict[str, np.ndarray]:
        """Run inference on all loaded models."""
        outputs = {}
        with torch.no_grad():
            for name, model in models.items():
                if name in self.inference_fns:
                    out = self.inference_fns[name](model, hist_tensor, meta_tensor)
                    outputs[name] = out.cpu().numpy()
        return outputs


def build_feature_vector(metadata: Dict) -> Tuple[np.ndarray, np.ndarray]:
    """Build model input from metadata dict."""
    # Histogram (256 bins)
    histogram = np.array(metadata.get("histogram", np.ones(256) * 100), dtype=np.float32)
    if len(histogram) != 256:
        raise ValueError(f"Histogram must have 256 bins, got {len(histogram)}")
    if histogram.sum() > 0:
        histogram = histogram / histogram.sum() * 10000
    
    # Metadata features (11 dims)
    meta = np.array([
        metadata.get("cct", 6500.0) / 10000.0,
        *metadata.get("wb_gains", [1.0, 1.0, 1.0]),
        metadata.get("exposure_time", 0.033),
        metadata.get("iso_gain", 1.0),
        metadata.get("focus_position", 0.5),
        metadata.get("sharpness", 0.5),
        metadata.get("brightness", 0.5),
        metadata.get("contrast", 0.5),
        metadata.get("noise_level", 0.1),
    ], dtype=np.float32)
    
    return histogram, meta


def load_metadata(metadata_path: Path) -> List[Dict]:
    """Load metadata from JSON or JSONL."""
    if metadata_path.suffix == ".jsonl":
        with open(metadata_path) as f:
            return [json.loads(line) for line in f]
    else:
        with open(metadata_path) as f:
            data = json.load(f)
            return data if isinstance(data, list) else [data]


# =============================================================================
# Teacher Model Loading Functions
# =============================================================================

def make_awb_loader(awb_repo_path: Path) -> Tuple[Callable, Callable]:
    """Create loader and inference functions for Time-Aware AWB."""
    sys.path.insert(0, str(awb_repo_path))
    
    from time_aware_awb.model import IllumEstimator
    from time_aware_awb.utils import rg_bg_to_illum_rgb
    
    # Find the best available model file
    models_dir = awb_repo_path / "models"
    model_files = list(models_dir.glob("model-awb*.pt"))
    if not model_files:
        raise FileNotFoundError(f"No AWB model files found in {models_dir}")
    
    # Prefer the basic model (without noise/SNR/pref suffixes)
    pref_order = ["model-awb.pt", "model-awb-pref.pt", "model-awb-w-noise.pt", 
                  "model-awb-w-noise-pref.pt", "model-awb-w-snr.pt", 
                  "model-awb-w-snr-pref.pt", "model-awb-w-noise-snr.pt", 
                  "model-awb-w-noise-snr-pref.pt"]
    
    model_file = None
    for pref in pref_order:
        matches = [f for f in model_files if f.name == pref]
        if matches:
            model_file = matches[0]
            break
    
    if model_file is None:
        model_file = model_files[0]
    
    print(f"Using AWB model: {model_file}")
    
    state_dict = torch.load(model_file, map_location="cpu")
    if 'state_dict' in state_dict:
        state_dict = state_dict['state_dict']
    if 'model' in state_dict:
        state_dict = state_dict['model']
    
    def load_fn():
        model = IllumEstimator(in_channels=15, hist_channels=1)  # 15 with time features
        model.load_state_dict(state_dict, strict=False)
        return model
    
    def inference_fn(model, hist_tensor, meta_tensor):
        # Time-Aware AWB expects 4-channel histogram [1, 4, 1, 256] for Conv2d
        hist_2d = hist_tensor.unsqueeze(1).unsqueeze(2)  # [1, 1, 1, 256] -> [1, 1, 1, 256]
        hist_2d = hist_2d.expand(-1, 4, 1, -1)  # [1, 4, 1, 256] - repeat for RGGB channels
        
        # Meta tensor should be [1, 15] with capture features + time features
        if meta_tensor.shape[1] < 15:
            # Pad with zeros for time features if not provided
            padding = torch.zeros(1, 15 - meta_tensor.shape[1], device=meta_tensor.device)
            meta_tensor = torch.cat([meta_tensor, padding], dim=1)
        
        illum_rgb = model(hist_2d, meta_tensor)
        illum = illum_rgb[0]
        wb_gains = illum / illum[1]  # Normalize to G=1
        return wb_gains.unsqueeze(0)  # [1, 3]
    
    return load_fn, inference_fn


def make_ccmnet_loader(ccmnet_repo_path: Path) -> Tuple[Callable, Callable]:
    """Create loader and inference functions for CCMNet."""
    sys.path.insert(0, str(ccmnet_repo_path / "src"))
    
    from ccmnet.ccmnet import CCMNet
    
    # Find checkpoint
    ckpt_candidates = list(ccmnet_repo_path.glob("*.pth")) + list(ccmnet_repo_path.glob("*.pt"))
    ckpt_candidates += list((ccmnet_repo_path / "src").glob("*.pth"))
    ckpt_candidates += list((ccmnet_repo_path / "src" / "ccmnet").glob("*.pth"))
    
    if not ckpt_candidates:
        # Check for checkpoints in common locations
        for subdir in ["checkpoints", "models", "weights", "pretrained"]:
            ckpt_candidates += list((ccmnet_repo_path / subdir).glob("*.pth"))
            ckpt_candidates += list((ccmnet_repo_path / subdir).glob("*.pt"))
    
    if not ckpt_candidates:
        raise FileNotFoundError(f"No CCMNet checkpoint found in {ccmnet_repo_path}")
    
    ckpt_path = ckpt_candidates[0]
    print(f"Using CCMNet checkpoint: {ckpt_path}")
    
    state_dict = torch.load(ckpt_path, map_location="cpu")
    if 'state_dict' in state_dict:
        state_dict = state_dict['state_dict']
    if 'model' in state_dict:
        state_dict = state_dict['model']
    if 'ccmnet' in state_dict:
        state_dict = state_dict['ccmnet']
    
    def load_fn():
        model = CCMNet()
        model.load_state_dict(state_dict, strict=False)
        return model
    
    def inference_fn(model, hist_tensor, meta_tensor):
        ccm = model(hist_tensor, meta_tensor)
        # Ensure output is [1, 9] or [1, 3, 3]
        if ccm.dim() == 2:
            return ccm  # [1, 9]
        elif ccm.dim() == 3:
            return ccm.view(1, -1)  # [1, 3, 3] -> [1, 9]
        return ccm
    
    return load_fn, inference_fn


def make_modular_isp_loader(modular_isp_path: Path, teacher_name: str) -> Tuple[Callable, Callable]:
    """Create loader for Modular Neural ISP teachers (tone/zoom)."""
    teacher_configs = {
        "tone": {
            "model_dir": "models/tone",
            "model_file": "model.py",
            "class_name": "ToneNet",
            "ckpt_pattern": "tone_pretrained.pth",
        },
        "zoom": {
            "model_dir": "models/zoom",
            "model_file": "model.py",
            "class_name": "ZoomNet",
            "ckpt_pattern": "zoom_pretrained.pth",
        },
    }
    
    if teacher_name not in teacher_configs:
        raise ValueError(f"Unknown modular ISP teacher: {teacher_name}")
    
    cfg = teacher_configs[teacher_name]
    model_dir = modular_isp_path / cfg["model_dir"]
    
    if not model_dir.exists():
        raise FileNotFoundError(f"Modular ISP {teacher_name} dir not found: {model_dir}")
    
    sys.path.insert(0, str(model_dir))
    module = importlib.import_module(cfg["model_file"].replace(".py", ""))
    ModelClass = getattr(module, cfg["class_name"])
    
    # Find checkpoint
    ckpt_path = model_dir / cfg["ckpt_pattern"]
    if not ckpt_path.exists():
        ckpt_candidates = list(model_dir.glob("*.pth")) + list(model_dir.glob("*.pt"))
        if ckpt_candidates:
            ckpt_path = ckpt_candidates[0]
        else:
            raise FileNotFoundError(f"No checkpoint found for {teacher_name} in {model_dir}")
    
    print(f"Using Modular ISP {teacher_name} checkpoint: {ckpt_path}")
    
    state_dict = torch.load(ckpt_path, map_location="cpu")
    if 'state_dict' in state_dict:
        state_dict = state_dict['state_dict']
    if 'model' in state_dict:
        state_dict = state_dict['model']
    
    def load_fn():
        model = ModelClass()
        model.load_state_dict(state_dict, strict=False)
        return model
    
    def inference_fn(model, hist_tensor, meta_tensor):
        out = model(hist_tensor, meta_tensor)
        return out
    
    return load_fn, inference_fn


def make_hdrnet_loader(hdrnet_path: Path) -> Tuple[Callable, Callable]:
    """Create loader for HDRNet (local tone mapping)."""
    # HDRNet typically predicts bilateral grid parameters or tone curve
    # This is a placeholder - implement based on actual HDRNet repo structure
    raise NotImplementedError("HDRNet loader not implemented - add when repo available")


def make_sid_loader(sid_path: Path) -> Tuple[Callable, Callable]:
    """Create loader for SID (See-in-the-Dark)."""
    # SID: end-to-end raw->RGB for low-light
    # Can provide denoising/exposure correction teacher outputs
    raise NotImplementedError("SID loader not implemented - add when repo available")


# =============================================================================
# Main Collection Script
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Collect teacher outputs from multiple ISP models for distillation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # With Time-Aware AWB and CCMNet
  python collect_all_teacher_outputs.py \\
    --metadata-path metadata.json \\
    --output-path teacher_dataset/all_outputs.npz \\
    --time-aware-awb-path time-aware-awb \\
    --ccmnet-path ccmnet

  # With all available teachers
  python collect_all_teacher_outputs.py \\
    --metadata-path metadata.json \\
    --output-path teacher_dataset/all_outputs.npz \\
    --time-aware-awb-path time-aware-awb \\
    --ccmnet-path ccmnet \\
    --modular-isp-path modularneuralisp

  # Using config file
  python collect_all_teacher_outputs.py \\
    --config teacher_config.json \\
    --metadata-path metadata.json \\
    --output-path teacher_dataset/all_outputs.npz
"""
    )
    
    # Input/Output
    parser.add_argument("--metadata-path", type=Path, required=True,
                        help="Path to metadata JSON/JSONL file")
    parser.add_argument("--output-path", type=Path, required=True,
                        help="Output NPZ file path")
    parser.add_argument("--config", type=Path,
                        help="JSON config file for teacher paths")
    
    # Teacher repo paths
    parser.add_argument("--time-aware-awb-path", type=Path,
                        help="Path to time-aware-awb repo")
    parser.add_argument("--ccmnet-path", type=Path,
                        help="Path to CCMNet repo")
    parser.add_argument("--modular-isp-path", type=Path,
                        help="Path to modularneuralisp repo (for tone/zoom)")
    parser.add_argument("--hdrnet-path", type=Path,
                        help="Path to HDRNet repo")
    parser.add_argument("--sid-path", type=Path,
                        help="Path to SID repo")
    
    # Processing options
    parser.add_argument("--device", type=str, default="cuda",
                        help="Device to use (cuda/cpu)")
    parser.add_argument("--max-samples", type=int, default=None,
                        help="Maximum samples to process")
    parser.add_argument("--batch-size", type=int, default=1,
                        help="Batch size for inference")
    
    args = parser.parse_args()
    
    # Load config file if provided
    if args.config:
        with open(args.config) as f:
            config = json.load(f)
            for key, value in config.items():
                if not getattr(args, key.replace('-', '_'), None):
                    setattr(args, key.replace('-', '_'), value)
    
    # Validate required paths
    if not args.metadata_path.exists():
        print(f"❌ Metadata file not found: {args.metadata_path}")
        sys.exit(1)
    
    if not args.time_aware_awb_path and not args.ccmnet_path and not args.modular_isp_path:
        print("❌ At least one teacher path must be provided")
        sys.exit(1)
    
    args.output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Load metadata
    print(f"📂 Loading metadata from {args.metadata_path}")
    metadata_list = load_metadata(args.metadata_path)
    if args.max_samples:
        metadata_list = metadata_list[:args.max_samples]
    print(f"   Loaded {len(metadata_list)} samples")
    
    # Initialize registry
    registry = TeacherRegistry(args.device)
    teacher_configs = {}
    
    # Register available teachers
    if args.time_aware_awb_path:
        if args.time_aware_awb_path.exists():
            teacher_configs["awb"] = args.time_aware_awb_path
            load_fn, inf_fn = make_awb_loader(args.time_aware_awb_path)
            registry.register_teacher("awb", load_fn, inf_fn, "wb_gains", (3,))
        else:
            print(f"⚠️  Time-Aware AWB path not found: {args.time_aware_awb_path}")
    
    if args.ccmnet_path:
        if args.ccmnet_path.exists():
            teacher_configs["ccm"] = args.ccmnet_path
            load_fn, inf_fn = make_ccmnet_loader(args.ccmnet_path)
            registry.register_teacher("ccm", load_fn, inf_fn, "ccm", (9,))
        else:
            print(f"⚠️  CCMNet path not found: {args.ccmnet_path}")
    
    if args.modular_isp_path:
        if args.modular_isp_path.exists():
            for teacher_name in ["tone", "zoom"]:
                try:
                    load_fn, inf_fn = make_modular_isp_loader(args.modular_isp_path, teacher_name)
                    registry.register_teacher(teacher_name, load_fn, inf_fn, 
                                            "tone_curve" if teacher_name == "tone" else "zoom_factor",
                                            (7,) if teacher_name == "tone" else (1,))
                    teacher_configs[teacher_name] = args.modular_isp_path
                except Exception as e:
                    print(f"⚠️  Could not register {teacher_name}: {e}")
        else:
            print(f"⚠️  Modular ISP path not found: {args.modular_isp_path}")
    
    if not registry.teachers:
        print("❌ No teachers registered!")
        sys.exit(1)
    
    print(f"\n🤖 Registered teachers: {list(registry.teachers.keys())}")
    
    # Load all teachers
    print(f"\n🤖 Loading teacher models...")
    teachers = registry.load_all()
    
    # Prepare output arrays
    num_samples = len(metadata_list)
    input_dim = 256 + 11
    
    inputs = np.zeros((num_samples, input_dim), dtype=np.float32)
    
    # Pre-allocate based on registered teachers
    output_arrays = {}
    for name in registry.teachers:
        key = registry.output_keys[name]
        shape = registry.output_shapes[name]
        output_arrays[key] = np.zeros((num_samples, *shape), dtype=np.float32)
    
    # Process samples
    print(f"\n🔮 Running inference on {num_samples} samples...")
    device = registry.device
    
    for i, metadata in enumerate(tqdm(metadata_list, desc="Processing")):
        histogram, meta = build_feature_vector(metadata)
        inputs[i] = np.concatenate([histogram, meta])
        
        hist_tensor = torch.from_numpy(histogram).unsqueeze(0).to(device)
        meta_tensor = torch.from_numpy(meta).unsqueeze(0).to(device)
        
        outputs = registry.run_inference(teachers, hist_tensor, meta_tensor)
        
        for name, out in outputs.items():
            key = registry.output_keys[name]
            output_arrays[key][i] = out.flatten()
    
    # Save dataset
    print(f"\n💾 Saving dataset to {args.output_path}")
    np.savez_compressed(
        args.output_path,
        inputs=inputs,
        histograms=inputs[:, :256],
        metadata=inputs[:, 256:],
        **output_arrays,
    )
    
    # Save dataset info
    info = {
        "num_samples": num_samples,
        "input_dim": input_dim,
        "histogram_bins": 256,
        "metadata_features": 11,
        "output_dims": {k: v.shape[1] for k, v in output_arrays.items()},
        "teachers_used": list(teacher_configs.keys()),
        "teacher_paths": {k: str(v) for k, v in teacher_configs.items()},
        "metadata_path": str(args.metadata_path),
    }
    
    info_path = args.output_path.with_suffix('.json')
    with open(info_path, 'w') as f:
        json.dump(info, f, indent=2)
    
    # Print summary
    print(f"\n✅ Dataset saved to {args.output_path}")
    print(f"   Samples: {num_samples}")
    print(f"   Input shape: {inputs.shape}")
    for key, arr in output_arrays.items():
        print(f"   {key}: {arr.shape}")
    
    # Print sample outputs
    print(f"\n📋 Sample outputs (first sample):")
    for key, arr in output_arrays.items():
        if key == "ccm":
            print(f"   {key}: {arr[0].reshape(3,3)}")
        else:
            print(f"   {key}: {arr[0]}")


if __name__ == "__main__":
    import sys
    main()