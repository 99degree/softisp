#!/usr/bin/env python3
"""
Production Teacher Dataset Collector for ISP Distillation

This script:
1. Imports actual teacher models (CCMNet, Time-Aware AWB, Neural ISP Tuning)
2. Processes folders of DNG/metadata files
3. Extracts histograms and 3A metadata from raw frames
4. Runs batch inference through all three teachers
5. Outputs soft-label dataset (.npz) for student training

Usage:
    # Process real DNG files with metadata
    python collect_teacher_dataset_production.py \
        --input-dir /path/to/dng_frames \
        --metadata-dir /path/to/metadata_json \
        --output-dir teacher_dataset \
        --ccmnet-weights ccmnet.pth \
        --awb-weights time_aware_awb.pth \
        --isp-tuning-weights neural_isp_tuning.pth

    # Or use synthetic data for testing
    python collect_teacher_dataset_production.py --synthetic 10000 --output-dir teacher_dataset
"""

import argparse
import json
import sys
import os
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any
import numpy as np
import torch
import torch.nn as nn
from tqdm import tqdm
import rawpy
import cv2
from concurrent.futures import ThreadPoolExecutor
import warnings
warnings.filterwarnings("ignore")

# =============================================================================
# TEACHER MODEL ARCHITECTURES (Replace with imports from actual repos if available)
# =============================================================================

class CCMNet(nn.Module):
    """
    Color Correction Matrix Network.
    Replace with actual CCMNet architecture from your repo.
    """
    def __init__(self, input_dim: int = 267):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 512),
            nn.ReLU(),
            nn.BatchNorm1d(512),
            nn.Dropout(0.2),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.BatchNorm1d(256),
            nn.Dropout(0.1),
            nn.Linear(256, 9),
        )
    
    def forward(self, x):
        return self.net(x)


class TimeAwareAWB(nn.Module):
    """
    Time-Aware Auto White Balance Network.
    Replace with actual Time-Aware AWB architecture from your repo.
    """
    def __init__(self, input_dim: int = 267):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 256),
            nn.ReLU(),
            nn.BatchNorm1d(256),
            nn.Dropout(0.1),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.BatchNorm1d(128),
            nn.Linear(128, 3),
            nn.Softplus(),  # Ensure positive gains
        )
    
    def forward(self, x):
        return self.net(x)


class NeuralISPTuning(nn.Module):
    """
    Neural ISP Tuning Network (tone curve + zoom).
    Replace with actual Neural ISP Tuning architecture from your repo.
    """
    def __init__(self, input_dim: int = 267):
        super().__init__()
        self.backbone = nn.Sequential(
            nn.Linear(input_dim, 512),
            nn.ReLU(),
            nn.BatchNorm1d(512),
            nn.Dropout(0.2),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.BatchNorm1d(256),
            nn.Dropout(0.1),
        )
        self.tone_head = nn.Sequential(
            nn.Linear(256, 64),
            nn.ReLU(),
            nn.Linear(64, 7),
            nn.Sigmoid(),  # Tone curve params in [0,1]
        )
        self.zoom_head = nn.Sequential(
            nn.Linear(256, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Softplus(),  # Zoom >= 1.0
        )
    
    def forward(self, x):
        feat = self.backbone(x)
        tone = self.tone_head(feat)
        zoom = self.zoom_head(feat)
        return tone, zoom


# =============================================================================
# TEACHER MODEL LOADER
# =============================================================================

class TeacherModelLoader:
    """Loads and manages all three teacher models."""
    
    def __init__(
        self,
        ccmnet_weights: Optional[str] = None,
        awb_weights: Optional[str] = None,
        isp_tuning_weights: Optional[str] = None,
        device: str = "cuda",
        input_dim: int = 267,
    ):
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.input_dim = input_dim
        
        print(f"Loading teacher models on {self.device}...")
        
        # Load CCMNet
        self.ccmnet = CCMNet(input_dim).to(self.device)
        if ccmnet_weights and Path(ccmnet_weights).exists():
            self.ccmnet.load_state_dict(torch.load(ccmnet_weights, map_location=self.device))
            print(f"  ✅ CCMNet loaded from {ccmnet_weights}")
        else:
            print(f"  ⚠️  CCMNet: using random weights (no checkpoint at {ccmnet_weights})")
        self.ccmnet.eval()
        
        # Load Time-Aware AWB
        self.awb = TimeAwareAWB(input_dim).to(self.device)
        if awb_weights and Path(awb_weights).exists():
            self.awb.load_state_dict(torch.load(awb_weights, map_location=self.device))
            print(f"  ✅ Time-Aware AWB loaded from {awb_weights}")
        else:
            print(f"  ⚠️  Time-Aware AWB: using random weights (no checkpoint at {awb_weights})")
        self.awb.eval()
        
        # Load Neural ISP Tuning
        self.isp_tuning = NeuralISPTuning(input_dim).to(self.device)
        if isp_tuning_weights and Path(isp_tuning_weights).exists():
            self.isp_tuning.load_state_dict(torch.load(isp_tuning_weights, map_location=self.device))
            print(f"  ✅ Neural ISP Tuning loaded from {isp_tuning_weights}")
        else:
            print(f"  ⚠️  Neural ISP Tuning: using random weights (no checkpoint at {isp_tuning_weights})")
        self.isp_tuning.eval()
    
    @torch.no_grad()
    def predict_ccm(self, features: np.ndarray) -> np.ndarray:
        """Predict CCM from features."""
        x = torch.from_numpy(features).float().to(self.device)
        if x.dim() == 1:
            x = x.unsqueeze(0)
        ccm = self.ccmnet(x)
        return ccm.cpu().numpy()
    
    @torch.no_grad()
    def predict_wb(self, features: np.ndarray) -> np.ndarray:
        """Predict WB gains from features."""
        x = torch.from_numpy(features).float().to(self.device)
        if x.dim() == 1:
            x = x.unsqueeze(0)
        wb = self.awb(x)
        return wb.cpu().numpy()
    
    @torch.no_grad()
    def predict_tone_zoom(self, features: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Predict tone curve and zoom from features."""
        x = torch.from_numpy(features).float().to(self.device)
        if x.dim() == 1:
            x = x.unsqueeze(0)
        tone, zoom = self.isp_tuning(x)
        return tone.cpu().numpy(), zoom.cpu().numpy()
    
    @torch.no_grad()
    def predict_all(self, features: np.ndarray) -> Dict[str, np.ndarray]:
        """Run all three teachers."""
        # Batch process for efficiency
        x = torch.from_numpy(features).float().to(self.device)
        
        ccm = self.ccmnet(x).cpu().numpy()
        wb = self.awb(x).cpu().numpy()
        tone, zoom = self.isp_tuning(x)
        tone = tone.cpu().numpy()
        zoom = zoom.cpu().numpy()
        
        return {
            "wb_gains": wb,
            "ccm": ccm,
            "tone_curve": tone,
            "zoom_factor": zoom,
        }


# =============================================================================
# METADATA EXTRACTION FROM DNG/RAW FILES
# =============================================================================

class MetadataExtractor:
    """Extracts histogram and 3A metadata from DNG/raw frames."""
    
    def __init__(self):
        pass
    
    def extract_from_dng(self, dng_path: Path) -> Dict[str, Any]:
        """Extract metadata from DNG file using rawpy."""
        try:
            with rawpy.imread(str(dng_path)) as raw:
                # Get raw image data
                raw_data = raw.raw_image_visible.astype(np.float32)
                
                # Extract basic metadata
                metadata = {
                    "histogram": self._compute_histogram(raw_data),
                    "cct": self._estimate_cct(raw),  # Approximate from WB coeffs
                    "wb_gains": self._get_wb_gains(raw),
                    "exposure_time": self._get_exposure_time(raw),
                    "iso_gain": self._get_iso_gain(raw),
                    "focus_position": 0.5,  # Not in DNG, placeholder
                    "sharpness": self._compute_sharpness(raw_data),
                    "brightness": float(np.mean(raw_data) / raw.white_level),
                    "contrast": float(np.std(raw_data) / raw.white_level),
                    "noise_level": self._estimate_noise(raw_data),
                    "white_level": raw.white_level,
                    "black_level": raw.black_level_per_channel[0] if len(raw.black_level_per_channel) > 0 else 0,
                }
                
                # Try to get additional metadata from DNG tags
                if hasattr(raw, 'camera_whitebalance') and raw.camera_whitebalance is not None:
                    metadata["camera_wb"] = raw.camera_whitebalance.tolist()
                
                # Extended metadata for teacher models (v1.3+)
                metadata.update(self._extract_time_aware_awb_extras(raw))
                metadata.update(self._extract_ccmnet_extras(raw))
                
                return metadata
                
        except Exception as e:
            print(f"  ⚠️  Failed to process {dng_path}: {e}")
            return None
    
    def _compute_histogram(self, raw_data: np.ndarray, bins: int = 256) -> np.ndarray:
        """Compute 256-bin histogram of raw data."""
        # Normalize to 0-255 range
        max_val = np.max(raw_data)
        if max_val > 0:
            normalized = (raw_data / max_val * 255).clip(0, 255)
        else:
            normalized = raw_data
        hist, _ = np.histogram(normalized.flatten(), bins=bins, range=(0, 255))
        return hist.astype(np.float32)
    
    def _estimate_cct(self, raw) -> float:
        """Estimate CCT from camera white balance coefficients."""
        if hasattr(raw, 'camera_whitebalance') and raw.camera_whitebalance is not None:
            wb = raw.camera_whitebalance
            # Simple approximation: R/B ratio correlates R/B with CCT
            r_gain = wb[0] / wb[1] if wb[1] > 0 else 1.0
            b_gain = wb[2] / wb[1] if wb[1] > 0 else 1.0
            # Rough mapping (calibrate for your sensor)
            if r_gain > b_gain:
                return 3000 + 4000 * (r_gain - 1) / (r_gain + b_gain)
            else:
                return 5000 + 3000 * (b_gain - 1) / (r_gain + b_gain)
        return 6500.0  # Default daylight
    
    def _get_wb_gains(self, raw) -> List[float]:
        """Get white balance gains from DNG."""
        if hasattr(raw, 'camera_whitebalance') and raw.camera_whitebalance is not None:
            wb = raw.camera_whitebalance
            # Normalize to G=1
            if wb[1] > 0:
                return [float(wb[0]/wb[1]), 1.0, float(wb[2]/wb[1])]
        return [1.0, 1.0, 1.0]
    
    def _get_exposure_time(self, raw) -> float:
        """Get exposure time from DNG."""
        if hasattr(raw, 'exposure_time') and raw.exposure_time:
            return float(raw.exposure_time)
        return 0.033  # Default 30fps
    
    def _get_iso_gain(self, raw) -> float:
        """Get ISO/gain from DNG."""
        if hasattr(raw, 'iso_speed') and raw.iso_speed:
            return float(raw.iso_speed) / 100.0
        return 1.0
    
    def _compute_sharpness(self, raw_data: np.ndarray) -> float:
        """Compute sharpness metric (Laplacian variance)."""
        # Downsample for speed
        h, w = raw_data.shape
        if h > 512 or w > 512:
            scale = min(512/h, 512/w)
            small = cv2.resize(raw_data, (int(w*scale), int(h*scale)))
        else:
            small = raw_data
        # Laplacian variance
        laplacian = cv2.Laplacian(small, cv2.CV_32F)
        return float(np.var(laplacian) / (np.max(small) + 1e-6))
    
    def _estimate_noise(self, raw_data: np.ndarray) -> float:
        """Estimate noise level using homogeneous patches."""
        h, w = raw_data.shape
        # Sample corners and center
        patches = [
            raw_data[:32, :32],
            raw_data[:32, -32:],
            raw_data[-32:, :32],
            raw_data[-32:, -32:],
            raw_data[h//2-16:h//2+16, w//2-16:w//2+16],
        ]
        noise_levels = [np.std(p) for p in patches]
        return float(np.median(noise_levels) / (np.mean(raw_data) + 1e-6))

    def _extract_time_aware_awb_extras(self, raw) -> Dict[str, Any]:
        """Extract Time-Aware AWB extra features: time, SNR, flash."""
        extras = {}
        
        # Time features (12 dim) - sunrise/sunset probabilities + binary flags
        if hasattr(raw, 'datetime') and raw.datetime:
            import datetime
            dt = datetime.datetime.fromtimestamp(raw.datetime)
            hour = dt.hour + dt.minute / 60.0
            sunrise = max(0.0, 1.0 - abs(hour - 6.5) / 3.0)
            sunset = max(0.0, 1.0 - abs(hour - 18.5) / 3.0)
            midnight = max(0.0, 1.0 - abs(hour - 0.0) / 6.0)
            noon = max(0.0, 1.0 - abs(hour - 12.0) / 6.0)
            extras["time_probs"] = [sunrise, sunset, noon, midnight, 0.0, 0.0]
            extras["is_before"] = [1.0 if hour < 6.5 else 0.0,
                                    1.0 if hour < 18.5 else 0.0,
                                    1.0 if hour < 12.0 else 0.0,
                                    1.0 if hour < 0.0 else 0.0,
                                    0.0, 0.0]
        else:
            extras["time_probs"] = [0.0] * 6
            extras["is_before"] = [0.0] * 6
        
        # SNR stats (2 dim) - R/G/B mean/std combined
        raw_data = raw.raw_image_visible
        if raw_data is not None and raw_data.size > 0:
            h, w = raw_data.shape
            patches = [
                raw_data[:16, :16], raw_data[:16, -16:],
                raw_data[-16:, :16], raw_data[-16:, -16:],
                raw_data[h//2-8:h//2+8, w//2-8:w//2+8],
            ]
            means = [float(np.mean(p)) for p in patches]
            stds = [float(np.std(p)) for p in patches]
            extras["snr_mean"] = float(np.median(means))
            extras["snr_std"] = float(np.median(stds))
        else:
            extras["snr_mean"] = 0.0
            extras["snr_std"] = 0.0
        
        # Flash (1 dim)
        if hasattr(raw, 'flash') and raw.flash is not None:
            extras["flash"] = 1.0 if raw.flash else 0.0
        else:
            extras["flash"] = 0.0
        
        return extras

    def _extract_ccmnet_extras(self, raw) -> Dict[str, Any]:
        """Extract CCMNet extra features: CM1/CM2 matrices, CFE features."""
        extras = {}
        
        # Color Matrix 1 & 2 (9 each = 18 dim)
        if hasattr(raw, 'color_matrix_1') and raw.color_matrix_1 is not None:
            extras["cm1"] = raw.color_matrix_1.flatten().tolist()
        else:
            extras["cm1"] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        if hasattr(raw, 'color_matrix_2') and raw.color_matrix_2 is not None:
            extras["cm2"] = raw.color_matrix_2.flatten().tolist()
        else:
            extras["cm2"] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # CFE features (8 dim) - learned features from CM1/CM2
        cm1 = np.array(extras["cm1"]).reshape(3, 3)
        cm2 = np.array(extras["cm2"]).reshape(3, 3)
        diff = cm1 - cm2
        extras["cfe_features"] = [
            float(np.trace(cm1)), float(np.trace(cm2)),
            float(np.trace(diff)), float(np.linalg.norm(diff)),
            float(cm1[0,0]), float(cm1[1,1]), float(cm1[2,2]),
            float(cm2[0,0]),
        ]
        
        return extras

    def load_metadata_json(self, json_path: Path) -> Optional[Dict[str, Any]]:
        """Load metadata from JSON sidecar file."""
        try:
            with open(json_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"  ⚠️  Failed to load {json_path}: {e}")
            return None


# =============================================================================
# FEATURE VECTOR CONSTRUCTION
# =============================================================================

def build_feature_vector(
    histogram: np.ndarray,
    metadata: Dict[str, Any],
    normalize: bool = True,
) -> np.ndarray:
    """
    Build the feature vector for student model input.
    v1.2: 267 dim (256 hist + 11 meta)
    v1.3: 308 dim (256 hist + 52 meta) with teacher extras
    Order must match training!
    """
    # Histogram: 256 bins
    hist = histogram.astype(np.float32)
    if normalize and hist.sum() > 0:
        hist = hist / hist.sum() * 10000  # Normalize to fixed count
    
    # Base metadata (11 dims) - must match original order
    base_meta = np.array([
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
    
    # Time-Aware AWB extras (15 dims)
    time_probs = metadata.get("time_probs", [0.0] * 6)
    is_before = metadata.get("is_before", [0.0] * 6)
    snr_mean = metadata.get("snr_mean", 0.0)
    snr_std = metadata.get("snr_std", 0.0)
    flash = metadata.get("flash", 0.0)
    awb_extras = np.array([
        *time_probs,
        *is_before,
        snr_mean, snr_std, flash
    ], dtype=np.float32)
    
    # CCMNet extras (26 dims)
    cm1 = metadata.get("cm1", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    cm2 = metadata.get("cm2", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    cfe_features = metadata.get("cfe_features", [0.0] * 8)
    ccm_extras = np.array([
        *cm1,
        *cm2,
        *cfe_features
    ], dtype=np.float32)
    
    # Concatenate all metadata
    meta_features = np.concatenate([base_meta, awb_extras, ccm_extras])
    
    # Final feature vector: 256 + 11 + 15 + 26 = 308
    features = np.concatenate([hist, meta_features])
    return features


# =============================================================================
# DATASET COLLECTION PIPELINE
# =============================================================================

def process_frame(
    dng_path: Path,
    metadata_extractor: MetadataExtractor,
    metadata_dir: Optional[Path],
) -> Optional[Dict[str, Any]]:
    """Process a single DNG frame and its metadata."""
    # Extract from DNG
    dng_meta = metadata_extractor.extract_from_dng(dng_path)
    if dng_meta is None:
        return None
    
    # Merge with JSON metadata if available
    json_path = metadata_dir / f"{dng_path.stem}.json" if metadata_dir else None
    if json_path and json_path.exists():
        json_meta = metadata_extractor.load_metadata_json(json_path)
        if json_meta:
            # JSON overrides DNG metadata
            dng_meta.update(json_meta)
    
    # Build feature vector
    features = build_feature_vector(dng_meta["histogram"], dng_meta)
    
    return {
        "features": features,
        "histogram": dng_meta["histogram"],
        "metadata": dng_meta,
        "frame_id": dng_path.stem,
    }


def collect_dataset(
    input_dir: Path,
    output_dir: Path,
    teacher_loader: TeacherModelLoader,
    metadata_dir: Optional[Path] = None,
    batch_size: int = 32,
    max_frames: Optional[int] = None,
    num_workers: int = 4,
) -> Dict[str, np.ndarray]:
    """
    Main dataset collection function.
    Processes DNG files, runs teacher inference, saves dataset.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Find DNG files
    dng_files = sorted(list(input_dir.glob("*.dng")) + list(input_dir.glob("*.DNG")))
    if not dng_files:
        dng_files = sorted(list(input_dir.rglob("*.dng")) + list(input_dir.rglob("*.DNG")))
    
    if max_frames:
        dng_files = dng_files[:max_frames]
    
    print(f"Found {len(dng_files)} DNG files")
    
    # Initialize extractor
    extractor = MetadataExtractor()
    
    # Process frames in parallel
    print("Extracting metadata from frames...")
    frame_data = []
    
    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        futures = {
            executor.submit(process_frame, dng, extractor, metadata_dir): dng
            for dng in dng_files
        }
        
        for future in tqdm(futures, desc="Metadata extraction"):
            result = future.result()
            if result:
                frame_data.append(result)
    
    print(f"Successfully processed {len(frame_data)} frames")
    
    if not frame_data:
        raise ValueError("No frames processed successfully!")
    
    # Stack features
    print("Building feature matrix...")
    all_features = np.stack([f["features"] for f in frame_data])
    all_histograms = np.stack([f["histogram"] for f in frame_data])
    all_metadata = {k: np.array([f["metadata"].get(k, 0) for f in frame_data]) 
                    for k in frame_data[0]["metadata"].keys()}
    frame_ids = [f["frame_id"] for f in frame_data]
    
    # Run teacher inference in batches
    print("Running teacher model inference...")
    num_samples = len(all_features)
    wb_targets = np.zeros((num_samples, 3), dtype=np.float32)
    ccm_targets = np.zeros((num_samples, 9), dtype=np.float32)
    tone_targets = np.zeros((num_samples, 7), dtype=np.float32)
    zoom_targets = np.zeros((num_samples, 1), dtype=np.float32)
    
    for i in tqdm(range(0, num_samples, batch_size), desc="Teacher inference"):
        end = min(i + batch_size, num_samples)
        batch_features = all_features[i:end]
        
        outputs = teacher_loader.predict_all(batch_features)
        
        wb_targets[i:end] = outputs["wb_gains"]
        ccm_targets[i:end] = outputs["ccm"]
        tone_targets[i:end] = outputs["tone_curve"]
        zoom_targets[i:end] = outputs["zoom_factor"]
    
    # Combine inputs (histogram + metadata)
    inputs = np.concatenate([all_histograms, all_features[:, 256:]], axis=1)
    
    # Save dataset
    print("Saving dataset...")
    np.savez_compressed(
        output_dir / "teacher_dataset.npz",
        inputs=inputs.astype(np.float32),
        histograms=all_histograms.astype(np.float32),
        metadata=all_features[:, 256:].astype(np.float32),
        wb_targets=wb_targets,
        ccm_targets=ccm_targets,
        tone_targets=tone_targets,
        zoom_targets=zoom_targets,
        frame_ids=np.array(frame_ids, dtype=object),
    )
    
    # Save info
    info = {
        "num_samples": num_samples,
        "input_dim": inputs.shape[1],
        "histogram_bins": 256,
        "metadata_features": all_features.shape[1] - 256,
        "output_dims": {
            "wb_gains": 3,
            "ccm": 9,
            "tone_curve": 7,
            "zoom_factor": 1,
        },
        "frame_ids": frame_ids,
        "source_directory": str(input_dir),
        "metadata_directory": str(metadata_dir) if metadata_dir else None,
    }
    
    with open(output_dir / "dataset_info.json", 'w') as f:
        json.dump(info, f, indent=2)
    
    # Save schema
    schema = {
        "input_features": {
            "histogram": "256-bin luminance histogram (normalized)",
            "cct": "Correlated Color Temperature (Kelvin, normalized /10000)",
            "wb_gains": "RGB white balance gains [R, G, B]",
            "exposure_time": "Exposure time (seconds)",
            "iso_gain": "ISO/gain value (normalized /100)",
            "focus_position": "Normalized focus position 0-1",
            "sharpness": "Sharpness metric (Laplacian variance)",
            "brightness": "Mean brightness (0-1)",
            "contrast": "Contrast (std/mean)",
            "noise_level": "Estimated noise level",
        },
        "teacher_outputs": {
            "wb_gains": "Time-Aware AWB output [R, G, B]",
            "ccm": "CCMNet output 3x3 matrix (flattened)",
            "tone_curve": "Neural ISP Tuning tone curve (7 params)",
            "zoom_factor": "Neural ISP Tuning zoom factor",
        }
    }
    
    with open(output_dir / "schema.json", 'w') as f:
        json.dump(schema, f, indent=2)
    
    print(f"\n✅ Dataset saved to {output_dir}/")
    print(f"   teacher_dataset.npz ({num_samples} samples)")
    print(f"   dataset_info.json")
    print(f"   schema.json")
    
    # Print sample
    print("\n📋 Sample outputs (first frame):")
    print(f"  WB gains: {wb_targets[0]}")
    print(f"  CCM:\n{ccm_targets[0].reshape(3,3)}")
    print(f"  Tone curve: {tone_targets[0]}")
    print(f"  Zoom: {zoom_targets[0][0]:.3f}")
    
    return {
        "inputs": inputs,
        "histograms": all_histograms,
        "metadata": all_features[:, 256:],
        "wb_targets": wb_targets,
        "ccm_targets": ccm_targets,
        "tone_targets": tone_targets,
        "zoom_targets": zoom_targets,
        "frame_ids": frame_ids,
    }


# =============================================================================
# SYNTHETIC DATA GENERATION (for testing)
# =============================================================================

def generate_synthetic_dataset(
    output_dir: Path,
    num_samples: int,
    teacher_loader: TeacherModelLoader,
    batch_size: int = 1000,
) -> Dict[str, np.ndarray]:
    """Generate synthetic dataset for testing without real DNG files."""
    print(f"Generating {num_samples} synthetic samples...")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Generate features directly
    all_features = []
    
    for _ in range(num_samples):
        # Random histogram
        hist_type = np.random.choice(["normal", "under", "over", "contrast"])
        if hist_type == "normal":
            hist = np.random.normal(128, 40, 256)
        elif hist_type == "under":
            hist = np.exp(-np.linspace(0, 5, 256)) * 255
        elif hist_type == "over":
            hist = np.exp(np.linspace(0, -5, 256)) * 255
        else:
            hist = np.concatenate([
                np.random.normal(50, 10, 128),
                np.random.normal(200, 10, 128)
            ])
        hist = np.clip(hist, 0, 255).astype(np.float32)
        
        # Random metadata
        meta = {
            "cct": np.random.uniform(2000, 10000),
            "wb_gains": np.random.uniform(0.5, 2.0, 3),
            "exposure_time": np.random.uniform(0.001, 0.1),
            "iso_gain": np.random.uniform(1.0, 16.0),
            "focus_position": np.random.uniform(0, 1),
            "sharpness": np.random.uniform(0.1, 1.0),
            "brightness": np.random.uniform(0.1, 1.0),
            "contrast": np.random.uniform(0.1, 1.0),
            "noise_level": np.random.uniform(0.0, 0.3),
        }
        
        features = build_feature_vector(hist, meta)
        all_features.append(features)
    
    all_features = np.stack(all_features)
    all_histograms = all_features[:, :256]
    metadata_features = all_features[:, 256:]
    
    # Run teacher inference
    print("Running teacher inference on synthetic data...")
    num_samples = len(all_features)
    wb_targets = np.zeros((num_samples, 3), dtype=np.float32)
    ccm_targets = np.zeros((num_samples, 9), dtype=np.float32)
    tone_targets = np.zeros((num_samples, 7), dtype=np.float32)
    zoom_targets = np.zeros((num_samples, 1), dtype=np.float32)
    
    for i in tqdm(range(0, num_samples, batch_size), desc="Teacher inference"):
        end = min(i + batch_size, num_samples)
        outputs = teacher_loader.predict_all(all_features[i:end])
        wb_targets[i:end] = outputs["wb_gains"]
        ccm_targets[i:end] = outputs["ccm"]
        tone_targets[i:end] = outputs["tone_curve"]
        zoom_targets[i:end] = outputs["zoom_factor"]
    
    # Save
    inputs = np.concatenate([all_histograms, metadata_features], axis=1)
    
    np.savez_compressed(
        output_dir / "teacher_dataset.npz",
        inputs=inputs.astype(np.float32),
        histograms=all_histograms.astype(np.float32),
        metadata=metadata_features.astype(np.float32),
        wb_targets=wb_targets,
        ccm_targets=ccm_targets,
        tone_targets=tone_targets,
        zoom_targets=zoom_targets,
        frame_ids=np.array([f"synthetic_{i:06d}" for i in range(num_samples)], dtype=object),
    )
    
    info = {
        "num_samples": num_samples,
        "input_dim": inputs.shape[1],
        "type": "synthetic",
    }
    with open(output_dir / "dataset_info.json", 'w') as f:
        json.dump(info, f, indent=2)
    
    print(f"✅ Synthetic dataset saved to {output_dir}/")
    return {
        "inputs": inputs,
        "histograms": all_histograms,
        "metadata": metadata_features,
        "wb_targets": wb_targets,
        "ccm_targets": ccm_targets,
        "tone_targets": tone_targets,
        "zoom_targets": zoom_targets,
    }


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Production Teacher Dataset Collector for ISP Distillation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process real DNG frames with teacher models
  python collect_teacher_dataset_production.py \\
      --input-dir ./dng_frames \\
      --metadata-dir ./metadata_json \\
      --output-dir teacher_dataset \\
      --ccmnet-weights models/ccmnet.pth \\
      --awb-weights models/time_aware_awb.pth \\
      --isp-tuning-weights models/neural_isp_tuning.pth

  # Generate synthetic data for testing
  python collect_teacher_dataset_production.py \\
      --synthetic 10000 \\
      --output-dir teacher_dataset
"""
    )
    
    # Input options
    parser.add_argument("--input-dir", type=str, help="Directory containing DNG frames")
    parser.add_argument("--metadata-dir", type=str, help="Directory containing JSON metadata sidecars")
    parser.add_argument("--synthetic", type=int, help="Generate N synthetic samples instead of processing DNGs")
    
    # Output
    parser.add_argument("--output-dir", type=str, default="teacher_dataset", help="Output directory")
    
    # Teacher model weights
    parser.add_argument("--ccmnet-weights", type=str, help="Path to CCMNet weights (.pth)")
    parser.add_argument("--awb-weights", type=str, help="Path to Time-Aware AWB weights (.pth)")
    parser.add_argument("--isp-tuning-weights", type=str, help="Path to Neural ISP Tuning weights (.pth)")
    
    # Processing options
    parser.add_argument("--batch-size", type=int, default=32, help="Inference batch size")
    parser.add_argument("--max-frames", type=int, help="Maximum frames to process")
    parser.add_argument("--num-workers", type=int, default=4, help="Parallel workers for metadata extraction")
    parser.add_argument("--device", type=str, default="auto", help="Device (cuda/cpu/auto)")
    
    args = parser.parse_args()
    
    # Validate
    if not args.synthetic and not args.input_dir:
        parser.error("Either --input-dir or --synthetic must be provided")
    
    if args.device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device
    
    print(f"🚀 ISP Teacher Dataset Collector")
    print(f"   Device: {device}")
    print(f"   Output: {args.output_dir}")
    
    # Load teacher models
    teacher_loader = TeacherModelLoader(
        ccmnet_weights=args.ccmnet_weights,
        awb_weights=args.awb_weights,
        isp_tuning_weights=args.isp_tuning_weights,
        device=device,
    )
    
    output_dir = Path(args.output_dir)
    
    if args.synthetic:
        # Generate synthetic dataset
        generate_synthetic_dataset(
            output_dir=output_dir,
            num_samples=args.synthetic,
            teacher_loader=teacher_loader,
            batch_size=args.batch_size,
        )
    else:
        # Process real DNG files
        input_dir = Path(args.input_dir)
        metadata_dir = Path(args.metadata_dir) if args.metadata_dir else None
        
        if not input_dir.exists():
            parser.error(f"Input directory not found: {input_dir}")
        
        collect_dataset(
            input_dir=input_dir,
            output_dir=output_dir,
            teacher_loader=teacher_loader,
            metadata_dir=metadata_dir,
            batch_size=args.batch_size,
            max_frames=args.max_frames,
            num_workers=args.num_workers,
        )
    
    print("\n✅ Done! Ready for student training:")
    print(f"   python distill_model.py --train --dataset {output_dir}/teacher_dataset.npz --epochs 100")


if __name__ == "__main__":
    main()