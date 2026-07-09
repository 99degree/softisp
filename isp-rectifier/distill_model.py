#!/usr/bin/env python3
"""
Distilled ISP Controller Model:
- Input: Histogram (1D), CCT, WB gains, 3A metadata, and other frame metadata.
- Output: Rectified parameters (WB gains, CCM, tone curve, zoom, etc.)
- Training: Distilled from teacher models (CCMNet, Time-Aware AWB, Neural ISP Tuning).
- Export: ONNX for Rust pipeline integration.

Usage:
    # Train with teacher dataset
    python distill_model.py --train --dataset teacher_dataset/teacher_dataset.npz --epochs 100

    # Export ONNX
    python distill_model.py --export --model checkpoints/best_model.pth --output fusedispcontroller.onnx

    # Full pipeline: train + export
    python distill_model.py --train --dataset teacher_dataset/teacher_dataset.npz --epochs 100 --export --output fusedispcontroller.onnx
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import numpy as np
import argparse
import json
from pathlib import Path
from typing import Tuple, Dict, Any, Optional
import sys


class ISPDistilledModel(nn.Module):
    """
    Distilled ISP Controller Model.
    Input:
        - histogram: (B, 256)  # 256-bin histogram
        - metadata: (B, N)     # Flattened metadata (CCT, WB, 3A, etc.)
    Output:
        - wbgains: (B, 3)      # WB gains [R, G, B]
        - ccm: (B, 9)          # CCM matrix (flattened 3x3)
        - tonecurve: (B, 7)    # Tone curve parameters
        - zoom_factor: (B, 1)  # Zoom factor
    """
    
    def __init__(self, metadata_dim: int = 267):
        super().__init__()
        
        # Histogram backbone: 1D CNN
        self.hist_backbone = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=7, padding=3),
            nn.BatchNorm1d(16),
            nn.ReLU(),
            nn.MaxPool1d(2),  # 256 -> 128
            nn.Conv1d(16, 32, kernel_size=5, padding=2),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.MaxPool1d(2),  # 128 -> 64
            nn.Conv1d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(32),  # -> 32
            nn.Flatten(),  # 64 * 32 = 2048
        )
        
        # Metadata backbone: MLP
        self.meta_backbone = nn.Sequential(
            nn.Linear(metadata_dim, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(128, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.Dropout(0.1),
        )
        
        # Combined feature dimension
        hist_out_dim = 64 * 32  # 2048
        meta_out_dim = 256
        combined_dim = hist_out_dim + meta_out_dim  # 2304
        
        # Shared fusion layer
        self.fusion = nn.Sequential(
            nn.Linear(combined_dim, 512),
            nn.BatchNorm1d(512),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(512, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.Dropout(0.1),
        )
        
        # Multi-head outputs
        self.wb_head = nn.Sequential(
            nn.Linear(256, 64),
            nn.ReLU(),
            nn.Linear(64, 3),
        )
        
        self.ccm_head = nn.Sequential(
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 9),
        )
        
        self.tone_head = nn.Sequential(
            nn.Linear(256, 64),
            nn.ReLU(),
            nn.Linear(64, 7),
        )
        
        self.zoom_head = nn.Sequential(
            nn.Linear(256, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
        )
    
    def forward(self, histogram: torch.Tensor, metadata: torch.Tensor) -> Dict[str, torch.Tensor]:
        # Histogram backbone
        hist_features = self.hist_backbone(histogram.unsqueeze(1))
        
        # Metadata backbone
        meta_features = self.meta_backbone(metadata)
        
        # Combine and fuse
        combined = torch.cat([hist_features, meta_features], dim=1)
        fused = self.fusion(combined)
        
        # Multi-head outputs
        return {
            "wbgains": self.wb_head(fused),
            "ccm": self.ccm_head(fused),
            "tonecurve": self.tone_head(fused),
            "zoom_factor": self.zoom_head(fused),
        }


def distillation_loss(
    student_output: Dict[str, torch.Tensor],
    teacher_output: Dict[str, torch.Tensor],
    weights: Dict[str, float] = None,
) -> torch.Tensor:
    """Compute distillation loss between student and teacher outputs."""
    if weights is None:
        weights = {"wbgains": 1.0, "ccm": 1.0, "tonecurve": 0.5, "zoom_factor": 0.5}
    
    loss = 0.0
    for key in student_output:
        if key in teacher_output:
            loss += weights[key] * nn.MSELoss()(student_output[key], teacher_output[key])
    
    return loss


def load_teacher_dataset(dataset_path: str) -> Tuple[DataLoader, int]:
    """Load teacher dataset from NPZ file."""
    data = np.load(dataset_path, allow_pickle=True)
    
    # Inputs: histogram + metadata concatenated
    # We need to split them back
    # Assuming dataset was saved with keys: 'inputs', 'wb_targets', 'ccm_targets', 'tone_targets', 'zoom_targets'
    if 'inputs' in data:
        inputs = data['inputs']
        # Split: first 256 = histogram, rest = metadata
        histogram = inputs[:, :256]
        metadata = inputs[:, 256:]
    else:
        # Legacy format: separate arrays
        histogram = data['histograms']
        metadata = data['metadata']
    
    wb_targets = data['wb_targets']
    ccm_targets = data['ccm_targets']
    tone_targets = data['tone_targets']
    zoom_targets = data['zoom_targets']
    
    print(f"Loaded dataset: {len(histogram)} samples")
    print(f"  Histogram shape: {histogram.shape}")
    print(f"  Metadata shape: {metadata.shape}")
    print(f"  WB targets shape: {wb_targets.shape}")
    print(f"  CCM targets shape: {ccm_targets.shape}")
    print(f"  Tone targets shape: {tone_targets.shape}")
    print(f"  Zoom targets shape: {zoom_targets.shape}")
    
    # Convert to tensors
    histogram = torch.from_numpy(histogram).float()
    metadata = torch.from_numpy(metadata).float()
    wb_targets = torch.from_numpy(wb_targets).float()
    ccm_targets = torch.from_numpy(ccm_targets).float()
    tone_targets = torch.from_numpy(tone_targets).float()
    zoom_targets = torch.from_numpy(zoom_targets).float()
    
    dataset = TensorDataset(histogram, metadata, wb_targets, ccm_targets, tone_targets, zoom_targets)
    
    return dataset, metadata.shape[1]


def train_model(
    model: ISPDistilledModel,
    dataset: TensorDataset,
    epochs: int = 100,
    batch_size: int = 32,
    lr: float = 1e-3,
    weight_decay: float = 1e-4,
    val_split: float = 0.1,
    device: str = "cuda",
    checkpoint_dir: str = "checkpoints",
) -> ISPDistilledModel:
    """Train the distilled model."""
    device = torch.device(device if torch.cuda.is_available() else "cpu")
    model = model.to(device)
    
    # Split train/val
    val_size = int(len(dataset) * val_split)
    train_size = len(dataset) - val_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4)
    
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    
    Path(checkpoint_dir).mkdir(parents=True, exist_ok=True)
    
    best_val_loss = float('inf')
    history = {"train_loss": [], "val_loss": []}
    
    print(f"Training on {device} for {epochs} epochs...")
    print(f"Train samples: {train_size}, Val samples: {val_size}")
    
    for epoch in range(epochs):
        # Training
        model.train()
        train_loss = 0.0
        
        for hist, meta, wb_t, ccm_t, tone_t, zoom_t in train_loader:
            hist = hist.to(device)
            meta = meta.to(device)
            
            teacher_output = {
                "wbgains": wb_t.to(device),
                "ccm": ccm_t.to(device),
                "tonecurve": tone_t.to(device),
                "zoom_factor": zoom_t.to(device),
            }
            
            optimizer.zero_grad()
            student_output = model(hist, meta)
            loss = distillation_loss(student_output, teacher_output)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
        
        train_loss /= len(train_loader)
        
        # Validation
        model.eval()
        val_loss = 0.0
        
        with torch.no_grad():
            for hist, meta, wb_t, ccm_t, tone_t, zoom_t in val_loader:
                hist = hist.to(device)
                meta = meta.to(device)
                
                teacher_output = {
                    "wbgains": wb_t.to(device),
                    "ccm": ccm_t.to(device),
                    "tonecurve": tone_t.to(device),
                    "zoom_factor": zoom_t.to(device),
                }
                
                student_output = model(hist, meta)
                loss = distillation_loss(student_output, teacher_output)
                val_loss += loss.item()
        
        val_loss /= len(val_loader)
        scheduler.step()
        
        history["train_loss"].append(train_loss)
        history["val_loss"].append(val_loss)
        
        print(f"Epoch {epoch+1:3d}/{epochs} | Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f} | LR: {optimizer.param_groups[0]['lr']:.2e}")
        
        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_loss': val_loss,
                'metadata_dim': meta.shape[1],
            }, f"{checkpoint_dir}/best_model.pth")
            print(f"  ✅ Saved best model (val_loss: {val_loss:.6f})")
    
    # Save final model
    torch.save({
        'epoch': epochs,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'val_loss': val_loss,
        'metadata_dim': meta.shape[1],
    }, f"{checkpoint_dir}/final_model.pth")
    
    # Save history
    with open(f"{checkpoint_dir}/history.json", 'w') as f:
        json.dump(history, f, indent=2)
    
    print(f"\nTraining complete! Best val loss: {best_val_loss:.6f}")
    return model


def export_to_onnx(
    model: ISPDistilledModel,
    output_path: str,
    metadata_dim: int,
    opset: int = 13,
):
    """Export model to ONNX format."""
    model.eval()
    
    # Dummy inputs
    dummy_histogram = torch.randn(1, 256)
    dummy_metadata = torch.randn(1, metadata_dim)
    
    # Forward pass
    output = model(dummy_histogram, dummy_metadata)
    
    output_names = list(output.keys())
    dynamic_axes = {
        "histogram": {0: "batch"},
        "metadata": {0: "batch"},
    }
    for name in output_names:
        dynamic_axes[name] = {0: "batch"}
    
    torch.onnx.export(
        model,
        (dummy_histogram, dummy_metadata),
        output_path,
        input_names=["histogram", "metadata"],
        output_names=output_names,
        dynamic_axes=dynamic_axes,
        opset_version=opset,
        do_constant_folding=True,
    )
    print(f"✅ Model exported to {output_path}")
    
    # Verify ONNX model
    import onnx
    onnx_model = onnx.load(output_path)
    onnx.checker.check_model(onnx_model)
    
    # Re-save to ensure self-contained (no external data)
    # If external .onnx.data was created, this embeds it back
    onnx.save(onnx_model, output_path)
    size = Path(output_path).stat().st_size / 1024
    print(f"✅ Model saved self-contained ({size:.1f} KB)")


def quantize_onnx(fp32_path: str, int8_path: str, fp16_path: str):
    """Quantize ONNX model to INT8 and FP16."""
    try:
        from onnxruntime.quantization import quantize_dynamic, QuantType
        quantize_dynamic(fp32_path, int8_path, weight_type=QuantType.QInt8)
        print(f"✅ INT8 model saved to {int8_path}")
    except Exception as e:
        print(f"⚠️ INT8 quantization failed: {e}")
    
    try:
        from onnxconverter_common import float16
        import onnx
        model = onnx.load(fp32_path)
        model_fp16 = float16.convert_float_to_float16(model)
        onnx.save(model_fp16, fp16_path)
        print(f"✅ FP16 model saved to {fp16_path}")
    except Exception as e:
        print(f"⚠️ FP16 quantization failed: {e}")


def main():
    parser = argparse.ArgumentParser(description="Distilled ISP Controller Training & Export")
    parser.add_argument("--train", action="store_true", help="Train the model")
    parser.add_argument("--dataset", type=str, default="teacher_dataset/teacher_dataset.npz", help="Path to teacher dataset")
    parser.add_argument("--epochs", type=int, default=100, help="Number of epochs")
    parser.add_argument("--batch-size", type=int, default=32, help="Batch size")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate")
    parser.add_argument("--device", type=str, default="auto", help="Device (cuda/cpu/auto)")
    parser.add_argument("--checkpoint-dir", type=str, default="checkpoints", help="Checkpoint directory")
    parser.add_argument("--export", action="store_true", help="Export to ONNX")
    parser.add_argument("--model", type=str, default="checkpoints/best_model.pth", help="Model checkpoint to export")
    parser.add_argument("--output", type=str, default="fusedispcontroller.onnx", help="ONNX output path")
    parser.add_argument("--quantize", action="store_true", help="Quantize exported ONNX model")
    
    args = parser.parse_args()
    
    if args.device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device
    
    if args.train:
        # Load dataset
        dataset, metadata_dim = load_teacher_dataset(args.dataset)
        
        # Initialize model
        model = ISPDistilledModel(metadata_dim=metadata_dim)
        print(f"Model initialized with metadata_dim={metadata_dim}")
        print(f"Parameters: {sum(p.numel() for p in model.parameters()):,}")
        
        # Train
        model = train_model(
            model=model,
            dataset=dataset,
            epochs=args.epochs,
            batch_size=args.batch_size,
            lr=args.lr,
            device=device,
            checkpoint_dir=args.checkpoint_dir,
        )
    
    if args.export:
        # Load model
        checkpoint = torch.load(args.model, map_location="cpu")
        metadata_dim = checkpoint.get('metadata_dim', 267)
        
        model = ISPDistilledModel(metadata_dim=metadata_dim)
        model.load_state_dict(checkpoint['model_state_dict'])
        
        # Export ONNX
        export_to_onnx(model, args.output, metadata_dim)
        
        # Quantize if requested
        if args.quantize:
            int8_path = args.output.replace(".onnx", "_int8.onnx")
            fp16_path = args.output.replace(".onnx", "_fp16.onnx")
            quantize_onnx(args.output, int8_path, fp16_path)
    
    if not args.train and not args.export:
        parser.print_help()


if __name__ == "__main__":
    main()