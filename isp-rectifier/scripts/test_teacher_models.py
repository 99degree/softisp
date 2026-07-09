#!/usr/bin/env python3
"""
Test script to verify teacher models can be loaded and run inference.
"""

import sys
import torch
from pathlib import Path

def test_time_aware_awb():
    """Test Time-Aware AWB model loading."""
    print("Testing Time-Aware AWB...")
    awb_path = Path("time-aware-awb")
    
    sys.path.insert(0, str(awb_path))
    from time_aware_awb.model import IllumEstimator
    
    # Check available model files
    models_dir = awb_path / "models"
    model_files = list(models_dir.glob("model-awb*.pt"))
    print(f"  Available models: {[f.name for f in model_files]}")
    
    # Load first model
    model_file = model_files[0]
    state_dict = torch.load(model_file, map_location="cpu")
    if 'state_dict' in state_dict:
        state_dict = state_dict['state_dict']
    if 'model' in state_dict:
        state_dict = state_dict['model']
    
    # Check model architecture - IllumEstimator has in_channels and hist_channels
    # From model.py: IllumEstimator(in_channels=15, hist_channels=1)
    model = IllumEstimator(in_channels=15, hist_channels=1)
    model.load_state_dict(state_dict, strict=False)
    model.eval()
    
    # Test inference
    hist = torch.randn(1, 1, 1, 256)  # [B, C, H, W] for Conv2d
    meta = torch.randn(1, 15)  # capture_data
    
    with torch.no_grad():
        out = model(hist, meta)
    
    print(f"  ✅ Model loaded successfully")
    print(f"  Input hist: {hist.shape}, meta: {meta.shape}")
    print(f"  Output: {out.shape} = {out[0]}")
    return True


def test_ccmnet():
    """Test CCMNet model loading."""
    print("Testing CCMNet...")
    ccmnet_path = Path("ccmnet")
    
    sys.path.insert(0, str(ccmnet_path / "src"))
    from ccmnet.ccmnet import CCMNet
    
    # Find checkpoint
    ckpt_candidates = list(ccmnet_path.glob("*.pth")) + list(ccmnet_path.glob("*.pt"))
    ckpt_candidates += list((ccmnet_path / "src").glob("*.pth"))
    ckpt_candidates += list((ccmnet_path / "src" / "ccmnet").glob("*.pth"))
    
    if not ckpt_candidates:
        print("  ⚠️  No checkpoint found, using random weights")
        model = CCMNet()
    else:
        ckpt_path = ckpt_candidates[0]
        print(f"  Using checkpoint: {ckpt_path}")
        state_dict = torch.load(ckpt_path, map_location="cpu")
        if 'state_dict' in state_dict:
            state_dict = state_dict['state_dict']
        if 'model' in state_dict:
            state_dict = state_dict['model']
        
        model = CCMNet()
        model.load_state_dict(state_dict, strict=False)
    
    model.eval()
    
    # Test inference
    hist = torch.randn(1, 256)
    meta = torch.randn(1, 11)
    
    with torch.no_grad():
        out = model(hist, meta)
    
    print(f"  ✅ Model loaded successfully")
    print(f"  Input hist: {hist.shape}, meta: {meta.shape}")
    print(f"  Output: {out.shape} = {out[0]}")
    return True


def test_ccmnet_forward():
    """Test CCMNet forward pass details."""
    print("Testing CCMNet architecture...")
    
    sys.path.insert(0, "ccmnet/src")
    from ccmnet.ccmnet import CCMNet
    
    model = CCMNet()
    print(f"  Model: {model}")
    
    # Count parameters
    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"  Total params: {total_params:,}")
    print(f"  Trainable params: {trainable_params:,}")
    
    # Test forward
    hist = torch.randn(2, 256)
    meta = torch.randn(2, 11)
    
    with torch.no_grad():
        out = model(hist, meta)
    
    print(f"  Input: hist={hist.shape}, meta={meta.shape}")
    print(f"  Output: {out.shape}")
    return True


if __name__ == "__main__":
    print("=" * 60)
    print("Teacher Model Verification Tests")
    print("=" * 60)
    
    try:
        test_time_aware_awb()
        print()
    except Exception as e:
        print(f"  ❌ AWB test failed: {e}")
        import traceback
        traceback.print_exc()
    
    print()
    try:
        test_ccmnet_forward()
        print()
    except Exception as e:
        print(f"  ❌ CCMNet architecture test failed: {e}")
        import traceback
        traceback.print_exc()
    
    print()
    try:
        test_ccmnet()
        print()
    except Exception as e:
        print(f"  ❌ CCMNet test failed: {e}")
        import traceback
        traceback.print_exc()
    
    print("=" * 60)
    print("All tests completed!")