#!/usr/bin/env python3
"""
Quick sanity check for the ISP Rectifier pipeline.
Run this to verify everything is working.
"""

import sys
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

def check_imports():
    """Check all required imports."""
    print("🔍 Checking imports...")
    
    try:
        import torch
        print(f"  ✅ PyTorch {torch.__version__}")
    except ImportError:
        print("  ❌ PyTorch not installed")
        return False
    
    try:
        import onnx
        print(f"  ✅ ONNX {onnx.__version__}")
    except ImportError:
        print("  ❌ ONNX not installed")
        return False
    
    try:
        import onnxruntime as ort
        print(f"  ✅ ONNX Runtime {ort.__version__}")
    except ImportError:
        print("  ❌ ONNX Runtime not installed")
        return False
    
    try:
        import rawpy
        print(f"  ✅ rawpy available")
    except ImportError:
        print("  ⚠️  rawpy not available (DNG processing disabled)")
    
    try:
        import cv2
        print(f"  ✅ OpenCV {cv2.__version__}")
    except ImportError:
        print("  ❌ OpenCV not installed")
        return False
    
    try:
        import tract_onnx
        print(f"  ✅ tract-onnx available")
    except ImportError:
        print("  ❌ tract-onnx not installed")
        return False
    
    return True


def check_model_files():
    """Check for model files."""
    print("\n🔍 Checking model files...")
    
    models = [
        "fusedispcontroller.onnx",
        "fusedispcontroller_int8.onnx", 
        "fusedispcontroller_fp16.onnx",
    ]
    
    for model in models:
        path = Path(model)
        if path.exists():
            size = path.stat().st_size / (1024 * 1024)
            print(f"  ✅ {model} ({size:.2f} MB)")
        else:
            print(f"  ⚠️  {model} not found")
    
    return True


def check_dataset():
    """Check for teacher dataset."""
    print("\n🔍 Checking dataset...")
    
    dataset_path = Path("teacher_dataset/teacher_dataset.npz")
    if dataset_path.exists():
        import numpy as np
        data = np.load(dataset_path)
        print(f"  ✅ Dataset found: {len(data['inputs'])} samples")
        print(f"     Inputs shape: {data['inputs'].shape}")
        print(f"     Keys: {list(data.keys())}")
        return True
    else:
        print("  ⚠️  No teacher dataset found")
        return False


def test_inference():
    """Test basic inference."""
    print("\n🧪 Testing inference...")
    
    try:
        from inference import ISPOptimizer, build_feature_vector
        
        # Try to load a model
        model_path = "fusedispcontroller_int8.onnx"
        if not Path(model_path).exists():
            model_path = "fusedispcontroller.onnx"
        if not Path(model_path).exists():
            print("  ⚠️  No model found, skipping inference test")
            return True
        
        optimizer = ISPOptimizer(model_path)
        
        # Create dummy input
        hist = np.random.randn(256).astype(np.float32)
        meta = np.random.randn(11).astype(np.float32)
        
        outputs = optimizer.optimize_raw(hist, meta)
        
        print(f"  ✅ Inference successful")
        print(f"     WB gains: {outputs['wb_gains']}")
        print(f"     CCM shape: {outputs['ccm'].shape}")
        print(f"     Tone curve: {outputs['tone_curve'].shape}")
        print(f"     Zoom: {outputs['zoom_factor'].shape}")
        
        return True
    except Exception as e:
        print(f"  ❌ Inference test failed: {e}")
        return False


def check_rust():
    """Check Rust build."""
    print("\n🔍 Checking Rust...")
    
    import subprocess
    result = subprocess.run(["cargo", "check", "--manifest-path", "Cargo.toml"], 
                          capture_output=True, text=True)
    if result.returncode == 0:
        print("  ✅ Rust crate compiles")
        return True
    else:
        print(f"  ❌ Rust check failed: {result.stderr[:200]}")
        return False


def main():
    print("=" * 60)
    print("ISP Rectifier - System Check")
    print("=" * 60)
    
    checks = [
        ("Imports", check_imports),
        ("Model Files", check_model_files),
        ("Dataset", check_dataset),
        ("Inference", test_inference),
        ("Rust", check_rust),
    ]
    
    results = []
    for name, check_fn in checks:
        try:
            results.append((name, check_fn()))
        except Exception as e:
            print(f"  ❌ {name} check crashed: {e}")
            results.append((name, False))
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    all_passed = True
    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  {status}: {name}")
        if not passed:
            all_passed = False
    
    print("=" * 60)
    if all_passed:
        print("🎉 All checks passed!")
        return 0
    else:
        print("⚠️  Some checks failed - run setup or fix issues")
        return 1


if __name__ == "__main__":
    sys.exit(main())