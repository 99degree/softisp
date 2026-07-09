#!/usr/bin/env python3
"""Test script for inference module."""

import json
import numpy as np
import tempfile
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))

from inference import ISPOptimizer, build_feature_vector, parse_outputs, clamp_params


def test_feature_vector():
    """Test feature vector construction."""
    hist = np.random.randint(0, 256, 256).astype(np.float32)
    meta = build_feature_vector(
        histogram=hist,
        cct=5500.0,
        wb_gains=(1.2, 1.0, 0.9),
        exposure_time=0.033,
        iso_gain=2.0,
        focus_position=0.5,
        sharpness=0.8,
        brightness=0.6,
        contrast=0.7,
        noise_level=0.1,
    )
    
    hist_vec, meta_vec = meta
    assert len(hist_vec) == 256
    assert len(meta_vec) == 11
    
    # Check specific values
    assert abs(meta_vec[0] - 0.55) < 0.01  # CCT normalized
    assert abs(meta_vec[1] - 1.2) < 0.01   # WB R
    assert abs(meta_vec[2] - 1.0) < 0.01   # WB G
    assert abs(meta_vec[3] - 0.9) < 0.01   # WB B
    assert abs(meta_vec[4] - 0.033) < 0.001  # exposure
    assert abs(meta_vec[5] - 2.0) < 0.01    # ISO
    
    print("✅ Feature vector test passed")


def test_parse_outputs():
    """Test output parsing."""
    # Simulate ONNX outputs
    outputs = {
        "wbgains": np.array([[1.1, 1.0, 0.9]], dtype=np.float32),
        "ccm": np.eye(3).flatten().reshape(1, -1).astype(np.float32),
        "tonecurve": np.linspace(0, 1, 7).reshape(1, -1).astype(np.float32),
        "zoom_factor": np.array([[1.5]], dtype=np.float32),
    }
    
    parsed = parse_outputs(outputs)
    
    assert "wb_gains" in parsed
    assert "ccm" in parsed
    assert "tone_curve" in parsed
    assert "zoom_factor" in parsed
    
    assert parsed["wb_gains"] == [1.1, 1.0, 0.9]
    assert parsed["ccm"] == [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    assert len(parsed["tone_curve"]) == 7
    assert parsed["zoom_factor"] == 1.5
    
    print("✅ Output parsing test passed")


def test_clamp_params():
    """Test parameter clamping."""
    params = {
        "wb_gains": [15.0, -0.5, 2.0],
        "ccm": [[3.0, -3.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "tone_curve": [-0.5, 1.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        "zoom_factor": 10.0,
    }
    
    clamped = clamp_params(params)
    
    assert all(0.1 <= v <= 10.0 for v in clamped["wb_gains"])
    assert all(-2.0 <= v <= 2.0 for row in clamped["ccm"] for v in row)
    assert all(0.0 <= v <= 1.0 for v in clamped["tone_curve"])
    assert 1.0 <= clamped["zoom_factor"] <= 4.0
    
    print("✅ Clamping test passed")


def test_input_json():
    """Test JSON input format."""
    input_data = {
        "histogram": list(np.random.randint(0, 256, 256)),
        "cct": 5500.0,
        "wb_gains": [1.2, 1.0, 0.9],
        "exposure_time": 0.033,
        "iso_gain": 2.0,
        "focus_position": 0.5,
        "sharpness": 0.8,
        "brightness": 0.6,
        "contrast": 0.7,
        "noise_level": 0.1,
    }
    
    # Save to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(input_data, f)
        temp_path = f.name
    
    try:
        with open(temp_path) as f:
            loaded = json.load(f)
        
        assert loaded["cct"] == 5500.0
        assert loaded["wb_gains"] == [1.2, 1.0, 0.9]
        assert len(loaded["histogram"]) == 256
        
        print("✅ JSON input test passed")
    finally:
        Path(temp_path).unlink()


def main():
    print("Running inference tests...\n")
    
    test_feature_vector()
    test_parse_outputs()
    test_clamp_params()
    test_input_json()
    
    print("\n✅ All tests passed!")


if __name__ == "__main__":
    main()