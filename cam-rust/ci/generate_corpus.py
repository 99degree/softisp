#!/usr/bin/env python3
"""
SoftISP Test Corpus Generator
Generates test data for fuzzing and regression testing
"""

import os
import sys
import json
import struct
from pathlib import Path

def create_onnx_corpus(output_dir: str):
    """Create ONNX test corpus with valid and edge-case models"""
    os.makedirs(output_dir, exist_ok=True)
    
    # Empty model
    with open(os.path.join(output_dir, "empty.onnx"), "wb") as f:
        f.write(b"")
    
    # Minimal valid ONNX header (magic number + version)
    with open(os.path.join(output_dir, "minimal_header.onnx"), "wb") as f:
        f.write(struct.pack("<I", 0x08000120))  # IR version 8, opset 1
        f.write(b"\x00" * 100)
    
    # Truncated model
    with open(os.path.join(output_dir, "truncated.onnx"), "wb") as f:
        f.write(b"\x08\x00\x01\x20")
    
    # Max-length field values
    with open(os.path.join(output_dir, "max_fields.onnx"), "wb") as f:
        f.write(b"\xff" * 4096)
    
    print(f"Created ONNX corpus in {output_dir}")

def create_pipeline_corpus(output_dir: str):
    """Create pipeline config test corpus"""
    os.makedirs(output_dir, exist_ok=True)
    
    # Empty config
    with open(os.path.join(output_dir, "empty.json"), "w") as f:
        f.write("")
    
    # Valid minimal config
    with open(os.path.join(output_dir, "minimal.json"), "w") as f:
        json.dump({"width": 1920, "height": 1080, "blocks": []}, f)
    
    # Config with all block types
    with open(os.path.join(output_dir, "all_blocks.json"), "w") as f:
        json.dump({
            "width": 1920,
            "height": 1080,
            "blocks": [
                "unpack", "demosaic", "ccm", "gamma", "sharpen",
                "denoise", "contrast", "display"
            ]
        }, f)
    
    # Deeply nested config
    with open(os.path.join(output_dir, "nested.json"), "w") as f:
        json.dump({"a": {"b": {"c": {"d": {"e": 1}}}}}, f)
    
    # Very long block name
    with open(os.path.join(output_dir, "long_name.json"), "w") as f:
        json.dump({"width": 1920, "height": 1080, "blocks": ["a" * 10000]}, f)
    
    print(f"Created pipeline corpus in {output_dir}")

def create_profile_corpus(output_dir: str):
    """Create profile test corpus"""
    os.makedirs(output_dir, exist_ok=True)
    
    profiles = ["high", "medium", "low", "HIGH", "MEDIUM", "LOW", 
                "High", "Medium", "Low", "", "invalid", "x" * 1000]
    
    for i, profile in enumerate(profiles):
        with open(os.path.join(output_dir, f"profile_{i}.txt"), "w") as f:
            f.write(profile)
    
    print(f"Created profile corpus in {output_dir}")

def create_sensor_corpus(output_dir: str):
    """Create sensor-specific test data for golden image testing"""
    os.makedirs(output_dir, exist_ok=True)
    
    sensors = {
        "imx586": {"width": 8000, "height": 6000, "bayer": "RGGB", "bits": 10},
        "ov13858": {"width": 4208, "height": 3120, "bayer": "RGGB", "bits": 10},
        "hi1336": {"width": 4000, "height": 3000, "bayer": "GRBG", "bits": 10},
        "s5k3l6": {"width": 4224, "height": 3136, "bayer": "RGGB", "bits": 10},
    }
    
    for name, props in sensors.items():
        sensor_dir = os.path.join(output_dir, name)
        os.makedirs(sensor_dir, exist_ok=True)
        
        # Save sensor properties
        with open(os.path.join(sensor_dir, "properties.json"), "w") as f:
            json.dump(props, f, indent=2)
        
        # Generate synthetic Bayer data (1024x1024 crop)
        width = min(props["width"], 1024)
        height = min(props["height"], 1024)
        
        # Generate gradient pattern
        data = bytearray(width * height * 2)  # 16-bit per pixel
        for y in range(height):
            for x in range(width):
                # Create gradient with Bayer pattern
                idx = (y * width + x) * 2
                value = ((x + y) % 1024) << (16 - props["bits"])
                data[idx] = value & 0xFF
                data[idx + 1] = (value >> 8) & 0xFF
        
        with open(os.path.join(sensor_dir, "gradient.raw"), "wb") as f:
            f.write(data)
    
    print(f"Created sensor corpus in {output_dir}")

if __name__ == "__main__":
    corpus_dir = sys.argv[1] if len(sys.argv) > 1 else "fuzz/testdata"
    
    print("Generating SoftISP test corpus...")
    print(f"Output directory: {corpus_dir}")
    print()
    
    create_onnx_corpus(os.path.join(corpus_dir, "onnx"))
    create_pipeline_corpus(os.path.join(corpus_dir, "pipeline"))
    create_profile_corpus(os.path.join(corpus_dir, "profile"))
    create_sensor_corpus(os.path.join(corpus_dir, "sensors"))
    
    print()
    print("Test corpus generation complete!")
