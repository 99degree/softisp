#!/usr/bin/env python3
"""
Standalone ONNX Quantization Script
Supports INT8 dynamic quantization and FP16 conversion.
"""

import argparse
import os
import sys
from pathlib import Path

import onnx
from onnxruntime.quantization import quantize_dynamic, QuantType


def quantize_int8(fp32_path: str, int8_path: str, per_channel: bool = True):
    """Quantize FP32 ONNX model to INT8."""
    print(f"Quantizing {fp32_path} -> {int8_path} (INT8 dynamic)")
    
    quantize_dynamic(
        model_input=fp32_path,
        model_output=int8_path,
        weight_type=QuantType.QInt8,
        per_channel=per_channel,
        reduce_range=False,
        extra_options={
            'EnableSubgraph': True,
            'ForceQuantizeNoInputCheck': False,
        }
    )
    print(f"✅ INT8 model saved to {int8_path}")


def quantize_fp16(fp32_path: str, fp16_path: str):
    """Convert FP32 ONNX model to FP16."""
    try:
        from onnxconverter_common import float16
        
        print(f"Converting {fp32_path} -> {fp16_path} (FP16)")
        
        model = onnx.load(fp32_path)
        model_fp16 = float16.convert_float_to_float16(
            model,
            keep_io_types=True,  # Keep inputs/outputs as FP32
        )
        onnx.save(model_fp16, fp16_path)
        print(f"✅ FP16 model saved to {fp16_path}")
        
    except ImportError:
        print("⚠️  onnxconverter-common not installed, skipping FP16")
        print("   Install with: pip install onnxconverter-common")


def validate_model(model_path: str):
    """Validate ONNX model."""
    print(f"Validating {model_path}...")
    model = onnx.load(model_path)
    onnx.checker.check_model(model)
    print(f"✅ Model valid: {model_path}")
    
    # Print model info
    print(f"  IR version: {model.ir_version}")
    print(f"  Opset: {model.opset_import[0].version}")
    print(f"  Inputs: {[i.name for i in model.graph.input]}")
    print(f"  Outputs: {[o.name for o in model.graph.output]}")
    print(f"  Parameters: {len(model.graph.initializer)}")


def get_model_size(path: str) -> float:
    """Get model size in MB."""
    return os.path.getsize(path) / (1024 * 1024)


def main():
    parser = argparse.ArgumentParser(description="Quantize ONNX models for ISP deployment")
    parser.add_argument("model", help="Path to FP32 ONNX model")
    parser.add_argument("--int8", action="store_true", help="Generate INT8 model")
    parser.add_argument("--fp16", action="store_true", help="Generate FP16 model")
    parser.add_argument("--all", action="store_true", help="Generate both INT8 and FP16")
    parser.add_argument("--output-dir", default=".", help="Output directory")
    parser.add_argument("--validate", action="store_true", help="Validate models after quantization")
    
    args = parser.parse_args()
    
    if not (args.int8 or args.fp16 or args.all):
        args.all = True  # Default to all
    
    fp32_path = Path(args.model)
    if not fp32_path.exists():
        print(f"❌ Model not found: {fp32_path}")
        sys.exit(1)
    
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    base_name = fp32_path.stem
    
    # Validate input
    if args.validate:
        validate_model(str(fp32_path))
    
    fp32_size = get_model_size(fp32_path)
    print(f"\n📦 FP32 model size: {fp32_size:.2f} MB")
    
    results = {}
    
    if args.int8 or args.all:
        int8_path = output_dir / f"{base_name}_int8.onnx"
        quantize_int8(str(fp32_path), str(int8_path))
        results['INT8'] = get_model_size(int8_path)
        
        if args.validate:
            validate_model(str(int8_path))
    
    if args.fp16 or args.all:
        fp16_path = output_dir / f"{base_name}_fp16.onnx"
        quantize_fp16(str(fp32_path), str(fp16_path))
        results['FP16'] = get_model_size(fp16_path)
        
        if args.validate:
            validate_model(str(fp16_path))
    
    # Summary
    print("\n" + "="*50)
    print("QUANTIZATION SUMMARY")
    print("="*50)
    print(f"FP32:  {fp32_size:.2f} MB (baseline)")
    for name, size in results.items():
        reduction = (1 - size / fp32_size) * 100
        print(f"{name}:   {size:.2f} MB ({reduction:.1f}% reduction)")


if __name__ == "__main__":
    main()