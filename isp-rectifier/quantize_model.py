#!/usr/bin/env python3
"""
Quantize an ONNX model with fallback to FP32 copy on failure.
Usage: python quantize_model.py <input.onnx> <output.onnx>
"""
import sys, os, shutil

def quantize_int8(input_path, output_path):
    try:
        from onnxruntime.quantization import quantize_dynamic, QuantType
        quantize_dynamic(input_path, output_path, weight_type=QuantType.QInt8)
        print(f"INT8 quantized: {output_path}")
    except Exception as e:
        print(f"Quantization failed: {e}")
        shutil.copy(input_path, output_path)
        print(f"Copied FP32 as fallback: {output_path}")

def quantize_fp16(input_path, output_path):
    try:
        from onnxconverter_common import float16
        import onnx
        model = onnx.load(input_path)
        model_fp16 = float16.convert_float_to_float16(model)
        onnx.save(model_fp16, output_path)
        print(f"FP16 quantized: {output_path}")
    except Exception as e:
        print(f"FP16 failed: {e}")
        shutil.copy(input_path, output_path)
        print(f"Copied FP32 as fallback: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python quantize_model.py <int8|fp16> <input.onnx> <output.onnx>")
        sys.exit(1)
    mode = sys.argv[1]
    if mode == "int8":
        quantize_int8(sys.argv[2], sys.argv[3])
    elif mode == "fp16":
        quantize_fp16(sys.argv[2], sys.argv[3])
    else:
        print(f"Unknown mode: {mode}")
        sys.exit(1)
