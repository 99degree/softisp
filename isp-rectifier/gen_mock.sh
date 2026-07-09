#!/bin/bash
# Generate mock ONNX model using existing ISPDistilledModel architecture.
#
# Usage:
#   ./gen_mock.sh                  # FP32 → models/fusedispcontroller.onnx
#   ./gen_mock.sh --all            # FP32 + INT8 + FP16
#   ./gen_mock.sh -o custom.onnx   # custom output path
#
# Requirements:
#   pip install torch onnx numpy
#   pip install onnxruntime onnxconverter-common  # for quantization + benchmark

set -e
cd "$(dirname "$0")"

mkdir -p models

python3 gen_mock.py \
    -o "${1:-models/fusedispcontroller.onnx}" \
    "${@:2}"

echo ""
echo "Models generated:"
ls -lh models/*.onnx 2>/dev/null || echo "  (none)"
