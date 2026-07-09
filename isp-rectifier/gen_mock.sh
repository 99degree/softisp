#!/bin/bash
# Generate mock ONNX model for ISP Rectifier pipeline testing.
#
# Usage:
#   ./gen_mock.sh                  # → models/fusedispcontroller.onnx
#   ./gen_mock.sh my_model.onnx   # custom output
#   ./gen_mock.sh --test           # generate + benchmark
#
# Requirements: pip install onnx numpy onnxruntime

set -e
cd "$(dirname "$0")"

mkdir -p models

python3 gen_mock_onnx.py -o "${1:-models/fusedispcontroller.onnx}" ${@:2}

echo ""
echo "Files in models/:"
ls -lh models/*.onnx 2>/dev/null || echo "  (none)"
