#!/bin/bash
# Generate mock ONNX model using existing ISPDistilledModel architecture.
#
# Usage:
#   ./gen_mock.sh                          # full FP32 → models/fusedispcontroller.onnx
#   ./gen_mock.sh --all                    # full FP32 + INT8 + FP16
#   ./gen_mock.sh --light                  # lightweight model (~118K)
#   ./gen_mock.sh --medium                 # medium-weight model (~350K)
#   ./gen_mock.sh -o custom.onnx           # custom output path
#   ./gen_mock.sh --all --light -o out.onnx  # light model with all formats
#
# Requirements:
#   pip install torch onnx numpy
#   pip install onnxruntime onnxconverter-common  # for quantization + benchmark

set -e
cd "$(dirname "$0")"

mkdir -p models

python3 gen_mock.py "$@"

echo ""
echo "Models generated:"
ls -lh models/*.onnx 2>/dev/null || echo "  (none)"

# Create all-in-one zip
if ls models/*.onnx 1>/dev/null 2>&1; then
  mkdir -p dist
  cp models/*.onnx dist/
  [ -f MODEL_CARD.md ] && cp MODEL_CARD.md dist/
  cd dist && zip -r ../models/isp-rectifier-models.zip . && cd ..
  echo ""
  echo "All-in-one zip: models/isp-rectifier-models.zip"
  ls -lh models/isp-rectifier-models.zip
  rm -rf dist
fi
