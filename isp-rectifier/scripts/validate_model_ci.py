#!/usr/bin/env python3
"""
Quick model validation script for CI
"""
import onnxruntime as ort
import numpy as np

# Quick model load test
sess = ort.InferenceSession('models/fusedispcontroller_int8.onnx')
print(f'Model loaded: {sess.get_inputs()[0].name} -> {sess.get_outputs()[0].name}')

# Quick inference test
hist = np.random.randn(1, 256).astype(np.float32)
meta = np.random.randn(1, 52).astype(np.float32)
out = ort.InferenceSession('models/fusedispcontroller_int8.onnx').run(None, {
    'histogram': hist, 'metadata': meta
})
print(f'Outputs: {[o.shape for o in out]}')