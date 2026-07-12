#!/usr/bin/env python3
"""
Validate model I/O shapes for all ONNX model variants
"""
import onnxruntime as ort
import numpy as np

# Validate all model variants
models = [
    'fusedispcontroller.onnx',
    'fusedispcontroller_int8.onnx',
    'fusedispcontroller_fp16.onnx',
    'fusedispcontroller_light.onnx',
    'fusedispcontroller_light_int8.onnx',
    'fusedispcontroller_medium.onnx',
    'fusedispcontroller_medium_int8.onnx',
]

print("Validating ONNX model I/O shapes...\n")

for name in models:
    try:
        path = f'models/{name}'
        sess = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
        inputs = {i.name: np.random.randn(*[d if isinstance(d, int) else 1 for d in i.shape]).astype(np.float32) for i in sess.get_inputs()}
        outputs = sess.run(None, inputs)
        print(f"✅ {name}:")
        print(f"    Inputs:  {[(i.name, i.shape) for i in sess.get_inputs()]}")
        print(f"    Outputs: {[(o.name, o.shape) for o in sess.get_outputs()]}")
    except Exception as e:
        print(f"❌ {name}: {e}")

print("\nDone!")