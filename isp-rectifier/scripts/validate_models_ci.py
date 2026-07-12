#!/usr/bin/env python3
"""
Extended model validation for CI
"""
import onnx
import onnxruntime as ort
import numpy as np

models = [
    'fusedispcontroller.onnx',
    'fusedispcontroller_int8.onnx',
    'fusedispcontroller_fp16.onnx',
    'fusedispcontroller_light.onnx',
    'fusedispcontroller_light_int8.onnx',
    'fusedispcontroller_medium.onnx',
    'fusedispcontroller_medium_int8.onnx',
]

for model_name in ['fusedispcontroller_int8.onnx']:
    path = f'isp-rectifier/models/{model_name}'
    print(f'\nValidating {model_name}...')

    # Load and check
    import onnx
    model = onnx.load(f'isp-rectifier/models/{model_name}')
    import onnx
    onnx.checker.check_model(model)
    print('  ONNX checker passed')

    # Quick inference test
    import onnxruntime as ort
    sess = ort.InferenceSession(f'isp-rectifier/models/{model_name}', providers=['CPUExecutionProvider'])
    input_names = [i.name for i in ort.InferenceSession(f'isp-rectifier/models/{model_name}').get_inputs()]
    output_names = [o.name for o in ort.InferenceSession(f'isp-rectifier/models/{model_name}').get_outputs()]

    # Quick inference
    hist = np.random.randn(1, 256).astype(np.float32)
    meta = np.random.randn(1, 52).astype(np.float32)
    sess = ort.InferenceSession(f'isp-rectifier/models/{model_name}', providers=['CPUExecutionProvider'])
    outputs = sess.run(None, {'histogram': np.random.randn(1, 256).astype(np.float32), 'metadata': np.random.randn(1, 52).astype(np.float32)})

    print(f'  Inputs: {input_names}')
    print(f'  Outputs: {output_names}')
    print(f'  Output shapes: {[o.shape for o in outputs]}')
    print(f'  Model validation passed!')