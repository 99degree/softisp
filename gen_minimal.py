# Minimal ONNX model: Cast → Conv(unpack) → Conv(ccm) → Output
import onnx
import numpy as np
from onnx import helper, TensorProto

BAYER_H, BAYER_W = 8, 8

inp = helper.make_tensor_value_info('input', TensorProto.FLOAT, [1, 1, BAYER_H, BAYER_W])
out = helper.make_tensor_value_info('output', TensorProto.FLOAT, [1, 3, BAYER_H//2, BAYER_W//2])

# Unpack: stride=2, k=2, 4ch
unpack_w = np.array([[[[1,0],[0,0]]],[[[0,1],[0,0]]],[[[0,0],[1,0]]],[[[0,0],[0,1]]]], dtype=np.float32)
unpack_b = np.zeros(4, dtype=np.float32)

# CCM: k=1, 4->3ch, identity-like but actually just extracts RGB
ccm_w = np.zeros((3,4,1,1), dtype=np.float32)
for i in range(3): ccm_w[i,i] = 1.0  # just take first 3 channels (bayer order: RGGB)
ccm_b = np.zeros(3, dtype=np.float32)

nodes = [
    helper.make_node('Conv', ['input', 'unpack_w', 'unpack_b'], ['rggb'],
                     kernel_shape=[2,2], strides=[2,2], pads=[0,0,0,0]),
    helper.make_node('Conv', ['rggb', 'ccm_w', 'ccm_b'], ['rgb'],
                     kernel_shape=[1,1], strides=[1,1], pads=[0,0,0,0]),
    helper.make_node('Identity', ['rgb'], ['output']),
]

init = [
    helper.make_tensor('unpack_w', TensorProto.FLOAT, [4,1,2,2], unpack_w.flatten().tolist()),
    helper.make_tensor('unpack_b', TensorProto.FLOAT, [4], unpack_b.tolist()),
    helper.make_tensor('ccm_w', TensorProto.FLOAT, [3,4,1,1], ccm_w.flatten().tolist()),
    helper.make_tensor('ccm_b', TensorProto.FLOAT, [3], ccm_b.tolist()),
]

graph = helper.make_graph(nodes, 'minimal_isp', [inp], [out], init)
model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 17)])
onnx.checker.check_model(model)
onnx.save(model, 'test_minimal.onnx')
print(f"Saved test_minimal.onnx")
