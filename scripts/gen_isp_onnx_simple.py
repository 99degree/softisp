import onnx
import numpy as np
from onnx import helper, TensorProto

BAYER_H = 8
BAYER_W = 8

input_tensor = helper.make_tensor_value_info('input', TensorProto.FLOAT, [1, 1, BAYER_H, BAYER_W])
output_tensor = helper.make_tensor_value_info('output', TensorProto.FLOAT, [1, 3, BAYER_H//2, BAYER_W//2])

# Unpack Conv: 2x2 stride 2, 4 output channels
unpack_w = np.array([[[[1,0],[0,0]]], [[[0,1],[0,0]]], [[[0,0],[1,0]]], [[[0,0],[0,1]]]], dtype=np.float32)
unpack_b = np.zeros(4, dtype=np.float32)
unpack_w_init = helper.make_tensor('unpack_w', TensorProto.FLOAT, [4,1,2,2], unpack_w.flatten().tolist())
unpack_b_init = helper.make_tensor('unpack_b', TensorProto.FLOAT, [4], unpack_b.tolist())

# CCM: 1x1 4->3 identity
ccm_w = np.zeros((3,4,1,1), dtype=np.float32)
for i in range(3): ccm_w[i,i] = 1.0
ccm_b = np.zeros(3, dtype=np.float32)
ccm_w_init = helper.make_tensor('ccm_w', TensorProto.FLOAT, [3,4,1,1], ccm_w.flatten().tolist())
ccm_b_init = helper.make_tensor('ccm_b', TensorProto.FLOAT, [3], ccm_b.tolist())

# Gamma
gamma_val = 1.0 / 2.4
gamma_init = helper.make_tensor('gamma', TensorProto.FLOAT, [], [gamma_val])

nodes = [
    helper.make_node('Conv', ['input', 'unpack_w', 'unpack_b'], ['unpack'],
                     kernel_shape=[2,2], strides=[2,2], pads=[0,0,0,0], name='unpack'),
    helper.make_node('Conv', ['unpack', 'ccm_w', 'ccm_b'], ['rgb'],
                     kernel_shape=[1,1], strides=[1,1], pads=[0,0,0,0], name='ccm'),
    helper.make_node('Pow', ['rgb', 'gamma'], ['output'], name='gamma'),
]

graph = helper.make_graph(nodes, 'isp_pipeline', [input_tensor], [output_tensor],
                          [unpack_w_init, unpack_b_init, ccm_w_init, ccm_b_init, gamma_init])
model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 17)])
onnx.checker.check_model(model)
onnx.save(model, 'test_isp_simple.onnx')
print(f"Saved test_isp_simple.onnx ({len(nodes)} nodes)")
