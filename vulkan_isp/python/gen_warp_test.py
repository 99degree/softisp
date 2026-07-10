#!/usr/bin/env python3
"""Generate a test ONNX model with GridSampler for Vulkan warp testing.

This model applies an identity warp (no motion) to verify GridSampler
works on the Vulkan backend without any custom ISP op.

Architecture: Input → GridSampler(identity_grid) → Output
Both are standard ONNX ops — no custom domain ops.
"""
import argparse
import numpy as np
import onnx
from onnx import helper, TensorProto

def build_warp_model(W, H):
    """Build a simple GridSampler model for identity warp testing."""
    nodes = []
    initializers = []

    # Input: [1, 3, H, W] RGB float32
    X = helper.make_tensor_value_info('input', TensorProto.FLOAT, [1, 3, H, W])

    # Output: [1, 3, H, W] RGB float32
    Y = helper.make_tensor_value_info('output', TensorProto.FLOAT, [1, 3, H, W])

    # Create identity grid: [1, H, W, 2] with (x, y) in [-1, 1]
    grid = np.zeros((1, H, W, 2), dtype=np.float32)
    for y in range(H):
        for x in range(W):
            grid[0, y, x, 0] = 2.0 * x / (W - 1) - 1.0  # x in [-1, 1]
            grid[0, y, x, 1] = 2.0 * y / (H - 1) - 1.0  # y in [-1, 1]

    initializers.append(helper.make_tensor(
        'grid', TensorProto.FLOAT, [1, H, W, 2], grid.tolist()))

    # GridSampler node (standard ONNX op_type 20)
    nodes.append(helper.make_node(
        'GridSampler',
        inputs=['input', 'grid'],
        outputs=['output'],
        mode=1,           # bilinear
        padding_mode=0,   # zeros
        align_corners=0,
    ))

    graph = helper.make_graph(
        nodes, 'warp_test', [X], [Y], initializer=initializers)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 16)])
    model.ir_version = 8
    return model


def main():
    p = argparse.ArgumentParser(description='Generate GridSampler warp test model')
    p.add_argument('--width', type=int, default=16)
    p.add_argument('--height', type=int, default=16)
    p.add_argument('-o', '--output', default='warp_test.onnx')
    args = p.parse_args()

    model = build_warp_model(args.width, args.height)
    onnx.save(model, args.output)
    print(f"Saved: {args.output}")
    print(f"Grid: {args.width}x{args.height}")

    try:
        from onnx import checker
        checker.check_model(model, full_check=True)
        print("ONNX check: PASS")
    except Exception as e:
        print(f"ONNX check: {e}")

    print(f"\nGraph: {len(model.graph.node)} nodes")
    for n in model.graph.node:
        ins = ', '.join(n.input)
        ous = ', '.join(n.output)
        print(f"  {n.op_type:13s}  {ins:30s} → {ous}")


if __name__ == '__main__':
    main()
