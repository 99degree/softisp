#!/usr/bin/env python3
"""
gen_isp_onnx_standard.py — ISP pipeline using ONLY standard ONNX ops.

Each ISP stage mapped to standard ONNX ops:
  UnpackBlc:  Conv(stride=2, kernel=2, 4ch) + arithmetic (approximation)
  Demosaic:   Conv(1×1, CCM matrix, 4ch→3ch)
  FCS:        Mul
  EE:         Conv(3×3, Laplacian kernel)
  LDCI:       AveragePool(3×3) + Sub + Mul + Add
  Display:    Pow (sRGB gamma)

Usage:
  python gen_isp_onnx_standard.py               # Full 6-stage
  python gen_isp_onnx_standard.py --no-unpack   # Skip unpack (RGGB input)

Requirements:
  pip install onnx protobuf numpy
"""

import argparse
import numpy as np
import os, sys
try:
    import onnx
    from onnx import helper, TensorProto, OperatorSetIdProto
    from onnx import AttributeProto, NodeProto, GraphProto, ModelProto
    from onnx.helper import make_node, make_tensor_value_info, make_graph, make_model, make_tensor
except ImportError:
    print("Install: pip install onnx protobuf numpy")
    sys.exit(1)

# ── ISP Constants ──
SENSOR_MAX = 1023.0

def build_graph(args):
    """Build ISP pipeline graph using only standard ONNX ops."""
    BW, BH = args.bayer_width, args.bayer_height
    FW, FH = BW // 2, BH // 2

    nodes = []
    prev = 'input'  # Bayer: INT32(1,1,BH,BW)

    # ── Helper: add node ──
    def add(op_type, name, inputs, **kwargs):
        nonlocal prev
        node = make_node(op_type, inputs=inputs, outputs=[name], name=name, **kwargs)
        nodes.append(node)
        prev = name
        return name

    # ── Stage 1: UnpackBlc (Bayer decode + normalize) ──
    if not args.no_unpack:
        # Strategy: Conv(2×2, stride=2, 4 output channels) extracts Bayer quads
        # Each output channel picks one of the 4 Bayer positions
        w_unpack = np.zeros((4, 1, 2, 2), dtype=np.float32)
        # R at (0,0), Gr at (0,1), Gb at (1,0), B at (1,1)
        w_unpack[0, 0, 0, 0] = 1.0  # R
        w_unpack[1, 0, 0, 1] = 1.0  # Gr
        w_unpack[2, 0, 1, 0] = 1.0  # Gb
        w_unpack[3, 0, 1, 1] = 1.0  # B
        b_unpack = np.zeros(4, dtype=np.float32)

        # Cast INT32 → FLOAT first
        if args.cast:
            add('Cast', 'bayer_f32', ['input'],
                to=getattr(TensorProto, 'FLOAT'))

        # Conv extracts 4 channels at half res
        add('Conv', 'bayer_4ch', ['bayer_f32' if args.cast else 'input'],
            strides=[2,2], kernel_shape=[2,2], pads=[0,0,0,0],
            dilations=[1,1], group=1)
        # Set weights via initializer
        # ... (ONNX needs weights as initializer tensors)

    # ── Stage 2: Demosaic + CCM (Conv 1×1, 4ch→3ch) ──
    # CCM matrix: identity (no color transform)
    # 3×4 kernel: row0 picks R, row1 averages G's, row2 picks B
    w_ccm = np.zeros((3, 4, 1, 1), dtype=np.float32)
    w_ccm[0, 0, 0, 0] = 1.0       # R output from ch0
    w_ccm[1, 1, 0, 0] = 0.5       # G from Gr
    w_ccm[1, 2, 0, 0] = 0.5       # G from Gb
    w_ccm[2, 3, 0, 0] = 1.0       # B from ch3
    b_ccm = np.zeros(3, dtype=np.float32)

    add('Conv', 'demosaic', [prev],
        strides=[1,1], kernel_shape=[1,1], pads=[0,0,0,0])

    # ── Stage 3: FCS (scale) — element-wise Mul ──
    # FCS = input * strength + offset. Use Mul + Add
    # For strength=1.0, offset=0: identity
    # Represent as Mul(1.0) + Add(0)
    # But to detect pattern: Mul + Add → fuse into FCS
    fcs_scale = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    fcs_bias = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    add('Mul', 'fcs_mul', [prev, 'fcs_scale'])
    add('Add', 'fcs', [prev, 'fcs_bias'])

    # ── Stage 4: EE (Laplacian edge enhance) — Conv 3×3 ──
    # Laplacian: L = [[0,-1,0],[-1,4,-1],[0,-1,0]]
    # output = center + 0.5*L(center) = 3*center - 0.5*(t+b+l+r)
    # As Conv: kernel = identity + 0.5*Laplacian
    # For each channel independently:
    w_ee = np.zeros((3, 3, 3, 3), dtype=np.float32)
    for c in range(3):
        # Unsharp mask kernel
        w_ee[c, c, :, :] = np.array([
            [0, -0.5, 0],
            [-0.5, 3.0, -0.5],
            [0, -0.5, 0]
        ], dtype=np.float32)
    b_ee = np.zeros(3, dtype=np.float32)

    add('Conv', 'ee', [prev],
        strides=[1,1], kernel_shape=[3,3],
        pads=[1,1,1,1], dilations=[1,1], group=3)

    # ── Stage 5: LDCI (local contrast) — AveragePool + Sub + Mul + Add ──
    # Step 1: AveragePool 3×3 (blur)
    add('AveragePool', 'ldci_blur', [prev],
        kernel_shape=[3,3], strides=[1,1], pads=[1,1,1,1])

    # Step 2: local_mean - center (diff)
    add('Sub', 'ldci_diff', ['ldci_blur', prev])

    # Step 3: diff * strength
    ldci_str = np.array([0.5, 0.5, 0.5], dtype=np.float32)
    add('Mul', 'ldci_amp', ['ldci_diff', 'ldci_str'])

    # Step 4: center + amplified_diff
    add('Add', 'ldci', [prev, 'ldci_amp'])

    # ── Stage 6: Display (gamma) — Pow ──
    # sRGB: gamma=2.2 → power = 1/2.4 if val > 0.0031308
    # Approximate with simple Pow
    gamma_exp = np.array([1.0/2.4], dtype=np.float32)
    add('Pow', 'display_gamma', [prev, 'gamma_exp'])

    # Clip to [0,1]
    zero_t = np.array([0.0], dtype=np.float32)
    one_t = np.array([1.0], dtype=np.float32)
    add('Clip', 'output', [prev, zero_t, one_t])

    # ── Graph ──
    graph_input = make_tensor_value_info(
        'input', TensorProto.FLOAT, [1, 1, BH, BW])

    graph_output = make_tensor_value_info(
        'output', TensorProto.FLOAT, [1, 3, FH, FW])

    # Create graph
    graph = make_graph(nodes, 'isp_pipeline_standard',
                       [graph_input], [graph_output])

    # Add weight initializers
    # (TODO: add initializer tensors for Conv weights, scale, etc.)

    model = make_model(graph,
        opset_imports=[helper.make_opsetid('', 17)],
        producer_name='isp_onnx_standard',
        producer_version='1.0')

    return model

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-unpack', action='store_true',
                       help='Skip unpack stage (input=4ch RGGB float)')
    parser.add_argument('--cast', action='store_true',
                       help='Insert Cast INT32→FLOAT before unpack')
    parser.add_argument('--bayer-width', type=int, default=3840)
    parser.add_argument('--bayer-height', type=int, default=2160)
    parser.add_argument('-o', '--output', default=None)
    args = parser.parse_args()

    model = build_graph(args)
    suffix = '_standard.onnx'
    out = args.output or f'isp_pipeline{suffix}'
    onnx.save(model, out)
    print(f'Saved: {out}')
    try:
        onnx.checker.check_model(model)
        print('ONNX check: PASS')
    except Exception as e:
        print(f'ONNX check: {e}')

if __name__ == '__main__':
    main()
