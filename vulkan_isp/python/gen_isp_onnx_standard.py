#!/usr/bin/env python3
"""
gen_isp_onnx_standard.py — ISP pipeline using ONLY standard ONNX ops.
Each stage maps cleanly to standard operators for MNN converter fusion.

Stages → ONNX ops:
  UnpackBlc:   Cast(→F32) + Conv(2×2,stride2,4ch)
  DemosaicCCM: Conv(1×1,4→3ch)
  FCS:         Mul(scale) + Add(bias)
  EE:          Conv(3×3,unsharp,group=3)
  LDCI:        AveragePool(3×3) + Sub + Mul + Add
  Display:     Pow(exp=1/2.4) + Clip(0,1)

Usage:
  python gen_isp_onnx_standard.py -o pipeline.onnx --bayer-width 8 --bayer-height 8

Requirements: pip install onnx protobuf numpy
"""

import argparse
import numpy as np
try:
    import onnx
    from onnx import helper, TensorProto, checker
    from onnx.helper import make_node, make_tensor_value_info, make_graph, make_model, make_tensor
except ImportError:
    print("Install: pip install onnx protobuf numpy")
    raise

# ── build_graph ──────────────────────────────────────────────────
def build_graph(BW, BH, no_unpack=False, bayer_pattern='RGGB'):
    """Build ISP pipeline using ONLY standard ONNX ops. Returns ModelProto."""
    FW, FH = BW // 2, BH // 2

    nodes = []
    initializers = []
    next_tid = 0

    def new_name(prefix):
        nonlocal next_tid
        n = f"{prefix}_{next_tid}"
        next_tid += 1
        return n

    # ── Helper: add initializer tensor ──
    def init_tensor(name, data):
        """Add a named initializer from numpy array."""
        t = make_tensor(name, TensorProto.FLOAT, list(data.shape), data.tobytes(), raw=True)
        initializers.append(t)

    # ── Helper: add node ──
    prev = 'input'

    def add_node(op_type, name, inputs, outputs=None, **kwargs):
        nonlocal prev
        out_name = name if outputs is None else outputs
        node = make_node(op_type, inputs=inputs, outputs=[out_name], name=name, **kwargs)
        nodes.append(node)
        prev = out_name
        return out_name

    # ═══════════════════════════════════════════════
    #  Stage 1: UnpackBlc — Bayer decode + normalize
    # ═══════════════════════════════════════════════
    if not no_unpack:
        # Cast INT32 → FLOAT (if input is int)
        add_node('Cast', 'bayer_f32', [prev], to=TensorProto.FLOAT)

        # Conv(2×2, stride=2, 4ch) — extracts Bayer quadrants
        # Bayer pattern determines which spatial position maps to R/Gr/Gb/B
        pattern_map = {
            'RGGB': [(0,0,'R'), (0,1,'Gr'), (1,0,'Gb'), (1,1,'B')],
            'GRBG': [(0,1,'R'), (0,0,'Gr'), (1,1,'Gb'), (1,0,'B')],
            'GBRG': [(1,0,'R'), (0,0,'Gr'), (1,1,'Gb'), (0,1,'B')],
            'BGGR': [(1,1,'R'), (0,1,'Gr'), (1,0,'Gb'), (0,0,'B')],
        }
        if bayer_pattern not in pattern_map:
            raise ValueError(f"Unknown Bayer pattern: {bayer_pattern}. Use RGGB, GRBG, GBRG, or BGGR.")

        W_UNPACK = np.zeros((4, 1, 2, 2), dtype=np.float32)
        for ch, (r, c, _name) in enumerate(pattern_map[bayer_pattern]):
            W_UNPACK[ch, 0, r, c] = 1.0
        B_UNPACK = np.zeros(4, dtype=np.float32)

        init_tensor('unpack_w', W_UNPACK)
        init_tensor('unpack_b', B_UNPACK)
        add_node('Conv', 'bayer_4ch', [prev, 'unpack_w', 'unpack_b'],
                 strides=[2,2], kernel_shape=[2,2], pads=[0,0,0,0])

    # ═══════════════════════════════════════════════
    #  Stage 2: Demosaic + CCM  (Conv 1×1, 4ch→3ch)
    # ═══════════════════════════════════════════════
    # CCM identity: R=R, G=(Gr+Gb)/2, B=B
    W_CCM = np.zeros((3, 4, 1, 1), dtype=np.float32)
    W_CCM[0, 0] = 1.0      # R ← ch0
    W_CCM[1, 1] = 0.5      # G ← ch1*0.5 + ch2*0.5
    W_CCM[1, 2] = 0.5
    W_CCM[2, 3] = 1.0      # B ← ch3
    B_CCM = np.zeros(3, dtype=np.float32)

    init_tensor('ccm_w', W_CCM)
    init_tensor('ccm_b', B_CCM)
    add_node('Conv', 'demosaic', [prev, 'ccm_w', 'ccm_b'],
             strides=[1,1], kernel_shape=[1,1], pads=[0,0,0,0])

    # ═══════════════════════════════════════════════
    #  Stage 3: FCS — Film Contrast Strength
    # ═══════════════════════════════════════════════
    FCS_SCALE = np.array([[[[1.0]], [[1.0]], [[1.0]]]], dtype=np.float32)  # 1×3×1×1
    FCS_BIAS  = np.zeros((1, 3, 1, 1), dtype=np.float32)

    init_tensor('fcs_scale', FCS_SCALE)
    init_tensor('fcs_bias',  FCS_BIAS)
    add_node('Mul', 'fcs_mul',  [prev, 'fcs_scale'])
    add_node('Add', 'fcs',      [prev, 'fcs_bias'])

    # ═══════════════════════════════════════════════
    #  Stage 4: EE — Edge Enhancement (Conv 3×3 unsharp)
    # ═══════════════════════════════════════════════
    # Unsharp mask kernel per channel: [[0,-.5,0],[-.5,3,-.5],[0,-.5,0]]
    W_EE = np.zeros((3, 1, 3, 3), dtype=np.float32)
    k = np.array([[0, -0.5, 0], [-0.5, 3.0, -0.5], [0, -0.5, 0]], dtype=np.float32)
    for c in range(3):
        W_EE[c, 0] = k
    B_EE = np.zeros(3, dtype=np.float32)

    init_tensor('ee_w', W_EE)
    init_tensor('ee_b', B_EE)
    add_node('Conv', 'ee', [prev, 'ee_w', 'ee_b'],
             strides=[1,1], kernel_shape=[3,3], pads=[1,1,1,1], group=3)

    # ═══════════════════════════════════════════════
    #  Stage 5: LDCI — Local Dynamic Contrast
    # ═══════════════════════════════════════════════
    # Save EE output before LDCI chain (needed for Sub and Add inputs)
    ee_out = prev

    # blur = AveragePool(3×3)
    add_node('AveragePool', 'ldci_blur', [prev],
             kernel_shape=[3,3], strides=[1,1], pads=[1,1,1,1])

    # diff = blur - center
    add_node('Sub', 'ldci_diff', ['ldci_blur', ee_out])

    # amp = diff * strength
    LDCI_STR = np.ones((1, 3, 1, 1), dtype=np.float32) * 0.5
    init_tensor('ldci_str', LDCI_STR)
    add_node('Mul', 'ldci_amp', ['ldci_diff', 'ldci_str'])

    # result = center + amp
    add_node('Add', 'ldci', [ee_out, 'ldci_amp'])

    # ═══════════════════════════════════════════════
    #  Stage 6: Display — sRGB gamma
    # ═══════════════════════════════════════════════
    GAMMA_EXP = np.array([1.0/2.4], dtype=np.float32).reshape(1, 1, 1, 1)
    init_tensor('gamma_exp', GAMMA_EXP)
    add_node('Pow', 'display_gamma', [prev, 'gamma_exp'])

    # Clip to [0,1]
    CLIP_MIN = np.array([0.0], dtype=np.float32)
    CLIP_MAX = np.array([1.0], dtype=np.float32)
    init_tensor('clip_min', CLIP_MIN)
    init_tensor('clip_max', CLIP_MAX)

    # Clip op with min/max as 2nd and 3rd inputs
    add_node('Clip', 'output', [prev, 'clip_min', 'clip_max'])

    # ── Graph ──
    graph_input = make_tensor_value_info('input', TensorProto.FLOAT, [1, 1, BH, BW])
    graph_output = make_tensor_value_info('output', TensorProto.FLOAT, [1, 3, FH, FW])
    graph = make_graph(nodes, 'isp_pipeline_standard',
                       [graph_input], [graph_output], initializer=initializers)

    model = make_model(graph,
        opset_imports=[helper.make_opsetid('', 17)],
        producer_name='isp_onnx_standard', producer_version='1.0')
    model.ir_version = onnx.IR_VERSION_2024_3_25  # latest available
    return model


# ── main ──────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser()
    p.add_argument('--no-unpack', action='store_true')
    p.add_argument('--bayer-width',  type=int, default=8)
    p.add_argument('--bayer-height', type=int, default=8)
    p.add_argument('--bayer-pattern', type=str, default='RGGB',
                    choices=['RGGB', 'GRBG', 'GBRG', 'BGGR'],
                    help='Bayer CFA pattern (default: RGGB)')
    p.add_argument('-o', '--output', default='isp_pipeline_standard.onnx')
    args = p.parse_args()

    BW, BH = args.bayer_width, args.bayer_height
    model = build_graph(BW, BH, no_unpack=args.no_unpack, bayer_pattern=args.bayer_pattern)
    onnx.save(model, args.output)
    print(f"Saved: {args.output}")

    try:
        checker.check_model(model, full_check=True)
        print("ONNX check: PASS")
    except Exception as e:
        print(f"ONNX check: {e}")

    # Print graph summary
    print(f"\nGraph: {len(model.graph.node)} nodes, {len(model.graph.initializer)} initializers")
    for n in model.graph.node:
        ins = ', '.join(n.input)
        ous = ', '.join(n.output)
        print(f"  {n.op_type:13s}  {ins:30s} → {ous}")

if __name__ == '__main__':
    main()
