#!/usr/bin/env python3
"""
gen_isp_onnx.py — ⚠ DEPRECATED — Use gen_isp_onnx_standard.py instead

This generator creates custom 'isp.ai' domain ops. The MNN converter's
IspOnnxOps.cpp was deleted, so these models will segfault MNNConvert.

The standard-ops equivalent generates the same pipeline using only
standard ONNX ops (Conv, Mul, Add, Pow, Clip, etc.) and produces
identical output values. The fusion pass detects the standard ops and
fuses them into Extra ops automatically.

Usage (replacement):
  python gen_isp_onnx_standard.py \
    --bayer-width 3840 --bayer-height 2160 \
    -o pipeline.onnx

Requirements:
  pip install onnx protobuf numpy
"""

import argparse
import numpy as np
import struct
import sys
import os

try:
    import onnx
    from onnx import helper, TensorProto
    from onnx import AttributeProto
except ImportError:
    print("Install onnx: pip install onnx protobuf numpy")
    sys.exit(1)

# ── Shader embedding (load from .spv files) ────────────────────────
SPV_DIR = os.path.dirname(os.path.abspath(__file__))

def load_spv(name):
    with open(os.path.join(SPV_DIR, name), 'rb') as f:
        return f.read()

SHADERS = {
    'unpack_blc':      load_spv('shader1_unpack_blc.spv'),
    'demosaic_noscale':load_spv('isp_opsets/demosaic_noscale.spv'),
    'fcs':             load_spv('shader3_fcs.spv'),
    'ee':              load_spv('shader4_ee.spv'),
    'ldci':            load_spv('shader5_ldci.spv'),
    'display':         load_spv('shader6_display_simple.spv'),
    'fcs_display':     load_spv('shader_fcs_display_fused.spv'),
    'ee_ldci':         load_spv('shader_ee_ldci_fused.spv'),
}

# ── Custom opset: isp.ai ──────────────────────────────────────────
DOMAIN = 'isp.ai'
OPSET_VERSION = 1

def make_op(node_name, op_type, output_shape, global_size, local_size,
            uniforms, spirv, inputs=['input']):
    """Create an ONNX node for an ISP operation."""
    # Build attributes matching the Extra op schema
    # We manually create AttrProto because make_node kwargs don't support raw bytes
    from onnx import AttributeProto
    
    def make_attr(name, dtype, value):
        attr = AttributeProto()
        attr.name = name
        attr.type = dtype
        if dtype == AttributeProto.INTS:
            attr.ints.extend(value)
        elif dtype == AttributeProto.FLOATS:
            attr.floats.extend(value)
        elif dtype == AttributeProto.INT:
            attr.i = value
        elif dtype == AttributeProto.FLOAT:
            attr.f = value
        elif dtype == AttributeProto.TENSOR:
            attr.t.CopyFrom(value)
        elif dtype == AttributeProto.STRING:
            attr.s = value if isinstance(value, bytes) else value.encode()
        return attr

    # Create the node
    from onnx import NodeProto
    node = NodeProto()
    node.name = node_name
    node.op_type = op_type
    node.domain = DOMAIN
    node.input.extend(inputs)
    node.output.extend([node_name])
    # Add attributes
    node.attribute.extend([
        make_attr('output_shape', AttributeProto.INTS, output_shape),
        make_attr('global_size', AttributeProto.INTS, global_size),
        make_attr('group_size', AttributeProto.INTS, local_size),
        make_attr('optimized_dispatch', AttributeProto.INT, 1),
        make_attr('const_floats', AttributeProto.FLOATS, uniforms),
        make_attr('spirv', AttributeProto.STRING, spirv),
    ])
    return node

def build_pipeline(args):
    """Build the ISP pipeline ONNX model."""
    BW, BH = args.bayer_width, args.bayer_height
    FW, FH = BW // 2, BH // 2  # unpack downsamples

    # ── Create graph input ──
    graph_input = helper.make_tensor_value_info(
        'bayer_input', TensorProto.INT32, [1, 1, BH, BW])

    # ── Build stages ──
    nodes = []
    prev = 'bayer_input'

    if args.fused:
        # Fused 4-stage: unpack + demosaic + fcs_display + ee_ldci
        stages = [
            ('unpack',     'UnpackBlc',       [1,4,FH,FW], [FW,FH,1], [16,16,1],
             [float(BW),float(BH),float(FW),float(FH),1023.0,0,0,0,0,1,1,1,1],
             SHADERS['unpack_blc']),
            ('demosaic',   'DemosaicNoscale', [1,3,FH,FW], [FW,FH,1], [16,16,1],
             [float(FW),float(FH),1.0, 1,0,0, 0,1,0, 0,0,1, 0,0,0,0],
             SHADERS['demosaic_noscale']),
            ('fcs_display','FcsDisplay',      [1,3,FH,FW], [FW,FH,1], [16,16,1],
             [float(FW),float(FH), 1.0, 0.0, 2.2, 0.0, 0,0,0],
             SHADERS['fcs_display']),
            ('ee_ldci',    'EeLdci',          [1,3,FH,FW], [FW,FH,1], [16,16,1],
             [float(FW),float(FH), 0.5, 0.01, 0.5, 1.0, 0,0],
             SHADERS['ee_ldci']),
        ]
    else:
        # Standard 6-stage
        stages = [
            ('unpack',      'UnpackBlc',       [1,4,FH,FW], [FW,FH,1], [16,16,1],
             [float(BW),float(BH),float(FW),float(FH),1023.0,0,0,0,0,1,1,1,1],
             SHADERS['unpack_blc']),
            ('demosaic',    'DemosaicNoscale', [1,3,FH,FW], [FW,FH,1], [16,16,1],
             [float(FW),float(FH),1.0, 1,0,0, 0,1,0, 0,0,1, 0,0,0,0],
             SHADERS['demosaic_noscale']),
            ('fcs',         'Fcs',             [1,3,FH,FW], [FW,FH,1], [16,16,1],
             [float(FW),float(FH), 1.0, 0.0, 0.0, 0,0,0,0],
             SHADERS['fcs']),
            ('ee',          'Ee',              [1,3,FH,FW], [FW,FH,1], [16,16,1],
             [float(FW),float(FH), 0.5, 0.01, 0,0,0,0],
             SHADERS['ee']),
            ('ldci',        'Ldci',            [1,3,FH,FW], [FW,FH,1], [16,16,1],
             [float(FW),float(FH), 0.5, 1.0, 0,0,0,0],
             SHADERS['ldci']),
            ('display',     'Display',         [1,3,FH,FW], [FW,FH,1], [16,16,1],
             [float(FW),float(FH), 0.0, 1.0, 1.0, 2.2, 0,0],
             SHADERS['display']),
        ]

    for name, optype, shape, gs, ls, uniforms, spirv in stages:
        node = make_op(name, optype, shape, gs, ls, uniforms, spirv, [prev])
        nodes.append(node)
        prev = name

    # ── Graph output ──
    graph_output = helper.make_tensor_value_info(
        prev, TensorProto.FLOAT, stages[-1][2])

    # ── Create graph ──
    graph = helper.make_graph(
        nodes,
        'isp_pipeline',
        [graph_input],
        [graph_output],
    )

    # ── Create model ──
    model = helper.make_model(
        graph,
        opset_imports=[
            helper.make_opsetid('', 17),          # standard ONNX opset
            helper.make_opsetid(DOMAIN, OPSET_VERSION),  # isp.ai domain
        ],
        producer_name='isp_onnx',
        producer_version='1.0',
    )

    return model

def main():
    parser = argparse.ArgumentParser(description='Generate ISP pipeline ONNX model')
    parser.add_argument('--fused', action='store_true',
                       help='Generate fused 4-stage model')
    parser.add_argument('--bayer-width', type=int, default=3840,
                       help='Input Bayer width (default: 3840)')
    parser.add_argument('--bayer-height', type=int, default=2160,
                       help='Input Bayer height (default: 2160)')
    parser.add_argument('-o', '--output', default=None,
                       help='Output .onnx path')
    args = parser.parse_args()

    model = build_pipeline(args)

    if args.output is None:
        suffix = '_fused' if args.fused else ''
        args.output = f'isp_pipeline{suffix}.onnx'

    onnx.save(model, args.output)
    print(f'ONNX model saved: {args.output}')
    print(f'  Input:  INT32[1,1,{args.bayer_height},{args.bayer_width}] Bayer')
    print(f'  Output: FLOAT[1,3,{args.bayer_height//2},{args.bayer_width//2}] RGB')
    print(f'  Stages: {"fused 4-stage" if args.fused else "standard 6-stage"}')
    print(f'  File:   {os.path.getsize(args.output)} bytes')

    # Validate
    try:
        onnx.checker.check_model(model)
        print('  ONNX check: PASS')
    except Exception as e:
        print(f'  ONNX check: WARN — {e}')

    print(f'\nConvert to MNN:')
    print(f'  MNNConvert -f ONNX --modelFile {args.output} \\')
    print(f'    --MNNModel isp_pipeline.mnn --bizCode isp')

if __name__ == '__main__':
    main()
