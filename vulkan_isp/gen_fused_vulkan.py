#!/usr/bin/env python3
"""
Generate MNN model with fused ISP Vulkan shaders.
Supports two modes:
- atomic: One shader per Extra op (simple, debuggable)
- fused: Multiple stages in one Extra op (reduced dispatch)
"""

import subprocess
import sys
import os

sys.path.insert(0, '/data/data/com.termux/files/home/MNN')
import flatbuffers
from MNN.Blob import (BlobStart, BlobEnd, BlobAddInt8s, BlobAddFloat32s, BlobAddInt32s,
                       BlobStartInt8sVector, BlobStartFloat32sVector, BlobStartInt32sVector)
from MNN.Attribute import AttributeStart, AttributeEnd, AttributeAddKey, AttributeAddI, AttributeAddTensor, AttributeAddB, AttributeAddList
from MNN.ListValue import ListValueStart, ListValueEnd, ListValueAddI
from MNN.Extra import ExtraStart, ExtraEnd, ExtraAddType, ExtraAddAttr, ExtraStartAttrVector
from MNN.Op import OpStart, OpEnd, OpAddType, OpAddMainType, OpAddMain, OpAddName, OpStartInputIndexesVector, OpStartOutputIndexesVector, OpAddInputIndexes, OpAddOutputIndexes
from MNN.Net import NetStart, NetEnd, NetStartOplistsVector, NetStartTensorNameVector, NetAddOplists, NetAddTensorName, NetStartExtraTensorDescribeVector, NetAddExtraTensorDescribe, NetAddBizCode
from MNN.OpType import OpType
from MNN.OpParameter import OpParameter
from MNN.TensorDescribe import TensorDescribeStart, TensorDescribeEnd, TensorDescribeAddBlob, TensorDescribeAddIndex, TensorDescribeAddName
from MNN.Input import InputStart, InputEnd, InputAddDtype, InputAddDformat, InputStartDimsVector
from MNN.DataType import DataType
from MNN.MNN_DATA_FORMAT import MNN_DATA_FORMAT

# Shader definitions
SHADERS = [
    # (name, filename, input_shape, output_shape)
    ('unpack_blc', 'shader1_unpack_blc',   [1, 4, 1080, 1920],  [1, 4, 1080, 1920]),
    ('demosaic',   'shader2_demosaic_ccm',   [1, 4, 1080, 1920],  [1, 3, 1080, 1920]),
    ('fcs',        'shader3_fcs',            [1, 3, 1080, 1920],  [1, 3, 1080, 1920]),
    ('ee',         'shader4_ee',             [1, 3, 1080, 1920],  [1, 3, 1080, 1920]),
    ('ldci',       'shader5_ldci',          [1, 3, 1080, 1920],  [1, 3, 1080, 1920]),
    ('display',    'shader6_display',       [1, 3, 1080, 1920],  [1, 3, 540, 960]),
]

def compile_shader(name, use_fp16=False):
    """Compile GLSL to SPIR-V"""
    base = f'/data/data/com.termux/files/home/softisp/vulkan_isp/{name}'
    src = f'{base}.comp'
    spv = f'{base}.spv'
    
    cmd = ['/data/data/com.termux/files/usr/bin/glslang', '-V', src, '-o', spv]
    if use_fp16:
        cmd.extend(['-D', 'USE_FP16=1'])
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Failed: {result.stderr}")
        return None
    
    with open(spv, 'rb') as f:
        return f.read()


def create_tensor_blob(builder, dims):
    """Create a Blob with specified dimensions"""
    BlobStartInt32sVector(builder, len(dims))
    for v in reversed(dims):
        builder.PrependInt32(v)
    dims_vec = builder.EndVector()
    
    BlobStart(builder)
    BlobAddInt32s(builder, dims_vec)
    return BlobEnd(builder)


def create_string(builder, s):
    """Create flatbuffers string"""
    return builder.CreateString(s)


def build_atomic_model(shader_dir='/data/data/com.termux/files/home/softisp/vulkan_isp'):
    """Build model with one shader per Extra op (atomic mode)"""
    builder = flatbuffers.Builder(1024 * 1024)
    
    # Tensor layout:
    # 0: input Bayer [1,1,2160,3840] INT16
    # 1-4: unpack output [1,1080,1920] x4 (r,g1,g2,b)
    # 5: demosaic output [1,3,1080,1920]
    # 6: fcs output
    # 7: ee output
    # 8: ldci output
    # 9: display output [1,3,540,960]
    
    INPUT_W, INPUT_H = 3840, 2160
    W, H = 1920, 1080
    W2, H2 = 960, 540
    
    tensor_names = [f'tensor_{i}' for i in range(10)]
    
    # Pre-compile shaders
    shader_data = {}
    for name, filename, in_shape, out_shape in SHADERS:
        spv = compile_shader(filename)
        if spv:
            shader_data[name] = spv
            print(f"  {name}: {len(spv)} bytes")
    
    # Create strings
    strings = {s: create_string(builder, s) for s in list(set([n for n, *_ in SHADERS])) + ['FusedISP', 'FusedISPNet'] + tensor_names}
    
    # Input op
    InputStartDimsVector(builder, 4)
    for v in [1, 1, INPUT_H, INPUT_W]:
        builder.PrependInt32(v)
    input_dims_vec = builder.EndVector()
    
    InputStart(builder)
    InputAddDtype(builder, DataType.DT_INT16)
    InputAddDformat(builder, MNN_DATA_FORMAT.NCHW)
    InputAddDims(builder, input_dims_vec)
    input_param = InputEnd(builder)
    
    OpStartInputIndexesVector(builder, 0)
    in0_vec = builder.EndVector()
    OpStartOutputIndexesVector(builder, 1)
    builder.PrependInt32(0)
    out0_vec = builder.EndVector()
    
    OpStart(builder)
    OpAddType(builder, OpType.Input)
    OpAddMainType(builder, OpParameter.Input)
    OpAddMain(builder, input_param)
    OpAddName(builder, strings['input'])
    OpAddInputIndexes(builder, in0_vec)
    OpAddOutputIndexes(builder, out0_vec)
    ops = [OpEnd(builder)]
    
    # Build Extra ops
    tensor_idx = 1  # Next available tensor index
    input_idx_map = [0, 1, 2, 3, 4, 5, 6, 7, 8]  # Input tensor for each shader
    output_idx_map = [1, 2, 3, 4, 5, 6, 7, 8, 9]  # Output tensor for each shader
    
    for i, (name, filename, in_shape, out_shape) in enumerate(SHADERS):
        spv = shader_data.get(name)
        if not spv:
            continue
            
        in_idx = input_idx_map[i]
        out_idx = output_idx_map[i]
        
        # SPIR-V blob
        BlobStartInt8sVector(builder, len(spv))
        for b in reversed(spv):
            builder.PrependByte(b)
        spirv_vec = builder.EndVector()
        
        BlobStart(builder)
        BlobAddInt8s(builder, spirv_vec)
        spirv_blob = BlobEnd(builder)
        
        # Global size
        gs = out_shape[2:] if len(out_shape) >= 4 else [W2, H2]
        BlobStartInt32sVector(builder, 3)
        for v in gs + [1]:
            builder.PrependInt32(v)
        gs_vec = builder.EndVector()
        
        BlobStart(builder)
        BlobAddInt32s(builder, gs_vec)
        gs_blob = BlobEnd(builder)
        
        # Uniform (CCM, WB, BLC, params)
        uniform = [3840.0, 2160.0, float(gs[0]), float(gs[1]), 1023.0,
                   1,0,0, 0,1,0, 0,0,1,  # CCM
                   1,1,1,1,  # WB
                   0,0,0,0]  # BLC
        BlobStartFloat32sVector(builder, len(uniform))
        for v in reversed(uniform):
            builder.PrependFloat32(v)
        uniform_vec = builder.EndVector()
        
        BlobStart(builder)
        BlobAddFloat32s(builder, uniform_vec)
        uniform_blob = BlobEnd(builder)
        
        # Bindings - first shader has 1 input, 4 outputs
        attrs = []
        
        # spirv
        AttributeStart(builder)
        AttributeAddKey(builder, strings['spirv'])
        AttributeAddI(builder, 0)
        AttributeAddTensor(builder, spirv_blob)
        attrs.append(AttributeEnd(builder))
        
        # global_size
        AttributeStart(builder)
        AttributeAddKey(builder, strings['global_size'])
        AttributeAddI(builder, 1)
        AttributeAddTensor(builder, gs_blob)
        attrs.append(AttributeEnd(builder))
        
        # const (uniform block)
        AttributeStart(builder)
        AttributeAddKey(builder, strings['const'])
        AttributeAddI(builder, 6)
        AttributeAddTensor(builder, uniform_blob)
        AttributeAddB(builder, True)
        attrs.append(AttributeEnd(builder))
        
        # inputs - binding 1 = tensor index
        # For shader 0: input is tensor 0, outputs are tensor 1-4
        # For shader 1: inputs are tensors 1-4, output is tensor 5
        if i == 0:
            # First shader: 1 input (tensor 0), output (tensor 1) - simplified for demo
            list_vec = ListValueCreateIVector(builder, [0, 1])
            ListValueStart(builder)
            ListValueAddI(builder, list_vec)
            list_obj = ListValueEnd(builder)
            
            AttributeStart(builder)
            AttributeAddKey(builder, strings['input'])
            AttributeAddI(builder, 1)
            AttributeAddList(builder, list_obj)
            AttributeAddB(builder, False)
            attrs.append(AttributeEnd(builder))
            
            # Output
            list_vec = ListValueCreateIVector(builder, [1, 2])
            ListValueStart(builder)
            ListValueAddI(builder, list_vec)
            list_obj = ListValueEnd(builder)
            
            AttributeStart(builder)
            AttributeAddKey(builder, strings['input'])
            AttributeAddI(builder, 2)
            AttributeAddList(builder, list_obj)
            AttributeAddB(builder, False)
            attrs.append(AttributeEnd(builder))
        else:
            # Other shaders: 4 inputs, 1 output
            for bi, ti in enumerate([1, 2, 3, 4]):
                list_vec = ListValueCreateIVector(builder, [ti, 1 + bi])
                ListValueStart(builder)
                ListValueAddI(builder, list_vec)
                list_obj = ListValueEnd(builder)
                
                AttributeStart(builder)
                AttributeAddKey(builder, strings['input'])
                AttributeAddI(builder, 1 + bi)
                AttributeAddList(builder, list_obj)
                AttributeAddB(builder, False)
                attrs.append(AttributeEnd(builder))
            
            # Output
            list_vec = ListValueCreateIVector(builder, [5, 5])
            ListValueStart(builder)
            ListValueAddI(builder, list_vec)
            list_obj = ListValueEnd(builder)
            
            AttributeStart(builder)
            AttributeAddKey(builder, strings['input'])
            AttributeAddI(builder, 5)
            AttributeAddList(builder, list_obj)
            AttributeAddB(builder, False)
            attrs.append(AttributeEnd(builder))
        
        # Create Extra op
        ExtraStartAttrVector(builder, len(attrs))
        for a in reversed(attrs):
            builder.PrependUOffsetTRelative(a)
        attr_vec = builder.EndVector()
        
        ExtraStart(builder)
        ExtraAddType(builder, strings['FusedISP'])
        ExtraAddAttr(builder, attr_vec)
        extra_op = ExtraEnd(builder)
        
        OpStartInputIndexesVector(builder, 1)
        builder.PrependInt32(in_idx)
        in_vec = builder.EndVector()
        
        OpStartOutputIndexesVector(builder, 1)
        builder.PrependInt32(out_idx)
        out_vec = builder.EndVector()
        
        OpStart(builder)
        OpAddType(builder, OpType.Extra)
        OpAddMainType(builder, OpParameter.Extra)
        OpAddMain(builder, extra_op)
        OpAddName(builder, strings[name])
        OpAddInputIndexes(builder, in_vec)
        OpAddOutputIndexes(builder, out_vec)
        ops.append(OpEnd(builder))
    
    # Oplists
    NetStartOplistsVector(builder, len(ops))
    for op in reversed(ops):
        builder.PrependUOffsetTRelative(op)
    ops_vec = builder.EndVector()
    
    # TensorDescribes
    tensor_dims = [
        [1, 1, INPUT_H, INPUT_W],    # 0: input
        [1, H, W], [1, H, W], [1, H, W], [1, H, W],  # 1-4: unpack
        [1, 3, H, W],  # 5: demosaic
        [1, 3, H, W], [1, 3, H, W], [1, 3, H, W],  # 6-8: fcs, ee, ldci
        [1, 3, H2, W2],  # 9: display
    ]
    
    tensor_descs = []
    for i, dims in enumerate(tensor_dims):
        blob = create_tensor_blob(builder, dims)
        TensorDescribeStart(builder)
        TensorDescribeAddBlob(builder, blob)
        TensorDescribeAddIndex(builder, i)
        TensorDescribeAddName(builder, strings[f'tensor_{i}'])
        tensor_descs.append(TensorDescribeEnd(builder))
    
    NetStartExtraTensorDescribeVector(builder, len(tensor_descs))
    for td in reversed(tensor_descs):
        builder.PrependUOffsetTRelative(td)
    extra_desc_vec = builder.EndVector()
    
    # Tensor names
    NetStartTensorNameVector(builder, len(tensor_names))
    for tn in reversed([strings[f'tensor_{i}'] for i in range(10)]):
        builder.PrependUOffsetTRelative(tn)
    tensor_names_vec = builder.EndVector()
    
    # Net
    NetStart(builder)
    NetAddOplists(builder, ops_vec)
    NetAddTensorName(builder, tensor_names_vec)
    NetAddExtraTensorDescribe(builder, extra_desc_vec)
    NetAddBizCode(builder, strings['FusedISPNet'])
    net = NetEnd(builder)
    
    builder.Finish(net)
    return builder.Output()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Generate fused ISP MNN model')
    parser.add_argument('--mode', choices=['atomic', 'fused'], default='atomic',
                       help='Generation mode')
    parser.add_argument('--output', default='/data/data/com.termux/files/home/softisp/vulkan_isp/fused_atomic.mnn')
    args = parser.parse_args()
    
    print(f"=== Generating Fused ISP Model ({args.mode} mode) ===\n")
    
    # Compile shaders
    print("Compiling shaders:")
    for name, filename, in_shape, out_shape in SHADERS:
        spv = compile_shader(filename)
        if spv:
            print(f"  {name}: {len(spv)} bytes SPIR-V")
        else:
            print(f"  {name}: FAILED")
    
    # Build model
    print("\nBuilding model...")
    if args.mode == 'atomic':
        model = build_atomic_model()
    else:
        print("Fused mode not yet implemented")
        return 1
    
    if model:
        with open(args.output, 'wb') as f:
            f.write(model)
        print(f"Model saved: {args.output} ({len(model)} bytes)")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())