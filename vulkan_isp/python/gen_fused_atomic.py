#!/usr/bin/env python3
"""
Generate MNN model with 6 atomic fused ISP shaders
Each shader replaces a block, compiled for both FP16 and FP32
"""

import subprocess
import struct
import sys
import os

sys.path.insert(0, '/data/data/com.termux/files/home/MNN')

from MNN.Blob import BlobStart, BlobEnd, BlobAddInt8s, BlobAddFloat32s, BlobAddInt32s, BlobStartInt8sVector, BlobStartFloat32sVector, BlobStartInt32sVector
from MNN.Attribute import AttributeStart, AttributeEnd, AttributeAddKey, AttributeAddI, AttributeAddTensor, AttributeAddB, AttributeAddList
from MNN.ListValue import ListValueCreateIVector
from MNN.Extra import ExtraStart, ExtraEnd, ExtraAddType, ExtraAddAttr, ExtraStartAttrVector
from MNN.Op import OpStart, OpEnd, OpAddType, OpAddMainType, OpAddMain, OpAddName, OpStartInputIndexesVector, OpStartOutputIndexesVector, OpAddInputIndexes, OpAddOutputIndexes
from MNN.Net import NetStart, NetEnd, NetStartOplistsVector, NetStartTensorNameVector, NetAddOplists, NetAddTensorName, NetStartExtraTensorDescribeVector, NetAddExtraTensorDescribe, NetAddBizCode
from MNN.OpType import OpType
from MNN.OpParameter import OpParameter
from MNN.TensorDescribe import TensorDescribeStart, TensorDescribeEnd, TensorDescribeAddBlob, TensorDescribeAddIndex, TensorDescribeAddName
from MNN.Input import InputStart, InputEnd, InputAddDtype, InputAddDformat, InputStartDimsVector
from MNN.DataType import DataType
from MNN.MNN_DATA_FORMAT import MNN_DATA_FORMAT
import flatbuffers

# Shader files (without .comp)
SHADERS = [
    ('unpack_blc',  'shader1_unpack_blc',   '[1,4,H/2,W/2]'),   # SpaceToDepth + BLC + WB
    ('demosaic',    'shader2_demosaic_ccm',  '[1,3,H/2,W/2]'),   # Demosaic + CCM
    ('fcs',         'shader3_fcs',           '[1,3,H/2,W/2]'),   # Fcs
    ('ee',          'shader4_ee',            '[1,3,H/2,W/2]'),   # Ee
    ('ldci',        'shader5_ldci',          '[1,3,H/2,W/2]'),   # Ldci
    ('display',     'shader6_display',       '[1,3,H/4,W/4]'),   # Display + Downscale
]

def compile_shader(name, use_fp16=False):
    """Compile GLSL to SPIR-V, optionally for FP16"""
    base = f'/data/data/com.termux/files/home/softisp/vulkan_isp/{name}'
    src = f'{base}.comp'
    spv = f'{base}.spv'
    
    # Add FP16 define if needed
    cmd = ['/data/data/com.termux/files/usr/bin/glslang', '-V', src, '-o', spv]
    if use_fp16:
        cmd.extend(['-D', 'USE_FP16=1'])
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Failed to compile {src}: {result.stderr}")
        return None
    
    with open(spv, 'rb') as f:
        return f.read()

def build_fused_isp_model():
    """Build MNN model with all 6 fused ISP ops"""
    builder = flatbuffers.Builder(1024 * 1024)
    
    # Constants
    INPUT_W, INPUT_H = 3840, 2160
    OUT_W, OUT_H = INPUT_W // 2, INPUT_H // 2  # After SpaceToDepth
    OUT_W2, OUT_H2 = OUT_W // 2, OUT_H // 2    # After display downscale
    
    # Tensor indices
    # 0: Input Bayer [1,1,H,W] INT16
    # 1: unpack_out_r [1,H/2,W/2]
    # 2: unpack_out_g1 [1,H/2,W/2]
    # 3: unpack_out_g2 [1,H/2,W/2]
    # 4: unpack_out_b [1,H/2,W/2]
    # 5: demosaic_out [1,3,H/2,W/2]
    # 6: fcs_out [1,3,H/2,W/2]
    # 7: ee_out [1,3,H/2,W/2]
    # 8: ldci_out [1,3,H/2,W/2]
    # 9: display_out [1,3,H/4,W/4]
    
    # Create all strings first
    name_str = builder.CreateString("FusedISP")
    net_name = builder.CreateString("FusedISPNet")
    
    tensor_names = []
    for i in range(10):
        tensor_names.append(builder.CreateString(f"tensor_{i}"))
    
    # Build ops list
    ops = []
    
    # Compile shaders (using FP32 for now)
    shader_bytes = []
    for name, shader_name, shape in SHADERS:
        spv = compile_shader(shader_name, use_fp16=False)
        if spv is None:
            print(f"Failed to compile {shader_name}")
            return None
        shader_bytes.append(spv)
    
    # Create Input op (index 0 -> tensor 0)
    InputStartDimsVector(builder, 4)
    for v in [1, 1, INPUT_H, INPUT_W]:
        builder.PrependInt32(v)
    input_dims_vec = builder.EndVector()
    
    InputStart(builder)
    InputAddDtype(builder, DataType.DT_INT16)
    InputAddDformat(builder, MNN_DATA_FORMAT.NCHW)
    InputAddDims(builder, input_dims_vec)
    input_param = InputEnd(builder)
    
    input_op_name = builder.CreateString("input")
    OpStartInputIndexesVector(builder, 0)
    input_in_vec = builder.EndVector()
    OpStartOutputIndexesVector(builder, 1)
    builder.PrependInt32(0)
    input_out_vec = builder.EndVector()
    
    OpStart(builder)
    OpAddType(builder, OpType.Input)
    OpAddMainType(builder, OpParameter.Input)
    OpAddMain(builder, input_param)
    OpAddName(builder, input_op_name)
    OpAddInputIndexes(builder, input_in_vec)
    OpAddOutputIndexes(builder, input_out_vec)
    ops.append(OpEnd(builder))
    
    # Create Extra ops for each shader
    input_idx_map = [0, 1, 2, 3, 4, 5, 6, 7, 8]  # shader i takes output of previous
    output_idx_map = [1, 2, 3, 4, 5, 6, 7, 8, 9]  # shader i outputs to this
    
    op_names = ['UnpackBlock', 'DemosaicBlock', 'FcsBlock', 'EeBlock', 'LdciBlock', 'DisplayBlock']
    
    for i, ((name, shader_name, shape), spv_bytes) in enumerate(zip(SHADERS, shader_bytes)):
        in_idx = input_idx_map[i]
        out_idx = output_idx_map[i]
        
        # Create SPIR-V blob
        BlobStartInt8sVector(builder, len(spv_bytes))
        for b in reversed(spv_bytes):
            builder.PrependByte(b)
        spirv_vec = builder.EndVector()
        
        BlobStart(builder)
        BlobAddInt8s(builder, spirv_vec)
        spirv_blob = builder.End(builder)
        
        # Create global_size vector
        if i == 5:  # Display
            w, h = OUT_W2, OUT_H2
        else:
            w, h = OUT_W, OUT_H
        
        BlobStartInt32sVector(builder, 3)
        for v in [w, h, 1]:
            builder.PrependInt32(v)
        gs_vec = builder.EndVector()
        
        BlobStart(builder)
        BlobAddInt32s(builder, gs_vec)
        gs_blob = builder.End(builder)
        
        # Create uniform data (placeholder)
        uniform_data = [
            float(INPUT_W), float(INPUT_H), float(w), float(h), float(1023.0),  # dims
            1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0,  # CCM
            1.0, 1.0, 1.0, 1.0,  # WB gains
            0.0, 0.0, 0.0, 0.0,  # BLC
            0.0, 0.5, 0.0,  # Fcs params
            0.3, 0.1, 0.0, 0.0,  # Ee params
            0.2, 0.1, 0.0, 0.0,  # Ldci params
            0.0, 1.0, 1.0, 2.2,  # Display params
        ]
        BlobStartFloat32sVector(builder, len(uniform_data))
        for v in reversed(uniform_data):
            builder.PrependFloat32(v)
        uniform_vec = builder.EndVector()
        
        BlobStart(builder)
        BlobAddFloat32s(builder, uniform_vec)
        uniform_blob = builder.End(builder)
        
        # Create binding indices
        # Shader 1: input (1 binding), output (4 bindings)
        # Shader 2-5: input (4 bindings), output (1 binding)  
        # Shader 6: input (1 binding), output (1 binding)
        if i == 0:
            bindings = [(1, 0), (2, 1), (3, 2), (4, 3), (5, 4)]  # input + 4 outputs
        elif i == 5:
            bindings = [(in_idx, 1), (out_idx, 2)]  # 1 input, 1 output
        else:
            bindings = [(in_idx, 1), (in_idx+1, 2), (in_idx+2, 3), (in_idx+3, 4), (out_idx, 5)]  # 4 inputs, 1 output
        
        # Create attrs
        attrs = []
        
        # spirv (i=0)
        type_str = builder.CreateString("FusedISP")
        key_spirv = builder.CreateString("spirv")
        AttributeStart(builder)
        AttributeAddKey(builder, key_spirv)
        AttributeAddI(builder, 0)
        AttributeAddTensor(builder, spirv_blob)
        attrs.append(AttributeEnd(builder))
        
        # global_size (i=1)
        key_gs = builder.CreateString("global_size")
        AttributeStart(builder)
        AttributeAddKey(builder, key_gs)
        AttributeAddI(builder, 1)
        AttributeAddTensor(builder, gs_blob)
        attrs.append(AttributeEnd(builder))
        
        # uniform (i=6)
        key_const = builder.CreateString("const")
        AttributeStart(builder)
        AttributeAddKey(builder, key_const)
        AttributeAddI(builder, 6)
        AttributeAddTensor(builder, uniform_blob)
        AttributeAddB(builder, True)
        attrs.append(AttributeEnd(builder))
        
        # Input bindings
        for bi, (tensor_idx, binding) in enumerate(bindings):
            key_input = builder.CreateString("input")
            list_vec = ListValueCreateIVector(builder, [tensor_idx, binding])
            ListValueStart(builder)
            ListValueAddI(builder, list_vec)
            list_obj = ListValueEnd(builder)
            
            AttributeStart(builder)
            AttributeAddKey(builder, key_input)
            AttributeAddI(builder, 2 + bi)
            AttributeAddList(builder, list_obj)
            AttributeAddB(builder, binding > 1)  # const if binding >= 2
            attrs.append(AttributeEnd(builder))
        
        # Create Extra op
        ExtraStartAttrVector(builder, len(attrs))
        for a in reversed(attrs):
            builder.PrependUOffsetTRelative(a)
        attr_vec = builder.EndVector()
        
        ExtraStart(builder)
        ExtraAddType(builder, type_str)
        ExtraAddAttr(builder, attr_vec)
        extra_op = builder.End(ExtraEnd)
        
        OpStartInputIndexesVector(builder, 1)
        builder.PrependInt32(in_idx)
        in_idx_vec = builder.EndVector()
        
        OpStartOutputIndexesVector(builder, 1)
        builder.PrependInt32(out_idx)
        out_idx_vec = builder.EndVector()
        
        op_name = builder.CreateString(op_names[i])
        OpStart(builder)
        OpAddType(builder, OpType.Extra)
        OpAddMainType(builder, OpParameter.Extra)
        OpAddMain(builder, extra_op)
        OpAddName(builder, op_name)
        OpAddInputIndexes(builder, in_idx_vec)
        OpAddOutputIndexes(builder, out_idx_vec)
        ops.append(OpEnd(builder))
    
    # Create oplists vector
    NetStartOplistsVector(builder, len(ops))
    for op in reversed(ops):
        builder.PrependUOffsetTRelative(op)
    ops_vec = builder.EndVector()
    
    # Create TensorDescribes for all tensors
    tensor_descs = []
    
    # Tensor 0: input Bayer [1,1,H,W] INT16
    BlobStartInt32sVector(builder, 4)
    for v in [1, 1, INPUT_H, INPUT_W]:
        builder.PrependInt32(v)
    input_dims_vec = builder.EndVector()
    BlobStart(builder)
    BlobAddInt32s(builder, input_dims_vec)
    input_blob = builder.End(BlobEnd)
    
    TensorDescribeStart(builder)
    TensorDescribeAddBlob(builder, input_blob)
    TensorDescribeAddIndex(builder, 0)
    TensorDescribeAddName(builder, tensor_names[0])
    tensor_descs.append(TensorDescribeEnd(builder))
    
    # Tensors 1-4: unpack output [1,H/2,W/2] F32
    for t in range(1, 5):
        BlobStartInt32sVector(builder, 3)
        for v in [1, OUT_H, OUT_W]:
            builder.PrependInt32(v)
        dims_vec = builder.EndVector()
        BlobStart(builder)
        BlobAddInt32s(builder, dims_vec)
        blob = builder.End(BlobEnd)
        
        TensorDescribeStart(builder)
        TensorDescribeAddBlob(builder, blob)
        TensorDescribeAddIndex(builder, t)
        TensorDescribeAddName(builder, tensor_names[t])
        tensor_descs.append(TensorDescribeEnd(builder))
    
    # Tensor 5: demosaic output [1,3,H/2,W/2] F32
    BlobStartInt32sVector(builder, 4)
    for v in [1, 3, OUT_H, OUT_W]:
        builder.PrependInt32(v)
    dims_vec = builder.EndVector()
    BlobStart(builder)
    BlobAddInt32s(builder, dims_vec)
    blob = builder.End(BlobEnd)
    
    TensorDescribeStart(builder)
    TensorDescribeAddBlob(builder, blob)
    TensorDescribeAddIndex(builder, 5)
    TensorDescribeAddName(builder, tensor_names[5])
    tensor_descs.append(TensorDescribeEnd(builder))
    
    # Tensors 6-8: fcs, ee, ldci output [1,3,H/2,W/2] F32
    for t in range(6, 9):
        BlobStartInt32sVector(builder, 4)
        for v in [1, 3, OUT_H, OUT_W]:
            builder.PrependInt32(v)
        dims_vec = builder.EndVector()
        BlobStart(builder)
        BlobAddInt32s(builder, dims_vec)
        blob = builder.End(BlobEnd)
        
        TensorDescribeStart(builder)
        TensorDescribeAddBlob(builder, blob)
        TensorDescribeAddIndex(builder, t)
        TensorDescribeAddName(builder, tensor_names[t])
        tensor_descs.append(TensorDescribeEnd(builder))
    
    # Tensor 9: display output [1,3,H/4,W/4] F32
    BlobStartInt32sVector(builder, 4)
    for v in [1, 3, OUT_H2, OUT_W2]:
        builder.PrependInt32(v)
    dims_vec = builder.EndVector()
    BlobStart(builder)
    BlobAddInt32s(builder, dims_vec)
    blob = builder.End(BlobEnd)
    
    TensorDescribeStart(builder)
    TensorDescribeAddBlob(builder, blob)
    TensorDescribeAddIndex(builder, 9)
    TensorDescribeAddName(builder, tensor_names[9])
    tensor_descs.append(TensorDescribeEnd(builder))
    
    # Create extraTensorDescribe vector
    NetStartExtraTensorDescribeVector(builder, len(tensor_descs))
    for td in reversed(tensor_descs):
        builder.PrependUOffsetTRelative(td)
    extra_desc_vec = builder.EndVector()
    
    # Create tensor names vector
    NetStartTensorNameVector(builder, len(tensor_names))
    for tn in reversed(tensor_names):
        builder.PrependUOffsetTRelative(tn)
    tensor_names_vec = builder.EndVector()
    
    # Create net
    biz_code = builder.CreateString("Test")
    NetStart(builder)
    NetAddOplists(builder, ops_vec)
    NetAddTensorName(builder, tensor_names_vec)
    NetAddExtraTensorDescribe(builder, extra_desc_vec)
    NetAddBizCode(builder, biz_code)
    net = NetEnd(builder)
    
    builder.Finish(net)
    return builder.Output()

def main():
    print("=== Generating Fused ISP Model (6 atomic shaders) ===")
    
    # Compile all shaders
    print("\nCompiling shaders...")
    for name, shader_name, shape in SHADERS:
        for use_fp16 in [False, True]:
            suffix = "_fp16" if use_fp16 else "_fp32"
            spv = compile_shader(shader_name, use_fp16=use_fp16)
            if spv:
                print(f"  {shader_name}{suffix}: {len(spv)} bytes")
            else:
                print(f"  {shader_name}{suffix}: FAILED")
    
    # Build model
    print("\nBuilding MNN model...")
    model_bytes = build_fused_isp_model()
    if model_bytes:
        path = '/data/data/com.termux/files/home/softisp/vulkan_isp/fused_isp_atomic.mnn'
        with open(path, 'wb') as f:
            f.write(model_bytes)
        print(f"Model saved: {path} ({len(model_bytes)} bytes)")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())