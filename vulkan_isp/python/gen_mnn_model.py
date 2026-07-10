#!/usr/bin/env python3
"""
Generate MNN model with fused ISP Extra op containing SPIR-V
Uses MNN flatbuffers builder API directly
"""

import flatbuffers
import sys
import os
import subprocess
import struct

# MNN flatbuffers generated modules
sys.path.insert(0, '/data/data/com.termux/files/home/MNN')

from MNN.Blob import BlobStart, BlobEnd, BlobAddInt8s, BlobAddFloat32s, BlobAddInt32s, BlobStartInt8sVector, BlobStartFloat32sVector, BlobStartInt32sVector
from MNN.Attribute import AttributeStart, AttributeEnd, AttributeAddKey, AttributeAddI, AttributeAddTensor, AttributeAddB, AttributeAddList
from MNN.ListValue import ListValueStart, ListValueEnd, ListValueStartIVector, ListValueCreateIVector, ListValueAddI
from MNN.Extra import ExtraStart, ExtraEnd, ExtraAddType, ExtraAddAttr, ExtraStartAttrVector
from MNN.Op import OpStart, OpEnd, OpAddType, OpAddMainType, OpAddMain, OpAddName, OpStartInputIndexesVector, OpStartOutputIndexesVector, OpAddInputIndexes, OpAddOutputIndexes
from MNN.Net import NetStart, NetEnd, NetStartOplistsVector, NetStartTensorNameVector, NetAddOplists, NetAddTensorName, NetStartExtraTensorDescribeVector, NetAddExtraTensorDescribe, NetAddBizCode
from MNN.OpType import OpType
from MNN.OpParameter import OpParameter
from MNN.DataType import DataType
from MNN.TensorDescribe import TensorDescribeStart, TensorDescribeEnd, TensorDescribeAddBlob, TensorDescribeAddIndex, TensorDescribeAddName
from MNN.Input import InputStart, InputEnd, InputAddDims, InputAddDtype, InputAddDformat, InputStartDimsVector
from MNN.MNN_DATA_FORMAT import MNN_DATA_FORMAT

def compile_glsl_to_spirv(glsl_path, spirv_path):
    """Compile GLSL to SPIR-V"""
    result = subprocess.run(['/data/data/com.termux/files/usr/bin/glslang', '-V', glsl_path, '-o', spirv_path],
                          capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Failed: {result.stderr}")
        return None
    with open(spirv_path, 'rb') as f:
        return f.read()

def build_mnn_model(spirv_bytes):
    """Build MNN model with fused ISP Extra op using flatbuffers builder"""

    builder = flatbuffers.Builder(1024 * 1024)  # 1MB initial

    # ============================================================
    # STEP 1: Create ALL strings FIRST (before any Start/Vector calls)
    # ============================================================
    type_str = builder.CreateString("FusedISP")
    name_str = builder.CreateString("FusedISP")
    key_spirv = builder.CreateString("spirv")
    key_gs = builder.CreateString("global_size")
    key_input = builder.CreateString("input")
    key_const = builder.CreateString("const")
    net_name = builder.CreateString("FusedISPNet")
    input_name = builder.CreateString("input")
    out_r_name = builder.CreateString("output_r")
    out_g_name = builder.CreateString("output_g")
    out_b_name = builder.CreateString("output_b")

    # ============================================================
    # STEP 2: Create extraTensorDescribe Blobs for output tensors
    # Input: [1, 1, 2160, 3840] INT16 at index 0
    # Output R: [1, 1080, 1920] FP16 at index 1
    # Output G: [1, 1080, 1920] FP16 at index 2
    # Output B: [1, 1080, 1920] FP16 at index 3
    # ============================================================
    # Output R blob
    BlobStartInt32sVector(builder, 3)
    builder.PrependInt32(1920)
    builder.PrependInt32(1080)
    builder.PrependInt32(1)
    out_r_dims = builder.EndVector()

    BlobStart(builder)
    BlobAddInt32s(builder, out_r_dims)  # dims
    out_r_blob = BlobEnd(builder)

    # Output G blob
    BlobStartInt32sVector(builder, 3)
    builder.PrependInt32(1920)
    builder.PrependInt32(1080)
    builder.PrependInt32(1)
    out_g_dims = builder.EndVector()

    BlobStart(builder)
    BlobAddInt32s(builder, out_g_dims)
    out_g_blob = BlobEnd(builder)

    # Output B blob
    BlobStartInt32sVector(builder, 3)
    builder.PrependInt32(1920)
    builder.PrependInt32(1080)
    builder.PrependInt32(1)
    out_b_dims = builder.EndVector()

    BlobStart(builder)
    BlobAddInt32s(builder, out_b_dims)
    out_b_blob = BlobEnd(builder)

    # ============================================================
    # STEP 2: Create data vectors (OK - these don't nest)
    # ============================================================
    # SPIR-V data vector
    BlobStartInt8sVector(builder, len(spirv_bytes))
    for b in reversed(spirv_bytes):
        builder.PrependByte(b)
    spirv_data_vec = builder.EndVector()

    # global_size vector
    global_size_data = [240, 135, 1]
    BlobStartInt32sVector(builder, len(global_size_data))
    for v in reversed(global_size_data):
        builder.PrependInt32(v)
    global_size_vec = builder.EndVector()

    # uniform data vector
    uniform_data = [
        1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0,  # CCM
        1.0, 1.0, 1.0, 1.0,  # WB gains
        0.0, 0.0, 0.0, 0.0,  # BLC
        3840.0, 2160.0, 1920.0, 1080.0, 1023.0  # sizes
    ]
    BlobStartFloat32sVector(builder, len(uniform_data))
    for v in reversed(uniform_data):
        builder.PrependFloat32(v)
    uniform_vec = builder.EndVector()

    # ============================================================
    # STEP 3: Create Blob objects (these nest - must finish before more strings)
    # ============================================================
    BlobStart(builder)
    BlobAddInt8s(builder, spirv_data_vec)
    spirv_blob = BlobEnd(builder)

    BlobStart(builder)
    BlobAddInt32s(builder, global_size_vec)
    global_size_blob = BlobEnd(builder)

    BlobStart(builder)
    BlobAddFloat32s(builder, uniform_vec)
    uniform_blob = BlobEnd(builder)

    # ============================================================
    # STEP 4: Create ListValue objects for attributes (nest)
    # ============================================================
    list_vec_0 = ListValueCreateIVector(builder, [0, 0])
    list_vec_1 = ListValueCreateIVector(builder, [0, 1])
    list_vec_2 = ListValueCreateIVector(builder, [1, 1])
    list_vec_3 = ListValueCreateIVector(builder, [2, 1])

    # ============================================================
    # STEP 5: Create Attribute objects (nest)
    # ============================================================
    attrs = []

    # attr[0]: spirv
    AttributeStart(builder)
    AttributeAddKey(builder, key_spirv)
    AttributeAddI(builder, 0)
    AttributeAddTensor(builder, spirv_blob)
    attrs.append(AttributeEnd(builder))

    # attr[1]: global_size
    AttributeStart(builder)
    AttributeAddKey(builder, key_gs)
    AttributeAddI(builder, 1)
    AttributeAddTensor(builder, global_size_blob)
    attrs.append(AttributeEnd(builder))

    # attr[2]: input binding 0
    ListValueStart(builder)
    ListValueAddI(builder, list_vec_0)
    list_obj_0 = ListValueEnd(builder)

    AttributeStart(builder)
    AttributeAddKey(builder, key_input)
    AttributeAddI(builder, 2)
    AttributeAddList(builder, list_obj_0)
    AttributeAddB(builder, False)
    attrs.append(AttributeEnd(builder))

    # attr[3]: output_r binding 0
    ListValueStart(builder)
    ListValueAddI(builder, list_vec_1)
    list_obj_1 = ListValueEnd(builder)

    AttributeStart(builder)
    AttributeAddKey(builder, key_input)
    AttributeAddI(builder, 3)
    AttributeAddList(builder, list_obj_1)
    AttributeAddB(builder, False)
    attrs.append(AttributeEnd(builder))

    # attr[4]: output_g binding 1
    ListValueStart(builder)
    ListValueAddI(builder, list_vec_2)
    list_obj_2 = ListValueEnd(builder)

    AttributeStart(builder)
    AttributeAddKey(builder, key_input)
    AttributeAddI(builder, 4)
    AttributeAddList(builder, list_obj_2)
    AttributeAddB(builder, False)
    attrs.append(AttributeEnd(builder))

    # attr[5]: output_b binding 2
    ListValueStart(builder)
    ListValueAddI(builder, list_vec_3)
    list_obj_3 = ListValueEnd(builder)

    AttributeStart(builder)
    AttributeAddKey(builder, key_input)
    AttributeAddI(builder, 5)
    AttributeAddList(builder, list_obj_3)
    AttributeAddB(builder, False)
    attrs.append(AttributeEnd(builder))

    # attr[6]: uniform binding
    AttributeStart(builder)
    AttributeAddKey(builder, key_const)
    AttributeAddI(builder, 6)
    AttributeAddTensor(builder, uniform_blob)
    AttributeAddB(builder, True)
    attrs.append(AttributeEnd(builder))

    # ============================================================
    # STEP 6: Create attr vector, Extra op
    # ============================================================
    ExtraStartAttrVector(builder, len(attrs))
    for a in reversed(attrs):
        builder.PrependUOffsetTRelative(a)
    attr_vec = builder.EndVector()

    ExtraStart(builder)
    ExtraAddType(builder, type_str)
    ExtraAddAttr(builder, attr_vec)
    extra_op = ExtraEnd(builder)

    # Create index vectors for Extra op BEFORE OpStart (no nesting allowed)
    OpStartInputIndexesVector(builder, 1)
    builder.PrependInt32(0)
    in_idx_vec = builder.EndVector()

    OpStartOutputIndexesVector(builder, 3)
    builder.PrependInt32(3)
    builder.PrependInt32(2)
    builder.PrependInt32(1)
    out_idx_vec = builder.EndVector()

    OpStart(builder)
    OpAddType(builder, OpType.Extra)
    OpAddMainType(builder, OpParameter.Extra)
    OpAddMain(builder, extra_op)
    OpAddName(builder, name_str)
    OpAddInputIndexes(builder, in_idx_vec)
    OpAddOutputIndexes(builder, out_idx_vec)
    extra_op_node = OpEnd(builder)

    # ============================================================
    # STEP 7: Create Input op to set input tensor dimensions
    # ============================================================
    input_name_str = builder.CreateString("input")

    # Create dims vector for Input op
    input_dims = [1, 1, 2160, 3840]
    InputStartDimsVector(builder, len(input_dims))
    for v in reversed(input_dims):
        builder.PrependInt32(v)
    input_dims_vec = builder.EndVector()

    InputStart(builder)
    InputAddDtype(builder, DataType.DT_INT16)
    InputAddDformat(builder, MNN_DATA_FORMAT.NCHW)
    InputAddDims(builder, input_dims_vec)
    input_param = InputEnd(builder)

    # Create Input op
    input_op_name = builder.CreateString("input")
    OpStartInputIndexesVector(builder, 0)  # no inputs
    input_in_idx_vec = builder.EndVector()

    OpStartOutputIndexesVector(builder, 1)
    builder.PrependInt32(0)
    input_out_idx_vec = builder.EndVector()

    OpStart(builder)
    OpAddType(builder, OpType.Input)
    OpAddMainType(builder, OpParameter.Input)
    OpAddMain(builder, input_param)
    OpAddName(builder, input_op_name)
    OpAddInputIndexes(builder, input_in_idx_vec)
    OpAddOutputIndexes(builder, input_out_idx_vec)
    input_op_node = OpEnd(builder)

    # ============================================================
    # STEP 8: Create Oplists vector (Input op first, then Extra op)
    # ============================================================
    NetStartOplistsVector(builder, 2)
    builder.PrependUOffsetTRelative(extra_op_node)
    builder.PrependUOffsetTRelative(input_op_node)
    ops_vec = builder.EndVector()

    # ============================================================
    # STEP 9: Create TensorDescribe for input tensor (index 0)
    # ============================================================
    input_name_str = builder.CreateString("input")
    input_dims = [1, 1, 2160, 3840]
    BlobStartInt32sVector(builder, len(input_dims))
    for v in reversed(input_dims):
        builder.PrependInt32(v)
    input_dims_vec = builder.EndVector()

    BlobStart(builder)
    BlobAddInt32s(builder, input_dims_vec)
    input_blob = BlobEnd(builder)

    TensorDescribeStart(builder)
    TensorDescribeAddBlob(builder, input_blob)
    TensorDescribeAddIndex(builder, 0)
    TensorDescribeAddName(builder, input_name)
    input_desc = TensorDescribeEnd(builder)

    # ============================================================
    # STEP 10: Create TensorDescribe for output tensors (indices 1, 2, 3)
    # ============================================================
    # Output R (index 1)
    TensorDescribeStart(builder)
    TensorDescribeAddBlob(builder, out_r_blob)
    TensorDescribeAddIndex(builder, 1)
    TensorDescribeAddName(builder, out_r_name)
    out_r_desc = TensorDescribeEnd(builder)

    # Output G (index 2)
    TensorDescribeStart(builder)
    TensorDescribeAddBlob(builder, out_g_blob)
    TensorDescribeAddIndex(builder, 2)
    TensorDescribeAddName(builder, out_g_name)
    out_g_desc = TensorDescribeEnd(builder)

    # Output B (index 3)
    TensorDescribeStart(builder)
    TensorDescribeAddBlob(builder, out_b_blob)
    TensorDescribeAddIndex(builder, 3)
    TensorDescribeAddName(builder, out_b_name)
    out_b_desc = TensorDescribeEnd(builder)

    # Create extraTensorDescribe vector
    NetStartExtraTensorDescribeVector(builder, 4)
    builder.PrependUOffsetTRelative(out_b_desc)
    builder.PrependUOffsetTRelative(out_g_desc)
    builder.PrependUOffsetTRelative(out_r_desc)
    builder.PrependUOffsetTRelative(input_desc)
    extra_desc_vec = builder.EndVector()

    NetStartTensorNameVector(builder, 4)
    builder.PrependUOffsetTRelative(out_b_name)
    builder.PrependUOffsetTRelative(out_g_name)
    builder.PrependUOffsetTRelative(out_r_name)
    builder.PrependUOffsetTRelative(input_name)
    tensor_names_vec = builder.EndVector()

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
    print("=== Generating MNN model with fused ISP shader ===")

    # Compile GLSL to SPIR-V
    spirv_bytes = compile_glsl_to_spirv('fused_isp.comp', 'fused_isp.spv')
    if spirv_bytes is None:
        return 1

    print(f"SPIR-V compiled: {len(spirv_bytes)} bytes")

    # Build MNN model
    model_bytes = build_mnn_model(spirv_bytes)
    if model_bytes is None:
        print("Failed to build model")
        return 1

    with open('fused_isp.mnn', 'wb') as f:
        f.write(model_bytes)
    print(f"Model saved: fused_isp.mnn ({len(model_bytes)} bytes)")

    return 0

if __name__ == '__main__':
    sys.exit(main())