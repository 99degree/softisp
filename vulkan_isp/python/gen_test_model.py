#!/usr/bin/env python3
"""Generate minimal MNN model with a trivial SPIR-V shader via VulkanFuse.

The shader (test_constant.spv) writes 4 float values to output:
  out_data[0] = 1.0
  out_data[1] = 2.0
  out_data[2] = 3.0
  out_data[3] = 4.0

Model: single Extra op, input tensor_0, output tensor_1
"""

import sys
import os
sys.path.insert(0, '/data/data/com.termux/files/home/MNN')

import flatbuffers
from MNN.OpType import OpType
from MNN.OpParameter import OpParameter
from flatbuffers.builder import Builder

def main():
    # Read SPIR-V
    spv_path = '/data/data/com.termux/files/home/softisp/vulkan_isp/test_constant.spv'
    with open(spv_path, 'rb') as f:
        spv = f.read()
    print(f"SPIR-V: {len(spv)} bytes")

    builder = flatbuffers.Builder(1024)

    # Create strings
    name_spirv = builder.CreateString('spirv')
    name_output_shape = builder.CreateString('output_shape')
    name_global_size = builder.CreateString('global_size')
    name_input = builder.CreateString('input')
    name_op = builder.CreateString('TestConst')
    name_t0 = builder.CreateString('tensor_0')
    name_t1 = builder.CreateString('tensor_1')
    name_net = builder.CreateString('TestNet')

    # SPIR-V blob - must create vector BEFORE starting table
    # Use PrependByte for int8 vector
    MNN.BlobStartInt8sVector(builder, len(spv))
    for b in reversed(spv):
        # flatbuffers adds bytes in reverse of declaration order
        # because vectors are built by PrependByte
        if isinstance(b, int):
            builder.PrependByte(b)
        else:
            builder.PrependByte(b)
    spirv_vec = builder.EndVector()

    from MNN.Blob import BlobStart, BlobEnd, BlobAddInt8s, BlobAddDataType
    from MNN.DataType import DataType

    BlobStart(builder)
    BlobAddInt8s(builder, spirv_vec)
    BlobAddDataType(builder, DataType.DT_INT8)
    spirv_blob = BlobEnd(builder)

    # Output shape blob [1,4,1,1]
    MNN.BlobStartInt32sVector(builder, 4)
    for v in reversed([1, 4, 1, 1]):
        builder.PrependInt32(v)
    shape_ivec = builder.EndVector()

    BlobStart(builder)
    BlobAddInt32s(builder, shape_ivec)
    shape_blob = BlobEnd(builder)

    # Global size blob [1,1,1]
    MNN.BlobStartInt32sVector(builder, 3)
    for v in reversed([1, 1, 1]):
        builder.PrependInt32(v)
    gs_ivec = builder.EndVector()

    BlobStart(builder)
    BlobAddInt32s(builder, gs_ivec)
    gs_blob = BlobEnd(builder)

    # List values for input and output bindings
    from MNN.ListValue import ListValueStart, ListValueEnd, ListValueAddI

    # Input: [is_output=0, binding=1] at tensor position 0
    MNN.ListValueStartIVector(builder, 2)
    for v in reversed([0, 1]):
        builder.PrependInt32(v)
    list_i_vec = builder.EndVector()

    ListValueStart(builder)
    ListValueAddI(builder, list_i_vec)
    list_input = ListValueEnd(builder)

    # Output: [is_output=1, binding=2] at tensor position 0
    MNN.ListValueStartIVector(builder, 2)
    for v in reversed([1, 2]):
        builder.PrependInt32(v)
    list_o_vec = builder.EndVector()

    ListValueStart(builder)
    ListValueAddI(builder, list_o_vec)
    list_output = ListValueEnd(builder)

    # Build attributes
    from MNN.Attribute import AttributeStart, AttributeEnd, AttributeAddKey, AttributeAddI, AttributeAddTensor, AttributeAddB, AttributeAddList

    attrs = []

    # spirv
    AttributeStart(builder)
    AttributeAddKey(builder, name_spirv)
    AttributeAddTensor(builder, spirv_blob)
    attrs.append(AttributeEnd(builder))

    # output_shape
    AttributeStart(builder)
    AttributeAddKey(builder, name_output_shape)
    AttributeAddTensor(builder, shape_blob)
    attrs.append(AttributeEnd(builder))

    # global_size
    AttributeStart(builder)
    AttributeAddKey(builder, name_global_size)
    AttributeAddTensor(builder, gs_blob)
    attrs.append(AttributeEnd(builder))

    # input (binding 1 for tensor 0)
    AttributeStart(builder)
    AttributeAddKey(builder, name_input)
    AttributeAddI(builder, 0)  # position 0 in op's input list
    AttributeAddList(builder, list_input)
    AttributeAddB(builder, False)
    attrs.append(AttributeEnd(builder))

    # input (binding 2 for output tensor 0)
    AttributeStart(builder)
    AttributeAddKey(builder, name_input)
    AttributeAddI(builder, 0)  # position 0 in op's output list
    AttributeAddList(builder, list_output)
    AttributeAddB(builder, False)
    attrs.append(AttributeEnd(builder))

    # Extra op
    from MNN.Extra import ExtraStart, ExtraEnd, ExtraAddType, ExtraAddAttr, ExtraStartAttrVector

    ExtraStartAttrVector(builder, len(attrs))
    for a in reversed(attrs):
        builder.PrependUOffsetTRelative(a)
    attr_vec = builder.EndVector()

    ExtraStart(builder)
    ExtraAddType(builder, name_op)
    ExtraAddAttr(builder, attr_vec)
    extra_op = ExtraEnd(builder)

    # Op
    from MNN.Op import OpStart, OpEnd, OpAddType, OpAddMainType, OpAddMain, OpAddName, OpStartInputIndexesVector, OpStartOutputIndexesVector, OpAddInputIndexes, OpAddOutputIndexes

    # Input indexes: [0]
    OpStartInputIndexesVector(builder, 1)
    builder.PrependInt32(0)
    in_vec = builder.EndVector()

    # Output indexes: [1]
    OpStartOutputIndexesVector(builder, 1)
    builder.PrependInt32(1)
    out_vec = builder.EndVector()

    OpStart(builder)
    OpAddType(builder, OpType.Extra)
    OpAddMainType(builder, OpParameter.Extra)
    OpAddMain(builder, extra_op)
    OpAddName(builder, name_op)
    OpAddInputIndexes(builder, in_vec)
    OpAddOutputIndexes(builder, out_vec)
    op = OpEnd(builder)

    # Net
    from MNN.Net import NetStart, NetEnd, NetAddOplists, NetAddTensorName, NetAddBizCode, NetStartOplistsVector, NetStartTensorNameVector

    NetStartOplistsVector(builder, 1)
    builder.PrependUOffsetTRelative(op)
    ops_vec = builder.EndVector()

    NetStartTensorNameVector(builder, 2)
    for tn in reversed([name_t1, name_t0]):
        builder.PrependUOffsetTRelative(tn)
    tensors_vec = builder.EndVector()

    NetStart(builder)
    NetAddOplists(builder, ops_vec)
    NetAddTensorName(builder, tensors_vec)
    NetAddBizCode(builder, name_net)
    net = NetEnd(builder)

    builder.Finish(net)

    # Save
    output_path = '/data/data/com.termux/files/home/softisp/vulkan_isp/test_minimal.mnn'
    with open(output_path, 'wb') as f:
        f.write(builder.Output())
    print(f"Model saved: {output_path} ({len(builder.Output())} bytes)")


if __name__ == '__main__':
    main()
