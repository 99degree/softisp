import struct

# Create a minimal valid ONNX model: FLOAT[1,3,4,4] → Mul(2.0) → output
# We'll craft the protobuf manually

# Helper to write Varint
def varint(n):
    r = b''
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            b |= 0x80
        r += bytes([b])
        if not n:
            return r

# Build TensorProto for scale_val = 2.0 (FLOAT32)
# dims=[], data_type=1 (FLOAT), raw_data=b'\x00\x00\x00@'
scale = b'\n\tscale_val'  # name
scale += b'\x10\x01'  # data_type=1 (FLOAT)
scale += b'\x18\x04'  # raw_data length=4
scale += b'B\x04\x00\x00\x00@'  # raw_data = 2.0f

# ValueInfo for input: name="input", type=tensor(elem_type=1, shape=[1,3,4,4])
# TensorShapeProto: dim[value=1], dim[value=3], dim[value=4], dim[value=4]
input_vi = b'\n\x05input'  # name
input_vi += b'\x12\x12'  # type, length=18
input_vi += b'\x08\x01'  # elem_type=1 (FLOAT)
input_vi += b'\x12\x0c'  # shape, length=12
input_vi += b'\x12\x01\x01'  # dim value=1
input_vi += b'\x12\x01\x03'  # dim value=3
input_vi += b'\x12\x01\x04'  # dim value=4
input_vi += b'\x12\x01\x04'  # dim value=4

# ValueInfo for output
output_vi = b'\n\x06output'
output_vi += b'\x12\x12'  # type
output_vi += b'\x08\x01'  # FLOAT
output_vi += b'\x12\x0c'  # shape
output_vi += b'\x12\x01\x01'  # 1
output_vi += b'\x12\x01\x03'  # 3
output_vi += b'\x12\x01\x04'  # 4
output_vi += b'\x12\x01\x04'  # 4

# Node: Mul
# NodeProto: name="mul", op_type="Mul", input=["input","scale_val"], output=["output"]
node = b'\n\x03mul'  # name
node += b'\x12\x03Mul'  # op_type
node += b'\x1a\x05input'  # input[0]
node += b'\x1a\tscale_val'  # input[1]
node += b'\x1a\x06output'  # output[0]

# GraphProto
graph = b'\x12\x10mul_test_graph'  # name
# nodes
graph += node
# inputs (value_info)
graph += input_vi
# outputs (value_info)
graph += output_vi
# initializers
graph += scale

# ModelProto
model = b'\x08\x07'  # ir_version=7
model += b'\x12\x19\x63\x61\x6d\x5f\x72\x75\x73\x74\x5f\x73\x69\x6d\x70\x6c\x65'  # producer_name
model += b'\x3a' + bytes([len(graph)]) + graph  # graph field

with open('simple_mul.onnx', 'wb') as f:
    f.write(model)
print(f"Written {len(model)} bytes to simple_mul.onnx")
