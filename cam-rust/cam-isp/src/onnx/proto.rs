//! Pure Rust ONNX protobuf wire encoder.
//! Ported from com.camcore.isp.onnx.proto.OnnxProto.kt
//!
//! Implements the subset of protobuf wire format needed for a valid ONNX ModelProto.
//! No external protobuf library needed — generates raw bytes.

use std::vec::Vec;

/// Helper to build protobuf fields incrementally.
#[derive(Default)]
pub struct ProtoBuf {
    bytes: Vec<u8>,
}

impl ProtoBuf {
    pub fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    pub fn append(mut self, other: &[u8]) -> Self {
        self.bytes.extend_from_slice(other);
        self
    }

    pub fn build(self) -> Vec<u8> {
        self.bytes
    }

    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

/// ONNX protobuf encoder.
pub struct Proto;

impl Proto {
    // ── Low-level wire helpers ──

    fn tag(field: u32, wire: u32) -> u32 {
        (field << 3) | wire
    }

    fn write_varint(mut value: u64) -> Vec<u8> {
        if value == 0 {
            return vec![0];
        }
        let mut buf = Vec::with_capacity(10);
        while value != 0 {
            let byte = ((value & 0x7f) as u8) | 0x80;
            buf.push(byte);
            value >>= 7;
        }
        // Clear MSB of last byte
        if let Some(last) = buf.last_mut() {
            *last &= 0x7f;
        }
        buf
    }

    // ── Field encoders ──

    /// Encode a string field (wire type 2 = LENGTH_DELIMITED).
    pub fn string(field: u32, value: &str) -> Vec<u8> {
        Self::raw_bytes(field, value.as_bytes())
    }

    /// Encode raw bytes (wire type 2).
    pub fn raw_bytes(field: u32, value: &[u8]) -> Vec<u8> {
        let t = Self::write_varint(Self::tag(field, 2) as u64);
        let l = Self::write_varint(value.len() as u64);
        let mut out = Vec::with_capacity(t.len() + l.len() + value.len());
        out.extend_from_slice(&t);
        out.extend_from_slice(&l);
        out.extend_from_slice(value);
        out
    }

    /// Encode an int64 (wire type 0).
    pub fn int64(field: u32, value: i64) -> Vec<u8> {
        let t = Self::write_varint(Self::tag(field, 0) as u64);
        let v = Self::write_varint(value as u64);
        let mut out = Vec::with_capacity(t.len() + v.len());
        out.extend_from_slice(&t);
        out.extend_from_slice(&v);
        out
    }

    /// Encode an int32 (wire type 0).
    pub fn int32(field: u32, value: i32) -> Vec<u8> {
        Self::int64(field, value as i64)
    }

    /// Encode boolean (wire type 0).
    pub fn bool(field: u32, value: bool) -> Vec<u8> {
        Self::int64(field, if value { 1 } else { 0 })
    }

    /// Encode a float (wire type 5 = FIXED32, little-endian).
    pub fn float32(field: u32, value: f32) -> Vec<u8> {
        let t = Self::write_varint(Self::tag(field, 5) as u64);
        let bits = value.to_bits();
        let v = bits.to_le_bytes();
        let mut out = Vec::with_capacity(t.len() + 4);
        out.extend_from_slice(&t);
        out.extend_from_slice(&v);
        out
    }

    /// Encode a repeated int64 field (unpacked, proto2 style).
    /// Each value gets its own tag + varint.
    pub fn repeated_int64s(field: u32, values: &[i64]) -> Vec<u8> {
        let mut result = Vec::new();
        for v in values {
            result.extend_from_slice(&Self::int64(field, *v));
        }
        result
    }

    /// Encode a packed repeated field (proto3 style).
    pub fn packed_int64s(field: u32, values: &[i64]) -> Vec<u8> {
        let mut payload = Vec::new();
        for v in values {
            payload.extend_from_slice(&Self::write_varint(*v as u64));
        }
        Self::raw_bytes(field, &payload)
    }

    /// Encode an embedded message.
    pub fn message<F>(field: u32, f: F) -> Vec<u8>
    where
        F: FnOnce() -> Vec<u8>,
    {
        Self::raw_bytes(field, &f())
    }

    // ── ONNX-specific builders ──

    /// `onnx.TensorShapeProto.Dimension` — string dim_param (field 2).
    pub fn tensor_dim_param(value: &str) -> Vec<u8> {
        Self::raw_bytes(1, &Self::string(2, value))
    }

    /// `onnx.TensorShapeProto.Dimension` — int64 dim_value (field 1).
    pub fn tensor_dim_value(value: i64) -> Vec<u8> {
        Self::raw_bytes(1, &Self::int64(1, value))
    }

    /// `onnx.TensorShapeProto.Dimension` — concise wrapper: auto-detect string vs int.
    pub fn tensor_dim(value: impl Into<DimValue>) -> Vec<u8> {
        match value.into() {
            DimValue::Int(v) => Self::tensor_dim_value(v),
            DimValue::Str(s) => Self::tensor_dim_param(&s),
        }
    }

    /// `onnx.TypeProto.Tensor` with given element type and shape dimensions.
    pub fn tensor_type(elem_type: i32, dims: &[Vec<u8>]) -> Vec<u8> {
        let shape = Self::join_bytes(dims);
        let mut tt = Self::int32(1, elem_type);
        tt.extend_from_slice(&Self::raw_bytes(2, &shape));
        Self::raw_bytes(1, &tt) // TypeProto.tensor_type = field 1
    }

    /// `onnx.ValueInfoProto`.
    pub fn value_info(name: &str, shape: &[Vec<u8>], elem_type: i32) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        let tt = Self::tensor_type(elem_type, shape);
        buf.extend_from_slice(&Self::raw_bytes(2, &tt));
        buf
    }

    /// `onnx.AttributeProto` for int64 array. type=INTS(7).
    pub fn attribute_ints(name: &str, ints: &[i64]) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        buf.extend_from_slice(&Self::repeated_int64s(8, ints));
        buf.extend_from_slice(&Self::int32(20, 7));
        buf
    }

    /// `onnx.AttributeProto` for int array with an explicit `i` int64 value.
    /// Sets both field 3 (i) and field 7 (ints) for VulkanFuse compatibility.
    pub fn attribute_input_ints(name: &str, i_value: i64, ints: &[i64]) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        buf.extend_from_slice(&Self::int64(3, i_value));
        buf.extend_from_slice(&Self::repeated_int64s(8, ints));
        buf.extend_from_slice(&Self::int32(20, 7));
        buf
    }

    /// `onnx.AttributeProto` for string. type=STRING(3).
    pub fn attribute_string(name: &str, str_value: &str) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        buf.extend_from_slice(&Self::raw_bytes(4, str_value.as_bytes()));
        buf.extend_from_slice(&Self::int32(20, 3));
        buf
    }

    /// `onnx.AttributeProto` for float. type=FLOAT(1).
    pub fn attribute_float(name: &str, float_value: f32) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        buf.extend_from_slice(&Self::float32(2, float_value));
        buf.extend_from_slice(&Self::int32(20, 1));
        buf
    }

    /// `onnx.AttributeProto` for float array (unpacked, proto2). type=FLOATS(6).
    pub fn attribute_floats(name: &str, values: &[f32]) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        for v in values {
            buf.extend_from_slice(&Self::float32(7, *v));
        }
        buf.extend_from_slice(&Self::int32(20, 6));
        buf
    }

    /// `onnx.AttributeProto` for int64. type=INT(2).
    pub fn attribute_int(name: &str, value: i64) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        buf.extend_from_slice(&Self::int64(3, value));
        buf.extend_from_slice(&Self::int32(20, 2));
        buf
    }

    /// `onnx.AttributeProto` for a tensor. type=TENSOR(4).
    /// The tensor bytes should be encoded via `tensor_proto_raw_bytes` or similar.
    pub fn attribute_tensor(name: &str, tensor_bytes: &[u8]) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        buf.extend_from_slice(&Self::raw_bytes(5, tensor_bytes));
        buf.extend_from_slice(&Self::int32(20, 4));
        buf
    }

    /// `onnx.AttributeProto` for a const with int value AND tensor data.
    /// Used for VulkanFuse const attributes that need both binding index (i)
    /// and the actual data (tensor).
    pub fn attribute_const_tensor(name: &str, i_value: i64, tensor_bytes: &[u8]) -> Vec<u8> {
        let mut buf = Self::string(1, name);
        buf.extend_from_slice(&Self::int64(3, i_value));
        buf.extend_from_slice(&Self::raw_bytes(5, tensor_bytes));
        buf.extend_from_slice(&Self::int32(20, 4));
        buf
    }

    /// `onnx.TensorProto` for raw bytes with specified data type.
    /// data_type: 3=INT8, 2=UINT8, 1=FLOAT, 6=INT32, etc.
    /// dims: shape of the tensor (must be set for MNNConverter to calculate dataSize)
    pub fn tensor_proto_raw_bytes(
        name: &str,
        data: &[u8],
        data_type: i32,
        dims: &[i64],
    ) -> Vec<u8> {
        let mut buf = Self::string(8, name);
        buf.extend_from_slice(&Self::repeated_int64s(1, dims));
        buf.extend_from_slice(&Self::int32(2, data_type));
        buf.extend_from_slice(&Self::raw_bytes(9, data));
        buf
    }

    /// `onnx.TensorProto` for a float ND tensor with explicit dims (packed float_data).
    pub fn tensor_proto_float(name: &str, dims: &[i64], values: &[f32]) -> Vec<u8> {
        let raw_size = values.len() * 4;
        let mut raw = Vec::with_capacity(raw_size);
        for v in values {
            raw.extend_from_slice(&v.to_bits().to_le_bytes());
        }
        let mut buf = Self::string(8, name);
        buf.extend_from_slice(&Self::repeated_int64s(1, dims));
        buf.extend_from_slice(&Self::int32(2, 1)); // data_type = FLOAT(1)
        buf.extend_from_slice(&Self::raw_bytes(9, &raw)); // field 9 = raw_data
        buf
    }

    /// `onnx.TensorProto` for a float scalar.
    pub fn tensor_proto_float_scalar(name: &str, value: f32) -> Vec<u8> {
        let raw = value.to_bits().to_le_bytes();
        let mut buf = Self::string(8, name);
        buf.extend_from_slice(&Self::int32(2, 1)); // data_type = FLOAT(1)
        buf.extend_from_slice(&Self::raw_bytes(9, &raw)); // field 9 = raw_data
        buf
    }

    /// `onnx.TensorProto` for an int32 scalar (no dims = scalar).
    pub fn tensor_proto_int32_scalar(name: &str, value: i32) -> Vec<u8> {
        let raw = value.to_le_bytes();
        let mut buf = Self::string(8, name);
        buf.extend_from_slice(&Self::int32(2, 6)); // data_type = INT32(6)
        buf.extend_from_slice(&Self::raw_bytes(9, &raw)); // raw_data (field 9)
        buf
    }

    /// `onnx.TensorProto` for an int64 1-D tensor.
    pub fn tensor_proto_int64(name: &str, values: &[i64]) -> Vec<u8> {
        let raw_size = values.len() * 8;
        let mut raw = Vec::with_capacity(raw_size);
        for v in values {
            raw.extend_from_slice(&v.to_le_bytes());
        }
        let mut buf = Self::string(8, name);
        buf.extend_from_slice(&Self::repeated_int64s(1, &[values.len() as i64])); // shape dims
        buf.extend_from_slice(&Self::int32(2, 7)); // data_type = INT64(7)
        buf.extend_from_slice(&Self::raw_bytes(9, &raw)); // raw_data
        buf
    }

    /// `onnx.NodeProto`.
    /// NOTE: MNN's ONNX parser reads op_type from field 4 (output) rather than field 2.
    /// So we encode: op_type → field 4, inputs → field 1, outputs → field 2.
    /// See docs/mnn-inference-guide.md for details.
    pub fn node(op_type: &str, inputs: &[&str], outputs: &[&str], attrs: &[Vec<u8>]) -> Vec<u8> {
        let mut buf = Self::string(4, op_type);
        for inp in inputs {
            buf.extend_from_slice(&Self::raw_bytes(1, inp.as_bytes()));
        }
        for out in outputs {
            buf.extend_from_slice(&Self::raw_bytes(2, out.as_bytes()));
        }
        for a in attrs {
            buf.extend_from_slice(&Self::raw_bytes(5, a));
        }
        buf
    }

    /// `onnx.GraphProto`.
    pub fn graph(
        name: &str,
        nodes: &[Vec<u8>],
        inputs: &[Vec<u8>],
        outputs: &[Vec<u8>],
        initializers: &[Vec<u8>],
        value_info: &[Vec<u8>],
    ) -> Vec<u8> {
        let mut buf = Self::string(2, name);
        for n in nodes {
            buf.extend_from_slice(&Self::raw_bytes(1, n));
        }
        for init in initializers {
            buf.extend_from_slice(&Self::raw_bytes(5, init));
        }
        for i in inputs {
            buf.extend_from_slice(&Self::raw_bytes(11, i));
        }
        for o in outputs {
            buf.extend_from_slice(&Self::raw_bytes(12, o));
        }
        for vi in value_info {
            buf.extend_from_slice(&Self::raw_bytes(13, vi));
        }
        buf
    }

    /// `onnx.ModelProto`.
    pub fn model(ir_version: i64, opset: &[u8], producer: &str, graph: &[u8]) -> Vec<u8> {
        let mut buf = Self::int64(1, ir_version);
        buf.extend_from_slice(&Self::string(2, producer));
        buf.extend_from_slice(&Self::raw_bytes(7, graph));
        buf.extend_from_slice(&Self::raw_bytes(8, opset));
        buf
    }

    /// `onnx.OperatorSetIdProto`.
    pub fn opset(domain: &str, version: i64) -> Vec<u8> {
        let mut buf = Self::string(1, domain);
        buf.extend_from_slice(&Self::int64(2, version));
        buf
    }

    // ── Utility ──

    fn join_bytes(items: &[Vec<u8>]) -> Vec<u8> {
        let total: usize = items.iter().map(|b| b.len()).sum();
        let mut out = Vec::with_capacity(total);
        for item in items {
            out.extend_from_slice(item);
        }
        out
    }
}

/// Helper enum for tensor dimension values (int64 or string).
pub enum DimValue {
    Int(i64),
    Str(String),
}

impl From<i64> for DimValue {
    fn from(v: i64) -> Self {
        DimValue::Int(v)
    }
}

impl From<i32> for DimValue {
    fn from(v: i32) -> Self {
        DimValue::Int(v as i64)
    }
}

impl From<&str> for DimValue {
    fn from(s: &str) -> Self {
        DimValue::Str(s.to_string())
    }
}

impl From<String> for DimValue {
    fn from(s: String) -> Self {
        DimValue::Str(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_varint() {
        assert_eq!(Proto::write_varint(0), vec![0]);
        assert_eq!(Proto::write_varint(1), vec![1]);
        assert_eq!(Proto::write_varint(300), vec![0xac, 0x02]);
    }

    #[test]
    fn test_float32() {
        let result = Proto::float32(1, 1.0f32);
        assert!(!result.is_empty());
        // First byte should be tag: field=1, wire=5 => (1<<3)|5 = 13 = 0x0d
        assert_eq!(result[0], 0x0d);
        // Next 4 bytes are 1.0 in IEEE 754 little-endian
        assert_eq!(&result[1..5], &[0, 0, 0x80, 0x3f]);
    }

    #[test]
    fn test_string_field() {
        let result = Proto::string(1, "test");
        // Tag: field=1, wire=2 => (1<<3)|2 = 10 = 0x0a
        assert_eq!(result[0], 0x0a);
        // Length varint: 4
        assert_eq!(result[1], 4);
        // Data: "test"
        assert_eq!(&result[2..], b"test");
    }

    #[test]
    fn test_value_info() {
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_param("H"),
            Proto::tensor_dim_param("W"),
        ];
        let vi = Proto::value_info("frame", &dims, 1);
        assert!(!vi.is_empty());
    }

    #[test]
    fn test_node() {
        let node = Proto::node("Sub", &["input", "blc"], &["output"], &[]);
        assert!(!node.is_empty());
    }

    #[test]
    fn test_model() {
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_param("H"),
            Proto::tensor_dim_param("W"),
        ];
        let nodes = vec![Proto::node("Identity", &["input"], &["output"], &[])];
        let inputs = vec![Proto::value_info("input", &dims, 1)];
        let outputs = vec![Proto::value_info("output", &dims, 1)];
        let graph = Proto::graph("test_graph", &nodes, &inputs, &outputs, &[], &[]);
        let opset = Proto::opset("", 16);
        let model = Proto::model(11, &opset, "cam_rust_proto", &graph);
        assert!(!model.is_empty());
        assert!(model.len() > 20);
    }
}
