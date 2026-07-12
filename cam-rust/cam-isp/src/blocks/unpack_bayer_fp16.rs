// cam-rust/cam-isp/src/blocks/unpack_bayer_fp16.rs
//! UnpackBayerToFp16Block: Unpacks packed INT32 Bayer data to 2-channel FP16
//!
//! Input: packed INT32 `[1,1,H,W]` where each int32 = (A << 16) | B
//!        and A, B are 16-bit values containing 10-bit Bayer samples in bits 0-9
//!
//! Output: 2-channel FP16 `[1,2,H,W]` (two unpacked channels)
//!         Each channel is normalised to `[0,1]` by dividing by 1023.0
//!
//! Note: This is a simplified version that unpacks to 2 channels.
//! A full Bayer unpack would produce 4 channels (R, Gr, Gb, B) with proper rearrangement.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

pub struct UnpackBayerToFp16Block {
    id: String,
    input_source: String,
    output_name: String,
}

impl Default for UnpackBayerToFp16Block {
    fn default() -> Self {
        Self::new()
    }
}

impl UnpackBayerToFp16Block {
    pub fn new() -> Self {
        Self {
            id: "unpack_bayer_fp16".into(),
            input_source: "RawInputBlock/frame".into(),
            output_name: "UnpackBayerToFp16Block/frame_fp16".into(),
        }
    }
}

impl IspBlock for UnpackBayerToFp16Block {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "UnpackBayerToFp16Block".into()
    }

    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.output_name)
    }

    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }

    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
    }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }

    fn output_tensors(&self) -> Vec<String> {
        vec![self.output_name.clone()]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.output_name,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(2),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            10, // FLOAT16
        ))
    }

    /// Build ONNX nodes for unpacking:
    /// 1. Split INT32 into high/low 16-bit halves
    /// 2. Mask to 10 bits (0x3FF)
    /// 3. Cast to FP16
    /// 4. Normalise by dividing by 1023.0
    /// 5. Concat into 2-channel output
    #[allow(clippy::vec_init_then_push)]
    fn nodes(&self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();

        // Constants
        out.push(Proto::tensor_proto_int32_scalar("shift_16", 16));
        out.push(Proto::tensor_proto_int32_scalar("mask_10", 0x3FF));
        out.push(Proto::tensor_proto_int32_scalar("mask_ffff", 0xFFFF));
        out.push(Proto::tensor_proto_float_scalar(
            "scale_1_1023",
            1.0 / 1023.0,
        ));

        // 1. Extract high 16 bits (A)
        out.push(Proto::node(
            "RightShift",
            &[&self.input_source, "shift_16"],
            &["high_16"],
            &[],
        ));

        // 2. Extract low 16 bits (B)
        out.push(Proto::node(
            "BitwiseAnd",
            &[&self.input_source, "mask_ffff"],
            &["low_16"],
            &[],
        ));

        // 3. Mask both to 10 bits
        out.push(Proto::node(
            "BitwiseAnd",
            &["high_16", "mask_10"],
            &["a_10"],
            &[],
        ));
        out.push(Proto::node(
            "BitwiseAnd",
            &["low_16", "mask_10"],
            &["b_10"],
            &[],
        ));

        // 4. Cast to FP16
        out.push(Proto::node(
            "Cast",
            &["a_10"],
            &["a_fp16"],
            &[Proto::attribute_int("to", 10)], // FLOAT16
        ));
        out.push(Proto::node(
            "Cast",
            &["b_10"],
            &["b_fp16"],
            &[Proto::attribute_int("to", 10)], // FLOAT16
        ));

        // 5. Normalise
        out.push(Proto::node(
            "Div",
            &["a_fp16", "scale_1_1023"],
            &["a_norm"],
            &[],
        ));
        out.push(Proto::node(
            "Div",
            &["b_fp16", "scale_1_1023"],
            &["b_norm"],
            &[],
        ));

        // 6. Concat into 2-channel output
        out.push(Proto::node(
            "Concat",
            &["a_norm", "b_norm"],
            &[&self.output_name],
            &[Proto::attribute_int("axis", 1)],
        ));

        out
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn input_elem_type(&self) -> i32 {
        6
    } // INT32
    fn output_elem_type(&self) -> i32 {
        10
    } // FLOAT16

    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        None
    }
    fn set_prev(&mut self, _: Box<dyn IspBlock>) {}
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        None
    }
    fn set_next(&mut self, _: Box<dyn IspBlock>) {}
    fn graph_input_name(&self) -> Option<&str> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unpack_bayer_fp16_id() {
        assert_eq!(UnpackBayerToFp16Block::new().id(), "unpack_bayer_fp16");
    }

    #[test]
    fn test_unpack_bayer_fp16_types() {
        let b = UnpackBayerToFp16Block::new();
        assert_eq!(b.input_elem_type(), 6); // INT32
        assert_eq!(b.output_elem_type(), 10); // FLOAT16
    }

    #[test]
    fn test_unpack_bayer_fp16_nodes() {
        let mut b = UnpackBayerToFp16Block::new();
        b.set_input_source("in/packed");
        let nodes = b.nodes();
        // Multiple nodes for unpack: ShiftLeft, ShiftRight, BitwiseAnd, Cast, Div, Concat
        assert!(nodes.len() >= 6);
    }

    #[test]
    fn test_unpack_bayer_fp16_tensors() {
        let mut b = UnpackBayerToFp16Block::new();
        b.set_input_source("in/packed");
        assert_eq!(
            b.output_tensors(),
            vec!["UnpackBayerToFp16Block/frame_fp16".to_string()]
        );
    }

    #[test]
    fn test_unpack_bayer_fp16_has_input_output() {
        let b = UnpackBayerToFp16Block::new();
        assert!(!b.input_tensors().is_empty());
        assert!(!b.output_tensors().is_empty());
    }

    #[test]
    fn test_unpack_bayer_fp16_extra_inputs() {
        let b = UnpackBayerToFp16Block::new();
        // FP16 unpack doesn't use extra inputs for BLC/WB (those are in the const buffer)
        let _ = b.extra_inputs();
    }
}
