//! BayerWbBlock — Bayer-domain white balance gain.
//!
//! Multiplies each Bayer channel by its white balance gain.
//! 4-channel multiplication (RGGB pattern order).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// BayerWbBlock — per-channel white balance gain for Bayer data.
///
/// Applies WB gains: `out_ch = in_ch * gain[ch]` for each of 4 Bayer
/// positions (R, G1, G2, B). Gains are hot-swappable at runtime.

pub struct BayerWbBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl BayerWbBlock {
    pub fn new() -> Self { Self { id: "bayer_wb".into(), prev: None, next: None, frame_tensor: "BayerWbBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for BayerWbBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "BayerWbBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1),Proto::tensor_dim_value(4),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node("Mul", &[&self.input_source, &format!("{}/gains", ns)], &[&format!("{}/wb_applied", ns)], &[]),
            Proto::node("Clip", &[&format!("{}/wb_applied", ns), &format!("{}/zero", ns), &format!("{}/one", ns)], &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float(&format!("{}/gains", ns), &[1, 4, 1, 1], &[1.0, 1.0, 1.0, 1.0]),
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/gains", self.tensor_ns()), 1, vec![1, 4, 1, 1])]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bayer_wb_id() {
        assert_eq!(BayerWbBlock::new().id(), "bayer_wb");
    }

    #[test]
    fn test_bayer_wb_nodes() {
        let mut b = BayerWbBlock::new();
        b.set_input_source("in/bayer");
        let nodes = b.nodes();
        // Mul + Clip = 2 nodes
        assert_eq!(nodes.len(), 2);
    }

    #[test]
    fn test_bayer_wb_initializers() {
        let b = BayerWbBlock::new();
        let inits = b.initializers();
        // gains + zero + one = 3
        assert_eq!(inits.len(), 3);
    }

    #[test]
    fn test_bayer_wb_extra_inputs() {
        let b = BayerWbBlock::new();
        let extra = b.extra_inputs();
        assert_eq!(extra.len(), 1);
        assert_eq!(extra[0].2, vec![1, 4, 1, 1]);
    }
}
