//! # LdciBlock — Local Dynamic Contrast Improvement
//!
//! ## ONNX Subgraph (Atomic)
//! Generates 4 ops: `Pool(AVG 3×3) → Sub → Mul → Add` → `enhanced`
//!
//! ## IspChainFusion Standard Rule: R5
//! R5 detects: `Pool(AVG,kernel=3×3,stride=1) + Sub + Mul + Add` → `isp.ldci`
//! * Pool computes per-pixel local mean in 3×3 window
//! * Sub: `diff = original - local_mean` (extracts local deviation)
//! * Mul: `boost = diff * strength` (scale contrast)
//! * Add: `output = original + boost` (recombine)
//!
//! The isp.ldci SPIR-V shader (`shader5_ldci.comp`) does all 4 ops in one
//! dispatch: 3×3 box blur → local_mean, diff = center - local_mean,
//! output = center + strength * diff. Per-channel on all 3 RGB channels.
//!
//! Trade-off vs original Rust block (global mean + Y-only):
//! - Original: ReduceMean(H,W) → global contrast → Y-only boost → 6 ops
//! - Current: Pool(AVG 3×3) → local contrast → per-channel boost → 4 ops, R5 fuses

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// LdciBlock — Local Detail and Contrast Enhancement.
///
/// ONNX subgraph: AvgPool → Sub → Mul → Add + Mul+Add for contrast.
/// Enhances local texture independently of global exposure.
/// Fuse target R4 (AvgPool+Sub+Mul+Add).
pub struct LdciBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl Default for LdciBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl LdciBlock {
    pub fn new() -> Self { Self { id: "ldci".into(), prev: None, next: None, frame_tensor: "LdciBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for LdciBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "LdciBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1),Proto::tensor_dim_value(3),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }

    /// Pool(AVG 3×3) → Sub(original, blur) → Mul(diff, strength) → Add(original, boost)
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Pool(AVG, kernel=3×3, stride=1, same-pad) → local mean
            Proto::node("AveragePool", &[&self.input_source],
                &[&format!("{}/local_mean", ns)],
                &[Proto::attribute_ints("kernel_shape", &[3, 3]),
                  Proto::attribute_ints("strides", &[1, 1]),
                  Proto::attribute_ints("pads", &[1, 1, 1, 1])]),
            // Sub(original, local_mean) → diff (local deviation)
            Proto::node("Sub", &[&self.input_source, &format!("{}/local_mean", ns)],
                &[&format!("{}/diff", ns)], &[]),
            // Mul(diff, strength) → boost (scaled contrast)
            Proto::node("Mul", &[&format!("{}/diff", ns), &format!("{}/strength", ns)],
                &[&format!("{}/boost", ns)], &[]),
            // Add(original, boost) → enhanced
            Proto::node("Add", &[&self.input_source, &format!("{}/boost", ns)],
                &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Strength scalar: controls contrast boost amount
            Proto::tensor_proto_float_scalar(&format!("{}/strength", ns), 0.5),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        // No runtime inputs — strength baked into model
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ldci_id() {
        assert_eq!(LdciBlock::new().id(), "ldci");
    }

    #[test]
    fn test_ldci_emit_onnx() {
        let mut b = LdciBlock::new();
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // AvgPool + Sub + Mul + Add = 4 nodes
        assert_eq!(nodes.len(), 4);
    }
}
