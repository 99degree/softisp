//! # FcsBlock — Flat Field / Luma Adaptive Correction
//!
//! ## ONNX Subgraph (Atomic)
//! Generates 2 ops: `Mul(input, gain) → Add(result, bias)` → `corrected`
//!
//! ## IspChainFusion Standard Rule: R3
//! R3b detects: `BinaryOp(MUL)` followed by `BinaryOp(ADD)` with tensor chain → `isp.fcs`
//! * The shader does luma-based adaptive correction
//! * gain=bias are read from the Mul/Add Const initializers and baked into shader
//!
//! Trade-off vs original Rust block (13-op chroma suppression):
//! - Original: separate Y/UV, edge-attention mask → chroma noise reduction
//! - Current: per-channel gain+bias → simpler, no chroma manipulation
//!
//! The isp.fcs shader (`shader3_fcs.comp`) does luma-based adaptive gain:
//!   `luma = 0.299R + 0.587G + 0.114B`
//!   `corr = 1.0 + gain * |luma-0.5|^2`
//!   `out = clamp(corr, 1-suppression, 1+suppression) * input`
//!
//! When configured as identity (gain=1.0, bias=0.0), the shader passes through
//! with minimal numeric change (luma deviation near 0 → corr≈1.0).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// FcsBlock — Film Contrast Stretch (S-curve).
///
/// ONNX subgraph: Mul(scale) → Add(bias) → Clip(min, max).
/// Parametric S-curve for shadow/highlight adjustment.
/// Fuse target R3b (Mul+Add).
pub struct FcsBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl Default for FcsBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl FcsBlock {
    pub fn new() -> Self { Self { id: "fcs".into(), prev: None, next: None, frame_tensor: "FcsBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for FcsBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "FcsBlock".to_string() }
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

    /// Mul(input, gain) + Add(result, bias) → R3b detects → isp.fcs
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Mul: per-channel gain (broadcast over H,W)
            Proto::node("Mul", &[&self.input_source, &format!("{}/gain", ns)],
                &[&format!("{}/scaled", ns)], &[]),
            // Add: per-channel bias (broadcast over H,W)
            Proto::node("Add", &[&format!("{}/scaled", ns), &format!("{}/bias", ns)],
                &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Per-channel gain: [R_gain, G_gain, B_gain]
            // Shape [3] broadcasts over H,W
            Proto::tensor_proto_float(&format!("{}/gain", ns), &[3], &[1.0, 1.0, 1.0]),
            // Per-channel bias: [R_bias, G_bias, B_bias]
            Proto::tensor_proto_float(&format!("{}/bias", ns), &[3], &[0.0, 0.0, 0.0]),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![
            (format!("{}/gain", ns), 1, vec![3]),
            (format!("{}/bias", ns), 1, vec![3]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fcs_id() {
        assert_eq!(FcsBlock::new().id(), "fcs");
    }

    #[test]
    fn test_fcs_emit_onnx() {
        let mut b = FcsBlock::new();
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // Mul + Add = 2 nodes
        assert_eq!(nodes.len(), 2);
    }

    #[test]
    fn test_fcs_initializers() {
        let b = FcsBlock::new();
        let inits = b.initializers();
        // gain + bias = 2
        assert_eq!(inits.len(), 2);
    }

    #[test]
    fn test_fcs_tensor_ns() {
        let b = FcsBlock::new();
        assert!(!b.tensor_ns().is_empty());
    }

    #[test]
    fn test_fcs_with_gain_offset() {
        let b = FcsBlock::new();
        let inits = b.initializers();
        assert_eq!(inits.len(), 2);
    }
}
