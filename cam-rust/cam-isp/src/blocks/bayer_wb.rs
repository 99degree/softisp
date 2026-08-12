//! BayerWbBlock — Bayer-domain white balance gain.
//!
//! Multiplies each Bayer channel by its white balance gain.
//! 4-channel multiplication (RGGB pattern order).

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// BayerWbBlock — per-channel white balance gain for Bayer data.
///
/// Applies WB gains: `out_ch = in_ch * gain[ch]` for each of 4 Bayer
/// positions (R, G1, G2, B). Gains are hot-swappable at runtime.
///
/// When gains are identity (all 1.0), the block emits a passthrough
/// Identity node to avoid breaking MNN's ISP fusion chain.
pub struct BayerWbBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    gains: [f32; 4],
}
impl Default for BayerWbBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl BayerWbBlock {
    pub fn new() -> Self {
        Self {
            id: "bayer_wb".into(),
            prev: None,
            next: None,
            frame_tensor: "BayerWbBlock/frame".into(),
            input_source: String::new(),
            gains: [1.0; 4],
        }
    }
    /// Set WB gains. Identity gains (all 1.0) cause the block to emit
    /// a passthrough Identity, preserving MNN ISP fusion compatibility.
    pub fn with_gains(mut self, r: f32, g1: f32, g2: f32, b: f32) -> Self {
        self.gains = [r, g1, g2, b];
        self
    }
    /// Set gains at runtime (mutates existing block).
    pub fn set_gains(&mut self, r: f32, g1: f32, g2: f32, b: f32) {
        self.gains = [r, g1, g2, b];
    }

    /// Get current gains.
    pub fn gains(&self) -> [f32; 4] {
        self.gains
    }

    /// Returns true when gains are identity (no-op).
    fn is_identity_gains(&self) -> bool {
        self.gains.iter().all(|g| (*g - 1.0f32).abs() < 1e-6)
    }
}
impl IspBlock for BayerWbBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "BayerWbBlock".to_string()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev.as_ref()
    }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev = Some(block);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next.as_ref()
    }
    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next = Some(block);
    }
    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }
    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.input_source,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        if self.is_identity_gains() {
            // Identity passthrough — avoids breaking MNN ISP fusion chain
            return vec![Proto::node(
                "Identity",
                &[&self.input_source],
                &[&self.frame_tensor],
                &[],
            )];
        }
        vec![
            Proto::node(
                "Mul",
                &[&self.input_source, &format!("{}/gains", ns)],
                &[&format!("{}/wb_applied", ns)],
                &[],
            ),
            Proto::node(
                "Clip",
                &[
                    &format!("{}/wb_applied", ns),
                    &format!("{}/zero", ns),
                    &format!("{}/one", ns),
                ],
                &[&self.frame_tensor],
                &[],
            ),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        if self.is_identity_gains() {
            return vec![]; // Identity passthrough needs no initializers
        }
        vec![
            // gains is a runtime input (fed by the engine per-frame); only
            // the Clip bounds are baked.
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        if self.is_identity_gains() {
            return vec![]; // Identity passthrough has no extra inputs
        }
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
        // Identity gains → 1 passthrough node
        assert_eq!(nodes.len(), 1);
    }

    #[test]
    fn test_bayer_wb_nodes_with_gains() {
        let mut b = BayerWbBlock::new().with_gains(1.2, 1.0, 1.0, 0.8);
        b.set_input_source("in/bayer");
        let nodes = b.nodes();
        // Non-identity gains → Mul + Clip = 2 nodes
        assert_eq!(nodes.len(), 2);
    }

    #[test]
    fn test_bayer_wb_initializers() {
        let b = BayerWbBlock::new();
        let inits = b.initializers();
        // Identity gains → no initializers
        assert_eq!(inits.len(), 0);
    }

    #[test]
    fn test_bayer_wb_initializers_with_gains() {
        let b = BayerWbBlock::new().with_gains(1.2, 1.0, 1.0, 0.8);
        let inits = b.initializers();
        // Non-identity gains → gains is a runtime input; only zero + one baked
        assert_eq!(inits.len(), 2, "gains is a runtime input now");
        assert_eq!(b.extra_inputs().len(), 1, "gains declared as extra input");
    }

    #[test]
    fn test_bayer_wb_extra_inputs() {
        let b = BayerWbBlock::new();
        let extra = b.extra_inputs();
        // Identity gains → no extra inputs
        assert_eq!(extra.len(), 0);
    }

    #[test]
    fn test_bayer_wb_extra_inputs_with_gains() {
        let b = BayerWbBlock::new().with_gains(1.2, 1.0, 1.0, 0.8);
        let extra = b.extra_inputs();
        assert_eq!(extra.len(), 1);
        assert_eq!(extra[0].2, vec![1, 4, 1, 1]);
    }

    #[test]
    fn test_bayer_wb_has_input_output() {
        let b = BayerWbBlock::new();
        assert!(!b.input_tensors().is_empty());
        assert!(!b.output_tensors().is_empty());
    }

    #[test]
    fn test_bayer_wb_tensor_ns() {
        let b = BayerWbBlock::new();
        assert!(!b.tensor_ns().is_empty());
    }
}
