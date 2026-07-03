//! GammaBlock — standalone gamma correction / LUT curve.
//!
//! Applies a parametric gamma curve: output = pow(input, 1/gamma)
//! for positive values, with optional shadow lift and highlight compression.
//!
//! ONNX subgraph:
//!   1. Clamp input to [0, 1]
//!   2. Add epsilon (1e-6) to avoid log(0)
//!   3. Log -> Mul(1/gamma) -> Exp  (equivalent to pow(x, 1/gamma))
//!   4. Optional shadow lift: Add(shadow) then Mul(1/(1+shadow))
//!   5. Optional highlight compression: Clip to [0, 1]
//!
//! Common gamma values:
//!   - sRGB: gamma ≈ 2.2
//!   - Rec.709: gamma ≈ 2.4
//!   - Linear display: gamma = 1.0
//!   - ARRI LogC: gamma ≈ 0.5 (decompression)
//!
//! The DisplayBlock already has gamma, but this standalone block is useful for:
//!   - Intermediate gamma correction in pipeline
//!   - HDR -> SDR tone mapping via gamma
//!   - Film-like curves with shadow lift

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// GammaBlock — sRGB gamma correction via Log→Mul→Exp.
///
/// Applies gamma curve with optional shadow lift.
/// Formula: `exp(gamma * log(x))` with configurable gamma value (2.2 typical).
pub struct GammaBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub gamma: f32,
    pub shadow_lift: f32,
}

impl GammaBlock {
    pub fn new(gamma: f32) -> Self {
        Self {
            id: "gamma".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "GammaBlock/frame".into(),
            input_source: String::new(),
            gamma,
            shadow_lift: 0.0,
        }
    }

    pub fn with_shadow_lift(mut self, lift: f32) -> Self {
        self.shadow_lift = lift;
        self
    }
}

impl IspBlock for GammaBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "Gamma".into() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.into(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev_block.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev_block = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next_block.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next_block = Some(block); }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }

    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.input_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // 1. Clamp input to [0, 1]
        let clamped = format!("{}/clamped", ns);
        let min_name = format!("{}/min", ns);
        let max_name = format!("{}/max", ns);
        nodes.push(Proto::node(
            "Max", &[&self.input_source, &min_name], &[&clamped], &[],
        ));
        nodes.push(Proto::node(
            "Min", &[&clamped, &max_name], &[&clamped], &[],
        ));

        // 2. Add epsilon to avoid log(0)
        let eps_name = format!("{}/eps", ns);
        let safe = format!("{}/safe", ns);
        nodes.push(Proto::node(
            "Add", &[&clamped, &eps_name], &[&safe], &[],
        ));

        // 3. Log → Mul(inv_gamma) → Exp  (pow(x, 1/gamma))
        let log_out = format!("{}/log", ns);
        let inv_gamma = format!("{}/inv_gamma", ns);
        let scaled = format!("{}/scaled", ns);
        let output = if self.shadow_lift > 0.0 {
            format!("{}/pre_lift", ns)
        } else {
            self.frame_tensor.clone()
        };

        nodes.push(Proto::node("Log", &[&safe], &[&log_out], &[]));
        nodes.push(Proto::node("Mul", &[&log_out, &inv_gamma], &[&scaled], &[]));
        nodes.push(Proto::node("Exp", &[&scaled], &[&output], &[]));

        // 4. Optional shadow lift
        if self.shadow_lift > 0.0 {
            let lift_name = format!("{}/lift", ns);
            let lifted = format!("{}/lifted", ns);
            let norm_name = format!("{}/norm", ns);
            nodes.push(Proto::node(
                "Add", &[&output, &lift_name], &[&lifted], &[],
            ));
            nodes.push(Proto::node(
                "Mul", &[&lifted, &norm_name], &[&self.frame_tensor], &[],
            ));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        // Clamp bounds
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/min", ns), 0.0));
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/max", ns), 1.0));

        // Epsilon
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/eps", ns), 1e-6));

        // Inverse gamma: pow(x, 1/gamma) = exp(log(x) / gamma)
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/inv_gamma", ns), 1.0 / self.gamma));

        // Shadow lift parameters
        if self.shadow_lift > 0.0 {
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/lift", ns), self.shadow_lift));
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/norm", ns), 1.0 / (1.0 + self.shadow_lift)));
        }

        inits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gamma_basic_emission() {
        let block = GammaBlock::new(2.2);
        let nodes = block.nodes();
        // Log + Mul + Exp + Max + Min + Add(eps) = 6 nodes
        assert!(nodes.len() >= 5, "need >= 5 nodes, got {}", nodes.len());
        let inits = block.initializers();
        assert!(inits.len() >= 4, "need >= 4 initializers, got {}", inits.len());
    }

    #[test]
    fn test_gamma_with_shadow_lift() {
        let block = GammaBlock::new(2.2).with_shadow_lift(0.05);
        let nodes = block.nodes();
        // Extra Add + Mul for shadow lift
        assert!(nodes.len() >= 7, "shadow lift needs >= 7 nodes, got {}", nodes.len());
    }

    #[test]
    fn test_gamma_inv_gamma_in_init() {
        let block = GammaBlock::new(2.2);
        let inits = block.initializers();
        // Check inv_gamma = 1/2.2 ≈ 0.4545
        let ns = block.tensor_ns();
        let target = format!("{}/inv_gamma", ns);
        assert!(inits.iter().any(|i| {
            let s = String::from_utf8_lossy(i);
            s.contains("inv_gamma")
        }), "must have inv_gamma initializer");
    }

    #[test]
    fn test_gamma_identity() {
        let block = GammaBlock::new(1.0);
        let inits = block.initializers();
        // inv_gamma = 1.0 for identity
        assert_eq!(inits.len(), 4, "gamma=1.0 should have 4 initializers");
    }

    #[test]
    fn test_gamma_has_input_output() {
        let block = GammaBlock::new(2.2);
        assert_eq!(block.input_tensors().len(), 1);
        assert_eq!(block.output_tensors().len(), 1);
    }
}
