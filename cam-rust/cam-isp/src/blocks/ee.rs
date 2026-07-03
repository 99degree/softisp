//! # EeBlock — Edge Enhancement (sharpening)
//!
//! ## ONNX Subgraph (Atomic)
//! Generates a SINGLE op: `Conv(3×3, group=3, unsharp_kernel)` → `edge`
//!
//! ## IspChainFusion Standard Rule: R4
//! R4 detects: `Conv(3×3, unsharp_kernel, outputCount=3)` → `isp.ee`
//! * kernel_shape=[3,3], strides=[1,1], pads=[1,1,1,1]
//! * unsharp weights: `{0, -0.5, 0, -0.5, 3.0, -0.5, 0, -0.5, 0}` per channel
//! * inputCount=3, outputCount=3 (per-channel edge detection)
//!
//! No chain consumption needed: the `isp.ee` SPIR-V shader does the entire
//! edge detection + add-back computation in one dispatch, replacing the
//! Conv + Mul(gain) + Add(input) + Clip chain from the original Rust block.
//!
//! ### Intended runtime shader: `isp.ee`
//! * SPIR-V `shader4_ee.comp` (embedded as `g_ee_spv`)
//! * Const buffer: `[W, H, strength=0.5, threshold=0.01]`
//! * Input: F32[1,3,H,W] RGB (original frame)
//! * Output: F32[1,3,H,W] RGB (edge-enhanced)
//! * Algorithm: `laplacian = 4*center - (t+b+l+r)`, `output = center + strength*laplacian`
//!
//! Trade-off vs original Rust block:
//! - Original: 3×5 laplacian + Y-only + per-frame gain → more flexible but 6 ops
//! - Current: 3×3 per-channel laplacian + baked strength → 1 op, fused by R4
//! Run-time parameterized strength requires modifying the shader's const buffer
//! through MNN's session input API (not currently implemented).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct EeBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl EeBlock {
    pub fn new() -> Self { Self { id: "ee".into(), prev: None, next: None, frame_tensor: "EeBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for EeBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "EeBlock".to_string() }
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

    /// Single Conv(3×3, group=3, unsharp) matching R4 → isp.ee
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // R4 detects: Conv(3×3, unsharp kernel, outputCount=3)
            // pads=[1,1,1,1] for 3×3 same-size convolution
            Proto::node("Conv", &[&self.input_source, &format!("{}/kernel_ee", ns), &format!("{}/bias_ee", ns)],
                &[&self.frame_tensor],
                &[Proto::attribute_ints("kernel_shape", &[3, 3]),
                  Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                  Proto::attribute_ints("strides", &[1, 1]),
                  Proto::attribute_int("group", 3)]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Unsharp kernel: center=3.0, neighbors=-0.5 (sum=1 → no DC shift)
        // R4 checks: first 9 weights must match {0,-0.5,0,-0.5,3,-0.5,0,-0.5,0}
        // For depthwise (group=3): weights shape [3, 1, 3, 3] = 9 floats per channel
        // All 3 channels use the same unsharp kernel (identical per-channel behavior)
        let k_unsharp: [f32; 9] = [0.0, -0.5, 0.0, -0.5, 3.0, -0.5, 0.0, -0.5, 0.0];
        let mut k = Vec::with_capacity(27);
        for _ in 0..3 { k.extend_from_slice(&k_unsharp); }
        vec![
            Proto::tensor_proto_float(&format!("{}/kernel_ee", ns), &[3, 1, 3, 3], &k),
            Proto::tensor_proto_float(&format!("{}/bias_ee", ns), &[3], &[0.0; 3]),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        // No runtime inputs — strength is baked in shader const buffer
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ee_id() {
        let b = EeBlock::new();
        assert_eq!(b.id(), "ee");
    }

    #[test]
    fn test_ee_tensors() {
        let mut b = EeBlock::new();
        b.set_input_source("prev/frame");
        assert_eq!(b.input_tensors(), vec!["prev/frame".to_string()]);
        assert_eq!(b.output_tensors(), vec!["EeBlock/frame".to_string()]);
    }

    #[test]
    fn test_ee_emit_onnx() {
        let mut b = EeBlock::new();
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // Conv(3x3 unsharp) = 1 node
        assert_eq!(nodes.len(), 1);
    }

    #[test]
    fn test_ee_kernel_shape() {
        let b = EeBlock::new();
        let inits = b.initializers();
        // kernel + bias = 2
        assert_eq!(inits.len(), 2);
    }
}
