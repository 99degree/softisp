//! SharpenBlock — unsharp mask sharpening.
//!
//! Enhances edges by subtracting a blurred version of the image.
//! ONNX subgraph:
//!   1. GaussianBlur: AvgPool(3×3) → blurred
//!   2. Diff = input - blurred
//!   3. output = input + strength * Diff
//!
//! Default strength: 0.5 (moderate sharpening).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// SharpenBlock — unsharp mask enhancement.
///
/// ONNX subgraph: AvgPool → Sub → Mul(strength) → Add(input).
/// Strength: 0.0=no effect, 0.5=moderate, 1.0=aggressive.
pub struct SharpenBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub strength: f32,
}

impl SharpenBlock {
    pub fn new(strength: f32) -> Self {
        Self {
            id: "sharpen".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "SharpenBlock/frame".into(),
            input_source: String::new(),
            strength,
        }
    }
}

impl IspBlock for SharpenBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "Sharpen".into() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.into(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev_block.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev_block = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next_block.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next_block = Some(block); }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn graph_output_name(&self) -> Option<&str> { Some(&self.frame_tensor) }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.input_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // 1. Blur: AvgPool(3×3) with padding
        let blurred = format!("{}/blurred", ns);
        nodes.push(Proto::node(
            "AveragePool", &[&self.input_source], &[&blurred],
            &[Proto::attribute_ints("kernel_shape", &[3, 3]),
              Proto::attribute_ints("pads", &[1, 1, 1, 1])],
        ));

        // 2. Diff = input - blur
        let diff = format!("{}/diff", ns);
        nodes.push(Proto::node("Sub", &[&self.input_source, &blurred], &[&diff], &[]));

        // 3. Scaled diff = strength * diff
        let strength_name = format!("{}/strength", ns);
        let scaled = format!("{}/scaled", ns);
        nodes.push(Proto::node("Mul", &[&diff, &strength_name], &[&scaled], &[]));

        // 4. Output = input + scaled
        nodes.push(Proto::node("Add", &[&self.input_source, &scaled], &[&self.frame_tensor], &[]));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![Proto::tensor_proto_float_scalar(
            &format!("{}/strength", self.tensor_ns()), self.strength)]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/strength", self.tensor_ns()), 1, vec![1])]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sharpen_emits_avgpool_sub_mul_add() {
        let block = SharpenBlock::new(0.5);
        let nodes = block.nodes();
        let tags = ["AveragePool", "Sub", "Mul", "Add"];
        for tag in &tags {
            assert!(nodes.iter().any(|n| String::from_utf8_lossy(n).contains(tag)),
                "must emit {}", tag);
        }
    }

    #[test]
    fn test_sharpen_has_one_input_one_output() {
        let block = SharpenBlock::new(0.3);
        assert_eq!(block.input_tensors().len(), 1);
        assert_eq!(block.output_tensors().len(), 1);
    }

    #[test]
    fn test_sharpen_has_initializer() {
        let block = SharpenBlock::new(0.7);
        assert_eq!(block.initializers().len(), 1);
    }

    #[test]
    fn test_sharpen_id() {
        assert_eq!(SharpenBlock::new(0.5).id(), "sharpen");
    }

    #[test]
    fn test_sharpen_tensor_ns() {
        let b = SharpenBlock::new(0.5);
        assert!(!b.tensor_ns().is_empty());
    }

    #[test]
    fn test_sharpen_various_strengths() {
        for s in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let b = SharpenBlock::new(s);
            let nodes = b.nodes();
            assert_eq!(nodes.len(), 4, "strength={}: AvgPool + Sub + Mul + Add", s);
        }
    }

    #[test]
    fn test_sharpen_extreme_strengths() {
        // Zero strength — identity (no sharpening)
        let b = SharpenBlock::new(0.0);
        assert_eq!(b.strength, 0.0);
        // Very high strength
        let b = SharpenBlock::new(100.0);
        assert_eq!(b.strength, 100.0);
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 4);
    }
}
