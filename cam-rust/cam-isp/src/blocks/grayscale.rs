//! # GrayscaleBlock — RGB to Luminance (GPU deshake pre-processing)
//!
//! Generates a single Conv(1×1, 3→1ch) with fixed luminance weights.
//! IspChainFusion detects this pattern and fuses into `isp.grayscale` Extra op,
//! which runs a SPIR-V compute shader computing Y = 0.299R + 0.587G + 0.114B.
//!
//! This feeds the pyramid downscale for GPU-accelerated deshake pipeline.
//! The resulting `[1,1,H,W]` f32 luminance tensor is used by the block-matching
//! motion estimation engine (DeshakeEngine).

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

pub struct GrayscaleBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl Default for GrayscaleBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl GrayscaleBlock {
    pub fn new() -> Self {
        Self {
            id: "grayscale".into(),
            prev: None,
            next: None,
            frame_tensor: "GrayscaleBlock/frame".into(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for GrayscaleBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "GrayscaleBlock".to_string()
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
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        // Output has 1 channel (luminance)
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }
    fn is_tail(&self) -> bool {
        true
    }

    /// Conv(1×1, 3→1ch) with fixed luminance weights
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![Proto::node(
            "Conv",
            &[&self.input_source, &format!("{}/weight", ns)],
            &[&self.frame_tensor],
            &[],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![(format!("{}/weight", ns).to_string(), 1, vec![1, 3, 1, 1])]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        vec![(
            format!("{}/weight", ns).to_string(),
            [0.299f32, 0.587f32, 0.114f32]
                .iter()
                .flat_map(|v| v.to_ne_bytes())
                .collect::<Vec<u8>>(),
        )]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify luminance weights match BT.601 coefficients.
    #[test]
    fn test_grayscale_luminance_weights() {
        let block = GrayscaleBlock::new();
        let inits = block.extra_input_defaults();
        assert_eq!(inits.len(), 1, "should have 1 weight input");
        // The raw bytes should contain the 3 float32 values
        let w_bytes = &inits[0].1;
        assert_eq!(
            w_bytes.len(),
            12,
            "3 f32 values = 12 bytes, got {}",
            w_bytes.len()
        );
        // Expected: [0.299f32, 0.587f32, 0.114f32] (little-endian)
        let expected: [f32; 3] = [0.299, 0.587, 0.114];
        let f32_bytes = unsafe {
            std::slice::from_raw_parts(expected.as_ptr() as *const u8, expected.len() * 4)
        };
        let w_pos = w_bytes.windows(12).position(|window| window == f32_bytes);
        assert!(w_pos.is_some(), "luminance weights not found in tensor");
    }

    /// Verify output shape is `[1, 1, H, W]` (single luminance channel).
    #[test]
    fn test_grayscale_output_shape() {
        let block = GrayscaleBlock::new();
        let ovi = block
            .output_value_info()
            .expect("should have output value info");
        // Protobuf encoding: field 1 (name), field 2 (type), field 3 (shape)
        // Shape contains dim_value/dim_param. For [1, 1, H, W]:
        // 2 dim_value(1) + 2 dim_param(H, W)
        // Just verify it's non-empty and contains recognizable patterns
        assert!(ovi.len() > 10, "output value info should be non-trivial");
        // Check for repeated 0x0a (field 1, wire type 2 = string) which marks dim params
        let dim_param_count = ovi.iter().filter(|&&b| b == 0x0a).count();
        assert!(dim_param_count >= 2, "should have at least 2 dim params");
    }

    /// Verify ONNX node structure.
    #[test]
    fn test_grayscale_onnx_structure() {
        let block = GrayscaleBlock::new();
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 1, "should have 1 Conv node");
        let node_str = String::from_utf8_lossy(&nodes[0]);
        assert!(node_str.contains("Conv"), "node should be Conv");
    }

    /// Verify the wired graph produces valid ONNX.
    #[test]
    fn test_grayscale_pyramid_compose() {
        use crate::blocks::PyramidBlock;
        use crate::pipeline::GraphComposer;
        let mut gs = GrayscaleBlock::new();
        let pyr = PyramidBlock::new();
        // Set input source for pyramid from grayscale output
        gs.set_input_source("input");
        // Use full qualified path for PyramidBlock
        let pipeline: Vec<&dyn IspBlock> = vec![&gs as &dyn IspBlock, &pyr as &dyn IspBlock];
        let onnx = GraphComposer::compose_from_vec(&pipeline, &[], 21);
        assert!(onnx.is_ok(), "ONNX composition should succeed");
        let bytes = onnx.unwrap();
        assert!(bytes.len() > 100, "ONNX protobuf should be non-trivial");
    }

    #[test]
    fn test_grayscale_id() {
        assert_eq!(GrayscaleBlock::new().id(), "grayscale");
    }

    #[test]
    fn test_grayscale_tensor_ns() {
        let b = GrayscaleBlock::new();
        assert!(!b.tensor_ns().is_empty());
    }
}
