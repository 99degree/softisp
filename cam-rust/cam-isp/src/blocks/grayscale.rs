//! # GrayscaleBlock — RGB to Luminance (GPU deshake pre-processing)
//!
//! Generates a single Conv(1×1, 3→1ch) with fixed luminance weights.
//! IspChainFusion detects this pattern and fuses into `isp.grayscale` Extra op,
//! which runs a SPIR-V compute shader computing Y = 0.299R + 0.587G + 0.114B.
//!
//! This feeds the pyramid downscale for GPU-accelerated deshake pipeline.
//! The resulting [1,1,H,W] f32 luminance tensor is used by the block-matching
//! motion estimation engine (DeshakeEngine).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct GrayscaleBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
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
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "GrayscaleBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.input_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        // Output has 1 channel (luminance)
        Some(Proto::value_info(&self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    fn is_tail(&self) -> bool { true }

    /// Conv(1×1, 3→1ch) with fixed luminance weights
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node("Conv", &[&self.input_source, &format!("{}/weight", ns)],
                &[&self.frame_tensor], &[]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // BT.601 luminance weights: Y = 0.299R + 0.587G + 0.114B
        // Weight shape: [1, 3, 1, 1] = [oc, ic, kh, kw]
        let lum_w = [0.299f32, 0.587f32, 0.114f32];
        vec![
            Proto::tensor_proto_float(&format!("{}/weight", ns), &[1, 3, 1, 1], &lum_w),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify luminance weights match BT.601 coefficients.
    #[test]
    fn test_grayscale_luminance_weights() {
        let block = GrayscaleBlock::new();
        let inits = block.initializers();
        assert_eq!(inits.len(), 1, "should have 1 initializer (weight)");
        // Parse the weight tensor
        let w_bytes = &inits[0];
        // Verify it starts with a valid ONNX tensor proto (field 1 = name)
        assert!(w_bytes.len() > 20, "weight tensor should be non-trivial");
        // The weight floats are written via Proto::tensor_proto_float
        // Expected: [0.299f32, 0.587f32, 0.114f32] (little-endian)
        // In raw_data (field 9), they appear as 12 bytes of float32 data
        // Let's find them by looking at the float values
        let expected: [f32; 3] = [0.299, 0.587, 0.114];
        let f32_bytes = unsafe {
            std::slice::from_raw_parts(
                expected.as_ptr() as *const u8,
                expected.len() * 4,
            )
        };
        // The floats are in raw_data which is the last part of the protobuf
        // Check that all expected float bytes appear in order
        let w_pos = w_bytes.windows(12)
            .position(|window| window == f32_bytes);
        assert!(w_pos.is_some(), "luminance weights not found in tensor");
    }

    /// Verify output shape is [1, 1, H, W] (single luminance channel).
    #[test]
    fn test_grayscale_output_shape() {
        let block = GrayscaleBlock::new();
        let ovi = block.output_value_info().expect("should have output value info");
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
        use crate::pipeline::GraphComposer;
        use crate::blocks::PyramidBlock;
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
        let s = String::from_utf8_lossy(&bytes);
        assert!(s.contains("Conv"), "ONNX should contain Conv op");
        assert!(s.contains("GrayscaleBlock"), "ONNX should contain grayscale tensor names");
        assert!(s.contains("PyramidBlock"), "ONNX should contain pyramid tensor names");
    }
}