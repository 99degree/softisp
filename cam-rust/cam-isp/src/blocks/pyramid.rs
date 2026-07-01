//! # PyramidBlock — 2× Nearest-Neighbor Downscale (GPU deshake pre-processing)
//!
//! Generates a single Conv(2×2, stride=2) with identity weight (top-left=1.0).
//! IspChainFusion detects this and fuses into `isp.pyramid` Extra op,
//! which runs a SPIR-V compute shader doing nearest-neighbor 2× downscale:
//!   `out[oy, ox] = input[2*oy, 2*ox]` per channel
//!
//! This reduces the luminance map resolution for the block-matching search
//! in the DeshakeEngine, enabling faster motion estimation.

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct PyramidBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl PyramidBlock {
    pub fn new() -> Self {
        Self {
            id: "pyramid".into(),
            prev: None,
            next: None,
            frame_tensor: "PyramidBlock/frame".into(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for PyramidBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "PyramidBlock".to_string() }
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
        // Output has same channels as input, but half H and W
        Some(Proto::value_info(&self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    fn is_tail(&self) -> bool { true }

    /// Conv(2×2, stride=2, oc=ic=3) with identity weight (top-left=1.0)
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node("Conv", &[&self.input_source, &format!("{}/weight", ns)],
                &[&self.frame_tensor], &[]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Nearest-neighbor 2× downscale: pick top-left pixel
        // Weight shape: [3, 3, 2, 2] — identity for each input→output channel
        // weight[oc][ic][y][x] = 1.0 if oc==ic && y==0 && x==0, else 0.0
        let mut w = vec![0.0f32; 3 * 3 * 2 * 2];
        for oc in 0..3 {
            let ic = oc;  // identity: each output channel reads from same input channel
            let idx = oc * 3 * 2 * 2 + ic * 2 * 2;  // position (oc, ic, 0, 0) = top-left
            w[idx] = 1.0;
        }
        vec![
            Proto::tensor_proto_float(&format!("{}/weight", ns), &[3, 3, 2, 2], &w),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify pyramid identity weights: top-left = 1.0 per output channel.
    /// Weight shape: [3, 3, 2, 2], position (oc, oc, 0, 0) = 1.0
    #[test]
    fn test_pyramid_identity_weights() {
        let block = PyramidBlock::new();
        let inits = block.initializers();
        assert_eq!(inits.len(), 1, "should have 1 initializer (weight)");
        let w_bytes = &inits[0];
        // Weight has 3*3*2*2 = 36 float32 values = 144 bytes of raw_data
        // Each output channel oc reads from its corresponding input channel ic
        // weight[oc][ic][y][x] = 1.0 if oc==ic && y==0 && x==0
        // Build the expected weight array
        let mut expected = vec![0.0f32; 36];
        for oc in 0..3 {
            let idx = oc * 3 * 2 * 2 + oc * 2 * 2;  // (oc, oc, 0, 0)
            expected[idx] = 1.0;
        }
        let expected_bytes = unsafe {
            std::slice::from_raw_parts(
                expected.as_ptr() as *const u8,
                expected.len() * 4,
            )
        };
        let w_pos = w_bytes.windows(expected_bytes.len())
            .position(|window| window == expected_bytes);
        assert!(w_pos.is_some(), "identity weights not found in tensor");
    }

    /// Verify output shape preserves channels [1, 3, H, W].
    #[test]
    fn test_pyramid_output_shape() {
        let block = PyramidBlock::new();
        let ovi = block.output_value_info().expect("should have output value info");
        assert!(ovi.len() > 10, "output value info should be non-trivial");
        let dim_param_count = ovi.iter().filter(|&&b| b == 0x0a).count();
        assert!(dim_param_count >= 2, "should have at least 2 dim params");
    }

    /// Verify ONNX node is Conv.
    #[test]
    fn test_pyramid_onnx_structure() {
        let block = PyramidBlock::new();
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 1, "should have 1 Conv node");
        let node_str = String::from_utf8_lossy(&nodes[0]);
        assert!(node_str.contains("Conv"), "node should be Conv");
    }

    /// Verify Grayscale→Pyramid composition produces valid ONNX.
    #[test]
    fn test_pyramid_compose_onnx() {
        use crate::pipeline::GraphComposer;
        use crate::blocks::GrayscaleBlock;
        let mut gs = GrayscaleBlock::new();
        let pyr = PyramidBlock::new();
        gs.set_input_source("input");
        let pipeline: Vec<&dyn IspBlock> = vec![&gs as &dyn IspBlock, &pyr as &dyn IspBlock];
        let onnx = GraphComposer::compose_from_vec(&pipeline, &[], 21);
        assert!(onnx.is_ok(), "ONNX composition should succeed");
        let bytes = onnx.unwrap();
        assert!(bytes.len() > 200, "ONNX protobuf should be non-trivial");
        let s = String::from_utf8_lossy(&bytes);
        assert!(s.contains("Conv"), "ONNX should contain Conv nodes");
        assert!(s.contains("GrayscaleBlock"), "should contain grayscale tensor");
        assert!(s.contains("PyramidBlock"), "should contain pyramid tensor");
    }
}