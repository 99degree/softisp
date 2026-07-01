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
