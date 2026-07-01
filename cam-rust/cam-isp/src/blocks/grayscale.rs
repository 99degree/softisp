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
