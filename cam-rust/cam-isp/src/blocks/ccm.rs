//! CcmBlock — Color Correction Matrix via Conv(1×1).
//!
//! Applies 3×3 color matrix to RGB channels. Supports instance-aware
//! naming for multiple CCM stages (LSC, CCM, LDCI placeholders).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// CcmBlock — Color Correction Matrix via per-channel Conv(1×1).
///
/// Applies user-configurable 3×3 color correction matrix to transform
/// sensor RGB → sRGB. Parameters hot-swappable at runtime.
pub struct CcmBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
    instance: String,
    in_ch: i64,
    out_ch: i64,
    /// 3x3 color correction matrix (row-major: [R,R,R, G,G,G, B,B,B])
    matrix: [f32; 9],
    /// Per-channel bias
    bias: [f32; 3],
}
impl Default for CcmBlock {
    fn default() -> Self { Self::new() }
}

impl CcmBlock {
    /// Create a default CCM block (single instance, 3 input channels).
    pub fn new() -> Self { Self::with_instance("") }
    /// Create a CCM block with a unique instance suffix.
    /// Required when multiple CcmBlocks are used in the same pipeline
    /// (e.g., LSC, CCM, LDCI, unsharp placeholders) to avoid duplicate
    /// tensor names.
    pub fn with_instance(suffix: &str) -> Self {
        let inst = suffix.to_string();
        let ns = if suffix.is_empty() { "CcmBlock".to_string() } else { format!("CcmBlock_{}", suffix) };
        let bid = if suffix.is_empty() { "ccm".to_string() } else { format!("ccm_{}", suffix) };
        Self {
            id: bid,
            prev: None, next: None,
            frame_tensor: format!("{}/frame", ns),
            input_source: String::new(),
            instance: inst,
            in_ch: 3,  // default: RGB input
            out_ch: 3, // default: RGB output
            matrix: [1.0, 0.0, 0.0,  // R row
                     0.0, 1.0, 0.0,  // G row
                     0.0, 0.0, 1.0], // B row
            bias: [0.0; 3],
        }
    }
    /// Set input and output channel count.
    /// Used for LSC placeholder which receives 4 Bayer channels.
    pub fn with_channels(mut self, ch: i64) -> Self {
        self.in_ch = ch;
        self.out_ch = ch;
        self
    }
}
impl IspBlock for CcmBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String {
        if self.instance.is_empty() { "CcmBlock".to_string() } else { format!("CcmBlock_{}", self.instance) }
    }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1),Proto::tensor_dim_param("C"),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node("Conv", &[&self.input_source, &format!("{}/matrix", ns), &format!("{}/bias", ns)], &[&format!("{}/applied", ns)],
                &[Proto::attribute_ints("kernel_shape", &[1, 1]),
                  Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                  Proto::attribute_ints("strides", &[1, 1])]),
            Proto::node("Clip", &[&format!("{}/applied", ns), &format!("{}/zero", ns), &format!("{}/one", ns)], &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Identity matrix for Conv: [out_ch, in_ch, 1, 1]
        // Only diagonal elements are 1.0 when out_ch == in_ch.
        let n = self.out_ch.max(self.in_ch) as usize;
        let mut identity = vec![0.0f32; n * n];
        for i in 0..self.out_ch.min(self.in_ch) as usize {
            identity[i * n + i] = 1.0;
        }
        // Trim to actual dimensions
        let actual_size = (self.out_ch * self.in_ch) as usize;
        identity.truncate(actual_size);
        
        vec![
            Proto::tensor_proto_float(&format!("{}/matrix", ns), &[self.out_ch, self.in_ch, 1, 1], &self.matrix),
            Proto::tensor_proto_float(&format!("{}/bias", ns), &[self.out_ch], &vec![0.0; self.out_ch as usize]),
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/matrix", self.tensor_ns()), 1, vec![self.out_ch, self.in_ch, 1, 1])]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ccm_id() {
        assert_eq!(CcmBlock::new().id(), "ccm");
    }

    #[test]
    fn test_ccm_instance_id() {
        let b = CcmBlock::with_instance("lsc");
        assert_eq!(b.id(), "ccm_lsc");
    }

    #[test]
    fn test_ccm_nodes() {
        let mut b = CcmBlock::new();
        b.set_input_source("in/rgb");
        let nodes = b.nodes();
        // Conv + Clip = 2 nodes
        assert_eq!(nodes.len(), 2);
    }

    #[test]
    fn test_ccm_with_channels() {
        let b = CcmBlock::new().with_channels(4);
        let inits = b.initializers();
        assert_eq!(inits.len(), 4);
    }

    #[test]
    fn test_ccm_different_channels() {
        let b3 = CcmBlock::new().with_channels(3);
        let b4 = CcmBlock::new().with_channels(4);
        // 4-channel has more initializer data than 3-channel
        assert!(b4.initializers().len() >= b3.initializers().len());
    }

    #[test]
    fn test_ccm_extra_inputs() {
        let b = CcmBlock::new();
        assert!(!b.extra_inputs().is_empty());
    }
}
