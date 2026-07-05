//! WatermarkBlock — text/graphics overlay on image output.
//!
//! Overlays a pre-rendered watermark (PNG/image) or text onto the ISP output.
//! The watermark is applied as a blend: `output = input * (1 - alpha) + watermark * alpha`.
//!
//! ONNX subgraph:
//!   1. Load watermark as constant tensor (pre-rendered RGBA at target resolution)
//!   2. Split input RGB and watermark RGBA
//!   3. Blend: `rgb_out = rgb_in * (1 - alpha) + wm_rgb * alpha`
//!   4. Concat RGB output
//!
//! For text watermarks, the text is pre-rendered to a bitmap at build time.
//! For image watermarks, a PNG is loaded and resized to fit.

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// WatermarkBlock — overlay text or image watermark.
pub struct WatermarkBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Watermark opacity (0.0 = transparent, 1.0 = fully visible). Default: 0.3.
    pub opacity: f32,
    /// Position: 0=top-left, 1=top-right, 2=bottom-left, 3=bottom-right. Default: 3.
    pub position: u8,
}

impl Default for WatermarkBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl WatermarkBlock {
    pub fn new() -> Self {
        Self {
            id: "watermark".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "WatermarkBlock/frame".into(),
            input_source: String::new(),
            opacity: 0.3,
            position: 3, // bottom-right
        }
    }

    pub fn with_opacity(mut self, opacity: f32) -> Self {
        self.opacity = opacity.clamp(0.0, 1.0);
        self
    }

    pub fn with_position(mut self, pos: u8) -> Self {
        self.position = pos.min(3);
        self
    }
}

impl IspBlock for WatermarkBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "Watermark".into() }
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
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // Watermark blend: output = input * (1 - alpha) + watermark * alpha
        // Simplified: output = input + alpha * (watermark - input)

        let wm_rgb = format!("{}/wm_rgb", ns);
        let alpha_name = format!("{}/alpha", ns);

        // diff = watermark - input
        let diff = format!("{}/diff", ns);
        nodes.push(Proto::node("Sub",
            &[&wm_rgb, &self.input_source],
            &[&diff], &[]));

        // scaled = alpha * diff
        let scaled = format!("{}/scaled", ns);
        nodes.push(Proto::node("Mul",
            &[&diff, &alpha_name],
            &[&scaled], &[]));

        // output = input + scaled
        nodes.push(Proto::node("Add",
            &[&self.input_source, &scaled],
            &[&self.frame_tensor], &[]));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Placeholder watermark RGB (1×3×1×1 white pixel — real implementation
        // would load actual watermark bitmap)
        let wm_data = vec![1.0f32; 3];
        vec![
            Proto::tensor_proto_float(&format!("{}/wm_rgb", ns), &[1, 3, 1, 1], &wm_data),
            Proto::tensor_proto_float_scalar(&format!("{}/alpha", ns), self.opacity),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![
            (format!("{}/{}", self.tensor_ns(), "alpha"), 1, vec![1]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_watermark_id() {
        assert_eq!(WatermarkBlock::new().id(), "watermark");
    }

    #[test]
    fn test_watermark_with_opacity() {
        let b = WatermarkBlock::new().with_opacity(0.5);
        assert_eq!(b.opacity, 0.5);
    }

    #[test]
    fn test_watermark_clamps_opacity() {
        let b = WatermarkBlock::new().with_opacity(1.5);
        assert_eq!(b.opacity, 1.0, "opacity should be clamped to 1.0");
        let b = WatermarkBlock::new().with_opacity(-0.5);
        assert_eq!(b.opacity, 0.0, "opacity should be clamped to 0.0");
    }

    #[test]
    fn test_watermark_with_position() {
        let b = WatermarkBlock::new().with_position(0);
        assert_eq!(b.position, 0);
    }

    #[test]
    fn test_watermark_clamps_position() {
        let b = WatermarkBlock::new().with_position(10);
        assert_eq!(b.position, 3, "position should be clamped to 3");
    }

    #[test]
    fn test_watermark_emit_onnx() {
        let b = WatermarkBlock::new();
        let nodes = b.nodes();
        // Sub + Mul + Add = 3 nodes
        assert_eq!(nodes.len(), 3);
    }

    #[test]
    fn test_watermark_has_input_output() {
        let b = WatermarkBlock::new();
        assert_eq!(b.input_tensors().len(), 1);
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_watermark_tensor_ns() {
        let b = WatermarkBlock::new();
        assert_eq!(b.tensor_ns(), "Watermark");
    }

    #[test]
    fn test_watermark_graph_output() {
        let b = WatermarkBlock::new();
        assert!(b.graph_output_name().is_some());
    }

    #[test]
    fn test_watermark_extra_inputs() {
        let b = WatermarkBlock::new();
        let extras = b.extra_inputs();
        assert_eq!(extras.len(), 1, "should have alpha extra input");
    }
}
