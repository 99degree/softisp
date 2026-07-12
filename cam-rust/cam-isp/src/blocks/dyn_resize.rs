//! DynResizeBlock — runtime resolution change within a pipeline.
//!
//! Emits ONNX Resize op to change spatial dimensions at runtime.
//! Useful for adaptive quality: reduce resolution under thermal pressure,
//! or scale up for display.
//!
//! ONNX subgraph:
//!   Resize(input, scales=`[1,1,2,2]`, mode='linear')
//!
//! The scales are stored as initializers and can be hot-swapped via
//! MNN's hot_swap_const_buffer() API.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// DynResizeBlock — dynamic resolution scaling with hot-swappable params.
///
/// ONNX Resize with bilinear interpolation. Scale factors stored as
/// initializers can be updated at runtime via `hot_swap_const_buffer()`
/// without rebuilding the MNN model.
pub struct DynResizeBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub target_w: u32,
    pub target_h: u32,
    /// Source dimensions (set when composing with known input size)
    pub src_w: u32,
    pub src_h: u32,
}

impl DynResizeBlock {
    pub fn new(target_w: u32, target_h: u32) -> Self {
        Self {
            id: "dyn_resize".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "DynResize/frame".into(),
            input_source: String::new(),
            target_w,
            target_h,
            src_w: 0,
            src_h: 0,
        }
    }

    pub fn with_source_size(mut self, w: u32, h: u32) -> Self {
        self.src_w = w;
        self.src_h = h;
        self
    }

    /// Compute scale factors from source to target.
    pub fn scale_factors(&self) -> (f32, f32) {
        if self.src_w == 0 || self.src_h == 0 {
            (1.0, 1.0) // will be computed at compose time
        } else {
            (
                self.target_w as f32 / self.src_w as f32,
                self.target_h as f32 / self.src_h as f32,
            )
        }
    }
}

impl IspBlock for DynResizeBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "DynResize".into()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.into();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev_block.as_ref()
    }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev_block = Some(block);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next_block.as_ref()
    }
    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next_block = Some(block);
    }

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
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(self.target_h as i64),
                Proto::tensor_dim_value(self.target_w as i64),
            ],
            1,
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let scales = format!("{}/scales", ns);
        vec![Proto::node(
            "Resize",
            &[&self.input_source, &scales],
            &[&self.frame_tensor],
            &[Proto::attribute_string("mode", "linear")],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let (sh, sw) = self.scale_factors();
        vec![Proto::tensor_proto_float(
            &format!("{}/scales", ns),
            &[4],
            &[1.0, 1.0, sh, sw],
        )]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dyn_resize_scales() {
        let block = DynResizeBlock::new(960, 540).with_source_size(1920, 1080);
        let (sh, sw) = block.scale_factors();
        assert!((sh - 0.5).abs() < 0.01);
        assert!((sw - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_dyn_resize_emits_resize() {
        let block = DynResizeBlock::new(960, 540).with_source_size(1920, 1080);
        let nodes = block.nodes();
        assert!(nodes
            .iter()
            .any(|n| String::from_utf8_lossy(n).contains("Resize")));
    }

    #[test]
    fn test_dyn_resize_output_shape() {
        let block = DynResizeBlock::new(640, 480);
        let vi = block.output_value_info().unwrap();
        let s = String::from_utf8_lossy(&vi);
        assert!(s.contains("DynResize"));
    }

    #[test]
    fn test_dyn_resize_has_one_input_one_output() {
        let block = DynResizeBlock::new(1280, 720);
        assert_eq!(block.input_tensors().len(), 1);
        assert_eq!(block.output_tensors().len(), 1);
    }

    #[test]
    fn test_dyn_resize_identity() {
        let block = DynResizeBlock::new(1920, 1080).with_source_size(1920, 1080);
        let (sh, sw) = block.scale_factors();
        assert!((sh - 1.0).abs() < 0.01);
        assert!((sw - 1.0).abs() < 0.01);
    }
}
