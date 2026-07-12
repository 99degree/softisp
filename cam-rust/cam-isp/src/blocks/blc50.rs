//! Blc50Block — 50Hz fixed pattern noise (FPN) subtraction.
//!
//! Subtracts a pre-captured dark frame to remove 50Hz power line noise.
//! Common in industrial and surveillance cameras where 50Hz flicker
//! creates fixed patterns in the raw sensor data.
//!
//! ONNX subgraph:
//!   1. Load dark frame as constant tensor
//!   2. Sub(input, dark_frame)
//!   3. Clip to [0, max_val]
//!
//! The dark frame is captured once during calibration and stored as an
//! initializer tensor. The subtraction is per-pixel, per-channel.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Blc50Block — 50Hz FPN subtraction via dark frame.
pub struct Blc50Block {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Per-channel offset (subtracted from each pixel). Default: [0, 0, 0].
    pub offsets: [f32; 3],
}

impl Default for Blc50Block {
    fn default() -> Self {
        Self::new()
    }
}

impl Blc50Block {
    pub fn new() -> Self {
        Self {
            id: "blc50".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "Blc50Block/frame".into(),
            input_source: String::new(),
            offsets: [0.0; 3],
        }
    }

    pub fn with_offsets(mut self, r: f32, g: f32, b: f32) -> Self {
        self.offsets = [r, g, b];
        self
    }
}

impl IspBlock for Blc50Block {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "Blc50".into()
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
        self.input_value_info()
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let dark = format!("{}/dark_frame", ns);
        let zero = format!("{}/zero", ns);
        let max_val = format!("{}/max_val", ns);
        let subtracted = format!("{}/subtracted", ns);

        vec![
            // Sub(input, dark_frame)
            Proto::node("Sub", &[&self.input_source, &dark], &[&subtracted], &[]),
            // Clip to [0, max]
            Proto::node("Max", &[&subtracted, &zero], &[&subtracted], &[]),
            Proto::node("Min", &[&subtracted, &max_val], &[&self.frame_tensor], &[]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float(&format!("{}/dark_frame", ns), &[1, 3, 1, 1], &self.offsets),
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/max_val", ns), 65535.0),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(
            format!("{}/{}", self.tensor_ns(), "dark_frame"),
            1,
            vec![1, 3, 1, 1],
        )]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_blc50_id() {
        assert_eq!(Blc50Block::new().id(), "blc50");
    }

    #[test]
    fn test_blc50_with_offsets() {
        let b = Blc50Block::new().with_offsets(10.0, 12.0, 8.0);
        assert_eq!(b.offsets, [10.0, 12.0, 8.0]);
    }

    #[test]
    fn test_blc50_emit_onnx() {
        let b = Blc50Block::new();
        let nodes = b.nodes();
        // Sub + Max + Min = 3 nodes
        assert_eq!(nodes.len(), 3);
    }

    #[test]
    fn test_blc50_has_input_output() {
        let b = Blc50Block::new();
        assert_eq!(b.input_tensors().len(), 1);
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_blc50_tensor_ns() {
        let b = Blc50Block::new();
        assert_eq!(b.tensor_ns(), "Blc50");
    }

    #[test]
    fn test_blc50_extra_inputs() {
        let b = Blc50Block::new();
        let extras = b.extra_inputs();
        assert_eq!(extras.len(), 1, "should have dark_frame extra input");
    }
}
