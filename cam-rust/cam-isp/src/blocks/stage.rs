//! StageBlock — pipeline stage marker (metadata only).
//!
//! Emits a single Identity node with a descriptive name.
//! Used to mark logical pipeline stages for profiling and debugging.
//! Unlike PassthroughBlock, StageBlock carries a stage index for ordering.

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// StageBlock — marks a named pipeline stage.
pub struct StageBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub stage_index: u32,
}

impl StageBlock {
    pub fn new(name: &str, index: u32) -> Self {
        Self {
            id: format!("stage_{}", name),
            prev_block: None,
            next_block: None,
            frame_tensor: format!("StageBlock/{}/frame", name),
            input_source: String::new(),
            stage_index: index,
        }
    }
}

impl IspBlock for StageBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { format!("Stage/{}", self.id) }
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
        vec![Proto::node("Identity", &[&self.input_source], &[&self.frame_tensor], &[])]
    }

    fn initializers(&self) -> Vec<Vec<u8>> { vec![] }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stage_id() {
        let b = StageBlock::new("preprocess", 0);
        assert_eq!(b.id(), "stage_preprocess");
        assert_eq!(b.stage_index, 0);
    }

    #[test]
    fn test_stage_emit_onnx() {
        let b = StageBlock::new("test", 1);
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 1);
    }

    #[test]
    fn test_stage_has_input_output() {
        let b = StageBlock::new("test", 0);
        assert_eq!(b.input_tensors().len(), 1);
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_stage_tensor_ns() {
        let b = StageBlock::new("color_correction", 2);
        assert!(b.tensor_ns().contains("Stage"));
    }
}
