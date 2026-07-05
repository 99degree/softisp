//! PassthroughBlock — identity block with naming for debugging.
//!
//! Emits a single Identity node. Useful for:
//!   - Named pipeline breakpoints for debugging
//!   - Placeholder blocks for future replacement
//!   - Testing pipeline composition without processing

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// PassthroughBlock — identity pass-through with a descriptive name.
pub struct PassthroughBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl PassthroughBlock {
    pub fn new(name: &str) -> Self {
        Self {
            id: format!("passthrough_{}", name),
            prev_block: None,
            next_block: None,
            frame_tensor: format!("PassthroughBlock/{}/frame", name),
            input_source: String::new(),
        }
    }
}

impl IspBlock for PassthroughBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { format!("Passthrough/{}", self.id) }
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
    fn test_passthrough_id() {
        let b = PassthroughBlock::new("debug_point");
        assert_eq!(b.id(), "passthrough_debug_point");
    }

    #[test]
    fn test_passthrough_emit_onnx() {
        let b = PassthroughBlock::new("test");
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 1);
        let s = String::from_utf8_lossy(&nodes[0]);
        assert!(s.contains("Identity"));
    }

    #[test]
    fn test_passthrough_has_input_output() {
        let b = PassthroughBlock::new("test");
        assert_eq!(b.input_tensors().len(), 1);
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_passthrough_tensor_ns() {
        let b = PassthroughBlock::new("my_breakpoint");
        assert_eq!(b.tensor_ns(), "Passthrough/passthrough_my_breakpoint");
    }
}
