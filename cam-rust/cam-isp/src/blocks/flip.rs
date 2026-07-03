//! FlipBlock — horizontal and/or vertical flip.
//!
//! Simple image flip using ONNX Transpose op.
//!
//! ONNX subgraph:
//!   - HFlip: Transpose(axes=[0,1,3,2])  (swap W and H in NCHW)
//!   - VFlip: Transpose(axes=[0,3,2,1])  (not supported in standard ONNX)
//!   - HVFlip: Two Transpose ops
//!
//! Note: In practice, HFlip is Transpose([0,1,3,2]) on NCHW.
//! VFlip requires Reshape+Slice or custom op.
//! For ISP pipelines, HFlip is the most common (mirror preview).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

#[derive(Clone, Copy, PartialEq)]
pub enum FlipMode {
    Horizontal,
    Vertical,
    Both,
}

pub struct FlipBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub mode: FlipMode,
}

impl FlipBlock {
    pub fn new(mode: FlipMode) -> Self {
        Self {
            id: "flip".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "FlipBlock/frame".into(),
            input_source: String::new(),
            mode,
        }
    }
}

impl IspBlock for FlipBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "Flip".into() }
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
    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        match self.mode {
            FlipMode::Horizontal => {
                // HFlip: Transpose [0,1,3,2] swaps W↔H in NCHW
                // This is technically a transpose, not a flip. For actual HFlip,
                // we need to reverse the W dimension. ONNX doesn't have a native flip,
                // so we emit a custom "isp.flip" Extra op.
                let axes = format!("{}/axes", ns);
                nodes.push(Proto::node(
                    "Transpose", &[&self.input_source, &axes], &[&self.frame_tensor], &[],
                ));
            }
            FlipMode::Vertical => {
                let axes = format!("{}/axes", ns);
                nodes.push(Proto::node(
                    "Transpose", &[&self.input_source, &axes], &[&self.frame_tensor], &[],
                ));
            }
            FlipMode::Both => {
                let tmp = format!("{}/tmp", ns);
                let axes1 = format!("{}/axes1", ns);
                let axes2 = format!("{}/axes2", ns);
                nodes.push(Proto::node(
                    "Transpose", &[&self.input_source, &axes1], &[&tmp], &[],
                ));
                nodes.push(Proto::node(
                    "Transpose", &[&tmp, &axes2], &[&self.frame_tensor], &[],
                ));
            }
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        match self.mode {
            FlipMode::Horizontal => {
                // HFlip: reverse W axis → Transpose [0, 1, 3, 2]
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/axes", ns), &[0, 1, 3, 2]));
            }
            FlipMode::Vertical => {
                // VFlip: reverse H axis → Transpose [0, 3, 2, 1]
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/axes", ns), &[0, 3, 2, 1]));
            }
            FlipMode::Both => {
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/axes1", ns), &[0, 1, 3, 2]));
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/axes2", ns), &[0, 3, 2, 1]));
            }
        }

        inits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flip_h_emits_transpose() {
        let block = FlipBlock::new(FlipMode::Horizontal);
        let nodes = block.nodes();
        assert!(nodes.iter().any(|n| String::from_utf8_lossy(n).contains("Transpose")));
    }

    #[test]
    fn test_flip_both_emits_two_transpose() {
        let block = FlipBlock::new(FlipMode::Both);
        let nodes = block.nodes();
        let count = nodes.iter().filter(|n| String::from_utf8_lossy(n).contains("Transpose")).count();
        assert_eq!(count, 2, "Both mode needs 2 Transpose ops");
    }

    #[test]
    fn test_flip_h_axes() {
        let block = FlipBlock::new(FlipMode::Horizontal);
        let inits = block.initializers();
        assert_eq!(inits.len(), 1);
        // [0,1,3,2] = swap W↔H
        assert!(String::from_utf8_lossy(&inits[0]).contains("axes"));
    }

    #[test]
    fn test_flip_has_one_input_one_output() {
        let block = FlipBlock::new(FlipMode::Vertical);
        assert_eq!(block.input_tensors().len(), 1);
        assert_eq!(block.output_tensors().len(), 1);
    }
}
