//! RawInputBlock — pipeline head, declares INT16 input tensor.
use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct RawInputBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl RawInputBlock {
    pub fn new() -> Self {
        Self {
            id: "raw_input".to_string(),
            prev: None,
            next: None,
            frame_tensor: "RawInputBlock/frame".to_string(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for RawInputBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "RawInputBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn graph_input_name(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_elem_type(&self) -> i32 { 5 } // INT16

    fn input_tensors(&self) -> Vec<String> { vec![] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
            Proto::tensor_dim_param("height"), Proto::tensor_dim_param("width"),
        ], 5))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }
}
