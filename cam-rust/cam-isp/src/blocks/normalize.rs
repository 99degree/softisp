//! NormalizeBlock — Cast INT32→FLOAT, Div by sensor_max.
use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct NormalizeBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl NormalizeBlock {
    pub fn new() -> Self {
        Self {
            id: "normalize".to_string(),
            prev: None,
            next: None,
            frame_tensor: "NormalizeBlock/frame".to_string(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for NormalizeBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "NormalizeBlock".to_string() }
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
        Some(Proto::value_info(&self.input_source, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
            Proto::tensor_dim_param("height"), Proto::tensor_dim_param("width"),
        ], 5))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
            Proto::tensor_dim_param("height"), Proto::tensor_dim_param("width"),
        ], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        vec![
            Proto::node("Cast", &[&self.input_source], &[&format!("{}/float", self.tensor_ns())],
                &[Proto::attribute_int("to", 1)]),
            Proto::node("Div", &[&format!("{}/float", self.tensor_ns()), &format!("{}/max_val", self.tensor_ns())],
                &[&self.frame_tensor], &[]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/max_val", self.tensor_ns()), 65535.0),
        ]
    }
}
