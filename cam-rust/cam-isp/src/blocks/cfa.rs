use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct CfaBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl CfaBlock {
    pub fn new() -> Self { Self { id: "cfa".into(), prev: None, next: None, frame_tensor: "CfaBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for CfaBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "CfaBlock".to_string() }
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
        Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(1), Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(4), Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let slice = format!("{}/slice", self.tensor_ns());
        vec![
            Proto::node("Slice", &[&self.input_source, &format!("{}/starts", self.tensor_ns()), &format!("{}/ends", self.tensor_ns()), &format!("{}/axes", self.tensor_ns()), &format!("{}/steps", self.tensor_ns())], &[&slice], &[]),
            Proto::node("Reshape", &[&slice, &format!("{}/shape", self.tensor_ns())], &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_int64(&format!("{}/starts", ns), &[0, 0, 0, 0]),
            Proto::tensor_proto_int64(&format!("{}/ends", ns), &[1, 4, 0, 0]),
            Proto::tensor_proto_int64(&format!("{}/axes", ns), &[1]),
            Proto::tensor_proto_int64(&format!("{}/steps", ns), &[1]),
            Proto::tensor_proto_int64(&format!("{}/shape", ns), &[1, 4, -1, -1]),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> { vec![] }
}
