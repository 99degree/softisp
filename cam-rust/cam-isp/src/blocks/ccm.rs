use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct CcmBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl CcmBlock {
    pub fn new() -> Self { Self { id: "ccm".into(), prev: None, next: None, frame_tensor: "CcmBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for CcmBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "CcmBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1),Proto::tensor_dim_value(3),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
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
        vec![
            Proto::tensor_proto_float(&format!("{}/matrix", ns), &[3, 3, 1, 1], &[1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]),
            Proto::tensor_proto_float(&format!("{}/bias", ns), &[3], &[0.0; 3]),
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/matrix", self.tensor_ns()), 1, vec![3, 3, 1, 1])]
    }
}
