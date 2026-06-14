use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct LdciBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl LdciBlock {
    pub fn new() -> Self { Self { id: "ldci".into(), prev: None, next: None, frame_tensor: "LdciBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for LdciBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "LdciBlock".to_string() }
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
            Proto::node("AveragePool", &[&self.input_source], &[&format!("{}/local_mean", ns)],
                &[Proto::attribute_ints("kernel_shape", &[9, 9]),
                  Proto::attribute_ints("pads", &[4, 4, 4, 4]),
                  Proto::attribute_ints("strides", &[1, 1])]),
            Proto::node("Sub", &[&self.input_source, &format!("{}/local_mean", ns)], &[&format!("{}/diff", ns)], &[]),
            Proto::node("Mul", &[&format!("{}/diff", ns), &format!("{}/y_mask", ns)], &[&format!("{}/diff_y", ns)], &[]),
            Proto::node("Mul", &[&format!("{}/diff_y", ns), &format!("{}/ldci_strength_scaled", ns)], &[&format!("{}/boost", ns)], &[]),
            Proto::node("Add", &[&self.input_source, &format!("{}/boost", ns)], &[&format!("{}/enhanced", ns)], &[]),
            Proto::node("Clip", &[&format!("{}/enhanced", ns), &format!("{}/min", ns), &format!("{}/max", ns)], &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float(&format!("{}/y_mask", ns), &[3, 1, 1], &[1.0, 0.0, 0.0]),
            Proto::tensor_proto_float(&format!("{}/min", ns), &[], &[0.0]),
            Proto::tensor_proto_float(&format!("{}/max", ns), &[], &[1.0]),
            Proto::tensor_proto_float(&format!("{}/inv_16", ns), &[], &[1.0/16.0]),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/ldci_strength_scaled", self.tensor_ns()), 1, vec![1])]
    }
}
