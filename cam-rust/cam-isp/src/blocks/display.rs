use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct DisplayBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
    pub target_width: u32,
}
impl DisplayBlock {
    pub fn new(target_width: u32) -> Self {
        Self { id: "display".into(), prev: None, next: None, frame_tensor: "DisplayBlock/frame".into(), input_source: String::new(), target_width }
    }
}
impl IspBlock for DisplayBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "DisplayBlock".to_string() }
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
    fn output_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.frame_tensor, &[Proto::tensor_dim_value(1),Proto::tensor_dim_value(4),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 2)) }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let sizes = format!("{}/sizes", ns);
        let scale = format!("{}/scale", ns);
        let mul_out = format!("{}/mul", ns);
        vec![
            Proto::node("Resize", &[&self.input_source, "", "", &sizes], &[&format!("{}/resized", ns)],
                &[Proto::attribute_string("mode", "linear"),
                  Proto::attribute_string("coordinate_transformation_mode", "asymmetric")]),
            Proto::node("Mul", &[&format!("{}/resized", ns), &scale], &[&mul_out], &[]),
            Proto::node("Cast", &[&mul_out], &[&self.frame_tensor], &[Proto::attribute_int("to", 2)]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/scale", ns), 255.0),
            Proto::tensor_proto_int64(&format!("{}/sizes", ns), &[1, 3, self.target_width as i64, -1]),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> { vec![] }
}

