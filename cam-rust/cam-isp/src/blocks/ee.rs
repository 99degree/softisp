use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct EeBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl EeBlock {
    pub fn new() -> Self { Self { id: "ee".into(), prev: None, next: None, frame_tensor: "EeBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for EeBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "EeBlock".to_string() }
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
            Proto::node("Conv", &[&self.input_source, &format!("{}/kernel_3ch", ns), &format!("{}/bias_3ch", ns)], &[&format!("{}/edge_3ch", ns)],
                &[Proto::attribute_ints("kernel_shape", &[3, 5]),
                  Proto::attribute_ints("pads", &[1, 2, 1, 2]),
                  Proto::attribute_ints("strides", &[1, 1]),
                  Proto::attribute_int("group", 3)]),
            Proto::node("Mul", &[&format!("{}/edge_3ch", ns), &format!("{}/y_mask", ns)], &[&format!("{}/edge_y", ns)], &[]),
            Proto::node("Mul", &[&format!("{}/edge_y", ns), &format!("{}/ee_gain_scaled", ns)], &[&format!("{}/boost_raw", ns)], &[]),
            Proto::node("Clip", &[&format!("{}/boost_raw", ns), &format!("{}/boost_min", ns), &format!("{}/boost_max", ns)], &[&format!("{}/boost", ns)], &[]),
            Proto::node("Add", &[&self.input_source, &format!("{}/boost", ns)], &[&format!("{}/enhanced", ns)], &[]),
            Proto::node("Clip", &[&format!("{}/enhanced", ns), &format!("{}/zero", ns), &format!("{}/one", ns)], &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let k: [f32; 15] = [-0.125,0.0,-0.125,0.0,-0.125,-0.125,0.0,1.0,0.0,-0.125,-0.125,0.0,-0.125,0.0,-0.125];
        let mut k3 = Vec::with_capacity(45);
        for _ in 0..3 { k3.extend_from_slice(&k); }
        vec![
            Proto::tensor_proto_float(&format!("{}/kernel_3ch", ns), &[3, 1, 3, 5], &k3),
            Proto::tensor_proto_float(&format!("{}/bias_3ch", ns), &[3], &[0.0; 3]),
            Proto::tensor_proto_float(&format!("{}/y_mask", ns), &[3, 1, 1], &[1.0, 0.0, 0.0]),
            Proto::tensor_proto_float(&format!("{}/inv_256", ns), &[], &[1.0/256.0]),
            Proto::tensor_proto_float(&format!("{}/boost_min", ns), &[], &[-0.25]),
            Proto::tensor_proto_float(&format!("{}/boost_max", ns), &[], &[0.25]),
            Proto::tensor_proto_float(&format!("{}/zero", ns), &[], &[0.0]),
            Proto::tensor_proto_float(&format!("{}/one", ns), &[], &[1.0]),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/ee_gain_scaled", self.tensor_ns()), 1, vec![1])]
    }
}
