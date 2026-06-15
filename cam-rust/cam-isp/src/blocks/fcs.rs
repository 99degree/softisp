use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct FcsBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl FcsBlock {
    pub fn new() -> Self { Self { id: "fcs".into(), prev: None, next: None, frame_tensor: "FcsBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for FcsBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "FcsBlock".to_string() }
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
            Proto::node("Abs", &[&format!("{}/edge_3ch", ns)], &[&format!("{}/edge_abs", ns)], &[]),
            Proto::node("Mul", &[&format!("{}/edge_abs", ns), &format!("zzz_{}/fcs_gain_scaled", ns)], &[&format!("{}/fcs_raw", ns)], &[]),
            Proto::node("Clip", &[&format!("{}/fcs_raw", ns), &format!("{}/zero", ns), &format!("{}/one", ns)], &[&format!("{}/fcs_clamped", ns)], &[]),
            Proto::node("Sub", &[&format!("{}/one", ns), &format!("{}/fcs_clamped", ns)], &[&format!("{}/fcs_attn", ns)], &[]),
            Proto::node("Mul", &[&format!("{}/fcs_attn", ns), &format!("{}/uv_mask", ns)], &[&format!("{}/uv_gain", ns)], &[]),
            Proto::node("Sub", &[&self.input_source, &format!("{}/half", ns)], &[&format!("{}/centered", ns)], &[]),
            Proto::node("Mul", &[&format!("{}/centered", ns), &format!("{}/uv_mask", ns)], &[&format!("{}/uv_centered", ns)], &[]),
            Proto::node("Sub", &[&format!("{}/centered", ns), &format!("{}/uv_centered", ns)], &[&format!("{}/y_centered", ns)], &[]),
            Proto::node("Mul", &[&format!("{}/uv_centered", ns), &format!("{}/uv_gain", ns)], &[&format!("{}/uv_suppressed", ns)], &[]),
            Proto::node("Add", &[&format!("{}/y_centered", ns), &format!("{}/uv_suppressed", ns)], &[&format!("{}/recombined", ns)], &[]),
            Proto::node("Add", &[&format!("{}/recombined", ns), &format!("{}/half", ns)], &[&format!("{}/result", ns)], &[]),
            Proto::node("Clip", &[&format!("{}/result", ns), &format!("{}/zero", ns), &format!("{}/one", ns)], &[&self.frame_tensor], &[]),
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
            Proto::tensor_proto_float(&format!("{}/uv_mask", ns), &[3, 1, 1], &[0.0, 1.0, 1.0]),
            Proto::tensor_proto_float(&format!("{}/inv_64", ns), &[], &[1.0/64.0]),
            Proto::tensor_proto_float(&format!("{}/half", ns), &[], &[0.5]),
            Proto::tensor_proto_float(&format!("{}/zero", ns), &[], &[0.0]),
            Proto::tensor_proto_float(&format!("{}/one", ns), &[], &[1.0]),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        // zzz_ prefix ensures alphabetical sort is AFTER the main input
        // "RawInputBlock/frame" (R < z), avoiding MNN input selection bug.
        vec![(format!("zzz_{}/fcs_gain_scaled", self.tensor_ns()), 1, vec![1])]
    }
}
