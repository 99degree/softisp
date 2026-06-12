use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct DemosaicBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
    pub bayer_pattern: i32,
}
impl DemosaicBlock {
    pub fn new(bayer_pattern: i32) -> Self {
        Self { id: "demosaic".into(), prev: None, next: None, frame_tensor: "DemosaicBlock/frame".into(), input_source: String::new(), bayer_pattern }
    }
}
impl IspBlock for DemosaicBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "DemosaicBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1),Proto::tensor_dim_value(4),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
    fn output_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.frame_tensor, &[Proto::tensor_dim_value(1),Proto::tensor_dim_value(3),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node("Conv", &[&self.input_source, &format!("{}/weight", ns), &format!("{}/bias", ns)], &[&format!("{}/conv", ns)],
                &[Proto::attribute_ints("kernel_shape", &[3, 3]),
                  Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                  Proto::attribute_ints("strides", &[1, 1]),
                  Proto::attribute_int("group", 4)]),
            Proto::node("Reshape", &[&format!("{}/conv", ns), &format!("{}/shape", ns)], &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut w = Vec::with_capacity(4*4*3*3);
        for i in 0..36 { w.push(if i%9==4 {1.0} else {0.0}); }
        w.extend_from_slice(&[0.0f32; 4*4*3*3 - 36]);
        vec![
            Proto::tensor_proto_float(&format!("{}/weight", ns), &[4, 4, 3, 3], &w),
            Proto::tensor_proto_float(&format!("{}/bias", ns), &[4], &[0.0; 4]),
            Proto::tensor_proto_int64(&format!("{}/shape", ns), &[1, 3, -1, -1]),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> { vec![] }
}
