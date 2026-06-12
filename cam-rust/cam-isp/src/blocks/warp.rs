use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct WarpBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
    pub use_precomputed_grid: bool,
}
impl WarpBlock {
    pub fn new(precomputed: bool) -> Self { Self { id: "warp".into(), prev: None, next: None, frame_tensor: "WarpBlock/frame".into(), input_source: String::new(), use_precomputed_grid: precomputed } }
}
impl IspBlock for WarpBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "WarpBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone(), format!("{}/warp_grid", self.tensor_ns())] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1),Proto::tensor_dim_value(3),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let warp_grid = format!("{}/warp_grid", self.tensor_ns());
        if self.use_precomputed_grid {
            vec![
                Proto::node("GridSampler", &[&self.input_source, &warp_grid], &[&self.frame_tensor],
                    &[Proto::attribute_string("mode", "bilinear"),
                      Proto::attribute_string("padding_mode", "zeros"),
                      Proto::attribute_int("align_corners", 0)]),
            ]
        } else {
            let resize_grid = format!("{}/resized_grid", self.tensor_ns());
            let resize_shape = format!("{}/resize_shape", self.tensor_ns());
            vec![
                Proto::node("Resize", &[&warp_grid, "", "", &resize_shape], &[&resize_grid],
                    &[Proto::attribute_string("mode", "linear")]),
                Proto::node("GridSampler", &[&self.input_source, &resize_grid], &[&self.frame_tensor],
                    &[Proto::attribute_string("mode", "bilinear"),
                      Proto::attribute_string("padding_mode", "zeros"),
                      Proto::attribute_int("align_corners", 0)]),
            ]
        }
    }
    fn initializers(&self) -> Vec<Vec<u8>> { vec![] }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        if self.use_precomputed_grid {
            vec![(format!("{}/warp_grid", self.tensor_ns()), 1, vec![1, -2, -2, 2])]
        } else {
            vec![(format!("{}/warp_grid", self.tensor_ns()), 1, vec![1, 13, 17, 2])]
        }
    }
    fn graph_input_name(&self) -> Option<&str> { None }
}
