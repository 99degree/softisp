//! RawInputBlock — pipeline head, declares INT32 input tensor (no UINT16 in pipeline).
use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct RawInputBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
    pub elem_type: i32,
}

impl RawInputBlock {
    pub fn new() -> Self {
        Self {
            id: "raw_input".to_string(),
            prev: None,
            next: None,
            frame_tensor: "RawInputBlock/frame".to_string(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
            elem_type: 6, // INT32 default — no UINT16 in pipeline
        }
    }
    
    /// Set concrete height/width for fixed-shape models (avoids MNN resize crash).
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
    /// Set only the concrete width (height stays symbolic).
    pub fn with_concrete_width(mut self, w: i64) -> Self {
        self.concrete_w = Some(w);
        self
    }
    
    /// Set element type: 1=FLOAT, 6=INT32 (default).
    pub fn with_elem_type(mut self, t: i32) -> Self {
        self.elem_type = t;
        self
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
    fn input_elem_type(&self) -> i32 { self.elem_type }

    fn input_tensors(&self) -> Vec<String> { vec![] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h), Proto::tensor_dim_value(w),
            ],
            (None, Some(w)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"), Proto::tensor_dim_value(w),
            ],
            _ => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W"),
            ],
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, self.elem_type))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }
}
