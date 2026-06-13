use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// CFA unpack block — extracts 2x2 Bayer quad positions using Conv stride=2.
///
/// Takes [1,1,H,W] float normalized Bayer data and outputs [1,4,H/2,W/2]
/// where each channel corresponds to one quad position (TL, TR, BL, BR).
///
/// Uses Conv(kernel=2, stride=2, 4 filters) matching the Java implementation.
/// Avoids Slice op which MNN doesn't handle.
pub struct CfaBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl CfaBlock {
    pub fn new() -> Self {
        Self {
            id: "cfa".into(),
            prev: None,
            next: None,
            frame_tensor: "CfaBlock/frame".into(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
        }
    }

    /// Set concrete height/width for fixed-shape models.
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
}

impl IspBlock for CfaBlock {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "CfaBlock".to_string()
    }

    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }

    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
    }

    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev.as_ref()
    }

    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev = Some(block);
    }

    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next.as_ref()
    }

    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next = Some(block);
    }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }

    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let oh = self.concrete_h.map(|h| h / 2);
        let ow = self.concrete_w.map(|w| w / 2);
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (oh, ow) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H2"),
                Proto::tensor_dim_param("W2"),
            ]
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node(
                "Conv",
                &[&self.input_source, &format!("{}/w", ns), &format!("{}/b", ns)],
                &[&self.frame_tensor],
                &[
                    Proto::attribute_ints("kernel_shape", &[2, 2]),
                    Proto::attribute_ints("strides", &[2, 2]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                    Proto::attribute_int("group", 1),
                ],
            ),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Conv weights [4, 1, 2, 2] — 4 filters, 1 input channel, 2x2 kernel
        // Each filter picks one quad position:
        //   filter 0 (TL): [[1,0],[0,0]]
        //   filter 1 (TR): [[0,1],[0,0]]
        //   filter 2 (BL): [[0,0],[1,0]]
        //   filter 3 (BR): [[0,0],[0,1]]
        let w = vec![
            1f32, 0f32, 0f32, 0f32, // TL
            0f32, 1f32, 0f32, 0f32, // TR
            0f32, 0f32, 1f32, 0f32, // BL
            0f32, 0f32, 0f32, 1f32, // BR
        ];
        vec![
            Proto::tensor_proto_float(&format!("{}/w", ns), &[4, 1, 2, 2], &w),
            Proto::tensor_proto_float(&format!("{}/b", ns), &[4], &[0f32; 4]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }
}
