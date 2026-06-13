use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// Demosaic block — Conv 1×1 converts 4 Bayer channels → 3 RGB channels.
///
/// Takes [1,4,H,W] input (from CfaBlock's Conv stride=2 unpack) and outputs
/// [1,3,H,W] where H,W are the halved dimensions after CfaBlock.
///
/// Uses Conv(1×1) with weights [3,4,1,1] matching the Java implementation.
///
/// Bayer pattern → weight mapping (SpaceToDepth channel order: TL, TR, BL, BR):
///   RGGB (0): R=TL, G=avg(TR,BL), B=BR
///   GRBG (1): R=TR, G=avg(TL,BR), B=BL
///   GBRG (2): R=BL, G=avg(TL,BR), B=TR
///   BGGR (3): R=BR, G=avg(TR,BL), B=TL
///
/// @param bayer_pattern Android color filter arrangement:
///   0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR. Default 2 (GBRG — common on Qualcomm).
pub struct DemosaicBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub bayer_pattern: i32,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl DemosaicBlock {
    pub fn new(bayer_pattern: i32) -> Self {
        Self {
            id: "demosaic".into(),
            prev: None,
            next: None,
            frame_tensor: "DemosaicBlock/frame".into(),
            input_source: String::new(),
            bayer_pattern,
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

    /// Compute Conv 1×1 weights [3,4,1,1] based on bayer pattern.
    fn conv_weights(&self) -> Vec<f32> {
        match self.bayer_pattern {
            0 => vec![ // RGGB: [R, G, G, B] → [R, G_avg, B]
                1f32, 0f32, 0f32, 0f32,    // out0=R: take ch0 (TL)
                0f32, 0.5f32, 0.5f32, 0f32, // out1=G: avg ch1,ch2 (TR,BL)
                0f32, 0f32, 0f32, 1f32,     // out2=B: take ch3 (BR)
            ],
            1 => vec![ // GRBG: [G, R, B, G] → [R, G_avg, B]
                0f32, 1f32, 0f32, 0f32,     // out0=R: take ch1
                0.5f32, 0f32, 0f32, 0.5f32, // out1=G: avg ch0,ch3
                0f32, 0f32, 1f32, 0f32,     // out2=B: take ch2
            ],
            2 => vec![ // GBRG: [G, B, R, G] → [R, G_avg, B]
                0f32, 0f32, 1f32, 0f32,     // out0=R: take ch2
                0.5f32, 0f32, 0f32, 0.5f32, // out1=G: avg ch0,ch3
                0f32, 1f32, 0f32, 0f32,     // out2=B: take ch1
            ],
            _ => vec![ // BGGR: [B, G, G, R] → [R, G_avg, B]
                0f32, 0f32, 0f32, 1f32,     // out0=R: take ch3
                0f32, 0.5f32, 0.5f32, 0f32, // out1=G: avg ch1,ch2
                1f32, 0f32, 0f32, 0f32,     // out2=B: take ch0
            ],
        }
    }
}

impl IspBlock for DemosaicBlock {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "DemosaicBlock".to_string()
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
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
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
                    Proto::attribute_ints("kernel_shape", &[1, 1]),
                    Proto::attribute_ints("strides", &[1, 1]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                ],
            ),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float(&format!("{}/w", ns), &[3, 4, 1, 1], &self.conv_weights()),
            Proto::tensor_proto_float(&format!("{}/b", ns), &[3], &[0f32; 3]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }
}
