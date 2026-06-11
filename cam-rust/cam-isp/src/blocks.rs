//! ISP processing blocks for the pipeline.
//! Each block emits ONNX graph fragments used by GraphComposer to build a fused model.
//! Ported from com.camcore.isp.pipeline.processing.*

use log::info;

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

// ---------------------------------------------------------------------------
// Helpers for shared patterns
// ---------------------------------------------------------------------------

/// Build output value info for NCHW float tensors.
fn nchw_value_info(name: &str, c: i64, elem_type: i32) -> Vec<u8> {
    Proto::value_info(name, &[
        Proto::tensor_dim_value(1),
        Proto::tensor_dim_value(c),
        Proto::tensor_dim_param("H"),
        Proto::tensor_dim_param("W"),
    ], elem_type)
}

// ---------------------------------------------------------------------------
// RawInputBlock — pipeline head, declares INT16 input tensor
// ---------------------------------------------------------------------------

pub struct RawInputBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl RawInputBlock {
    pub fn new() -> Self {
        Self {
            id: "raw_input".to_string(),
            prev: None,
            next: None,
            frame_tensor: "RawInputBlock/frame".to_string(),
            input_source: String::new(),
        }
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
    fn input_elem_type(&self) -> i32 { 5 } // INT16

    fn input_tensors(&self) -> Vec<String> { vec![] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
            Proto::tensor_dim_param("height"), Proto::tensor_dim_param("width"),
        ], 5)) // INT16
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        self.input_value_info()
    }
}

// ---------------------------------------------------------------------------
// NormalizeBlock — Cast INT16→FLOAT, Div by sensor_max
// ---------------------------------------------------------------------------

pub struct NormalizeBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl NormalizeBlock {
    pub fn new() -> Self {
        Self {
            id: "normalize".to_string(),
            prev: None,
            next: None,
            frame_tensor: "NormalizeBlock/frame".to_string(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for NormalizeBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "NormalizeBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let cast_out = format!("{}/cast", self.tensor_ns());
        let sensor_max = format!("{}/sensor_max", self.tensor_ns());
        vec![
            Proto::node("Cast", &[&self.input_source], &[&cast_out],
                &[Proto::attribute_int("to", 1)]), // INT16→FLOAT
            Proto::node("Div", &[&cast_out, &sensor_max], &[&self.frame_tensor], &[]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/sensor_max", self.tensor_ns()), 1, vec![1])]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(nchw_value_info(&self.frame_tensor, 1, 1))
    }
}

// ---------------------------------------------------------------------------
// CfaBlock — extract 2×2 Bayer quad positions via Conv stride=2
// ---------------------------------------------------------------------------

pub struct CfaBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl CfaBlock {
    pub fn new() -> Self {
        Self {
            id: "cfa".to_string(),
            prev: None,
            next: None,
            frame_tensor: "CfaBlock/frame".to_string(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for CfaBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "CfaBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let w_name = format!("{}/w", self.tensor_ns());
        let b_name = format!("{}/b", self.tensor_ns());
        vec![
            Proto::node("Conv", &[&self.input_source, &w_name, &b_name], &[&self.frame_tensor],
                &[Proto::attribute_ints("kernel_shape", &[2, 2]),
                  Proto::attribute_ints("strides", &[2, 2]),
                  Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                  Proto::attribute_int("group", 1)]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let w_name = format!("{}/w", self.tensor_ns());
        let b_name = format!("{}/b", self.tensor_ns());
        vec![
            // Conv weights [4,1,2,2]
            Proto::tensor_proto_float(&w_name, &[4, 1, 2, 2], &[
                1.0,0.0,0.0,0.0, // TL
                0.0,1.0,0.0,0.0, // TR
                0.0,0.0,1.0,0.0, // BL
                0.0,0.0,0.0,1.0, // BR
            ]),
            // Bias [4]
            Proto::tensor_proto_float(&b_name, &[4], &[0.0, 0.0, 0.0, 0.0]),
        ]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(4),
            Proto::tensor_dim_param("H2"), Proto::tensor_dim_param("W2"),
        ], 1))
    }
}

// ---------------------------------------------------------------------------
// BlcBlock — Black Level Correction: Sub(blc_values)
// ---------------------------------------------------------------------------

pub struct BlcBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl BlcBlock {
    pub fn new() -> Self {
        Self {
            id: "blc".to_string(),
            prev: None,
            next: None,
            frame_tensor: "BlcBlock/frame".to_string(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for BlcBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "BlcBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let blc_name = format!("{}/blc", self.tensor_ns());
        vec![
            Proto::node("Sub", &[&self.input_source, &blc_name], &[&self.frame_tensor], &[]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/blc", self.tensor_ns()), 1, vec![4, 1, 1])]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(4),
            Proto::tensor_dim_param("H2"), Proto::tensor_dim_param("W2"),
        ], 1))
    }
}

// ---------------------------------------------------------------------------
// BayerWbBlock — White Balance: Mul(gains)
// ---------------------------------------------------------------------------

pub struct BayerWbBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl BayerWbBlock {
    pub fn new() -> Self {
        Self {
            id: "bayer_wb".to_string(),
            prev: None,
            next: None,
            frame_tensor: "BayerWbBlock/frame".to_string(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for BayerWbBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "BayerWbBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let gains_name = format!("{}/gains", self.tensor_ns());
        vec![
            Proto::node("Mul", &[&self.input_source, &gains_name], &[&self.frame_tensor], &[]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/gains", self.tensor_ns()), 1, vec![4, 1, 1])]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(4),
            Proto::tensor_dim_param("H2"), Proto::tensor_dim_param("W2"),
        ], 1))
    }
}

// ---------------------------------------------------------------------------
// DemosaicBlock — SpaceToDepth(2) + Conv(1×1) Bayer→RGB
// ---------------------------------------------------------------------------

pub struct DemosaicBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub bayer_pattern: i32, // 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR
}

impl DemosaicBlock {
    pub fn new(bayer_pattern: i32) -> Self {
        Self {
            id: "demosaic".to_string(),
            prev: None,
            next: None,
            frame_tensor: "DemosaicBlock/frame".to_string(),
            input_source: String::new(),
            bayer_pattern,
        }
    }

    fn weights_for_pattern(&self) -> Vec<f32> {
        match self.bayer_pattern {
            0 => vec![ // RGGB: [R, G, G, B] → [R, G_avg, B]
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.5, 0.5, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
            1 => vec![ // GRBG
                0.0, 1.0, 0.0, 0.0,
                0.5, 0.0, 0.0, 0.5,
                0.0, 0.0, 1.0, 0.0,
            ],
            2 => vec![ // GBRG
                0.0, 0.0, 1.0, 0.0,
                0.5, 0.0, 0.0, 0.5,
                0.0, 1.0, 0.0, 0.0,
            ],
            _ => vec![ // BGGR or default
                0.0, 0.0, 0.0, 1.0,
                0.0, 0.5, 0.5, 0.0,
                1.0, 0.0, 0.0, 0.0,
            ],
        }
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

    fn nodes(&self) -> Vec<Vec<u8>> {
        let conv_in = format!("{}/conv_in", self.tensor_ns());
        let w_name = format!("{}/conv_w", self.tensor_ns());
        let b_name = format!("{}/conv_b", self.tensor_ns());
        vec![
            Proto::node("SpaceToDepth", &[&self.input_source], &[&conv_in],
                &[Proto::attribute_int("blocksize", 2)]),
            Proto::node("Conv", &[&conv_in, &w_name, &b_name], &[&self.frame_tensor],
                &[Proto::attribute_ints("kernel_shape", &[1, 1]),
                  Proto::attribute_ints("strides", &[1, 1]),
                  Proto::attribute_ints("pads", &[0, 0, 0, 0])]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let w_name = format!("{}/conv_w", self.tensor_ns());
        let b_name = format!("{}/conv_b", self.tensor_ns());
        vec![
            Proto::tensor_proto_float(&w_name, &[3, 4, 1, 1], &self.weights_for_pattern()),
            Proto::tensor_proto_float(&b_name, &[3], &[0.0, 0.0, 0.0]),
        ]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
            Proto::tensor_dim_param("H2"), Proto::tensor_dim_param("W2"),
        ], 1))
    }
}

// ---------------------------------------------------------------------------
// CcmBlock — Color Correction Matrix (Gemm/MatMul)
// ---------------------------------------------------------------------------

pub struct CcmBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl CcmBlock {
    pub fn new() -> Self {
        Self {
            id: "ccm".to_string(),
            prev: None,
            next: None,
            frame_tensor: "CcmBlock/frame".to_string(),
            input_source: String::new(),
        }
    }
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

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ccm_name = format!("{}/matrix", self.tensor_ns());
        // Use Conv with 1×1 kernel for per-pixel color matrix application
        // This is equivalent to MatMul with [3,3] on the channel dimension
        vec![
            Proto::node("Gemm", &[&self.input_source, &ccm_name, &format!("{}/bias", self.tensor_ns())],
                &[&self.frame_tensor],
                &[Proto::attribute_float("alpha", 1.0),
                  Proto::attribute_float("beta", 1.0),
                  Proto::attribute_int("transB", 1)]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ccm_name = format!("{}/matrix", self.tensor_ns());
        let bias_name = format!("{}/bias", self.tensor_ns());
        vec![
            // Default identity matrix + zero bias
            Proto::tensor_proto_float(&ccm_name, &[3, 3],
                &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
            Proto::tensor_proto_float(&bias_name, &[3], &[0.0, 0.0, 0.0]),
        ]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
            Proto::tensor_dim_param("H2"), Proto::tensor_dim_param("W2"),
        ], 1))
    }
}

// ---------------------------------------------------------------------------
// ToneBlock — gamma/contrast/brightness mapping
// ---------------------------------------------------------------------------

pub struct ToneBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl ToneBlock {
    pub fn new() -> Self {
        Self {
            id: "tone".to_string(),
            prev: None,
            next: None,
            frame_tensor: "ToneBlock/frame".to_string(),
            input_source: String::new(),
        }
    }
}

impl IspBlock for ToneBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "ToneBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn nodes(&self) -> Vec<Vec<u8>> {
        // Tone mapping: contrast → brightness → gamma
        // y = ((x - 0.5) * contrast + 0.5 + brightness).clamp(0,1) ^ (1/gamma)
        let sub_out = format!("{}/sub_mean", self.tensor_ns());
        let mul_contrast = format!("{}/mul_contrast", self.tensor_ns());
        let add_bright = format!("{}/add_bright", self.tensor_ns());
        let clamp_out = format!("{}/clamp", self.tensor_ns());
        let gamma_recip = format!("{}/gamma_recip", self.tensor_ns());
        let pow_out = format!("{}/pow", self.tensor_ns());
        let half: &str = "ToneBlock/half";
        vec![
            // x - 0.5
            Proto::node("Sub", &[&self.input_source, half], &[&sub_out], &[]),
            // (x-0.5) * contrast
            Proto::node("Mul", &[&sub_out, &format!("{}/contrast", self.tensor_ns())], &[&mul_contrast], &[]),
            // + 0.5 + brightness
            Proto::node("Add", &[&mul_contrast, half], &[&add_bright], &[]),
            Proto::node("Add", &[&add_bright, &format!("{}/brightness", self.tensor_ns())], &[&clamp_out], &[]),
            // Clip to [0, 1]
            Proto::node("Clip", &[&clamp_out], &[&pow_out],
                &[Proto::attribute_float("min", 0.0),
                  Proto::attribute_float("max", 1.0)]),
            // Pow(1/gamma)
            Proto::node("Pow", &[&pow_out, &gamma_recip], &[&self.frame_tensor], &[]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![
            Proto::tensor_proto_float_scalar("ToneBlock/half", 0.5),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![
            (format!("{}/contrast", self.tensor_ns()), 1, vec![1]),
            (format!("{}/brightness", self.tensor_ns()), 1, vec![1]),
            (format!("{}/gamma_recip", self.tensor_ns()), 1, vec![1]),
        ]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
            Proto::tensor_dim_param("H2"), Proto::tensor_dim_param("W2"),
        ], 1))
    }
}

// ---------------------------------------------------------------------------
// DisplayBlock — Resize, Transpose to NHWC, Cast to uint8
// ---------------------------------------------------------------------------

pub struct DisplayBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub target_width: u32,
}

impl DisplayBlock {
    pub fn new(target_width: u32) -> Self {
        Self {
            id: "display_output".to_string(),
            prev: None,
            next: None,
            frame_tensor: "DisplayBlock/frame".to_string(),
            input_source: String::new(),
            target_width,
        }
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

    fn output_elem_type(&self) -> i32 { 2 } // UINT8
    fn graph_output_name(&self) -> Option<&str> { Some(&self.frame_tensor) }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let resize_out = format!("{}/resize", self.tensor_ns());
        let transpose_out = format!("{}/transpose", self.tensor_ns());
        let pad_out = format!("{}/pad", self.tensor_ns());
        let gather_out = format!("{}/gather", self.tensor_ns());
        let mul_out = format!("{}/mul", self.tensor_ns());
        let sizes = format!("{}/sizes", self.tensor_ns());
        let pads = format!("{}/pads", self.tensor_ns());
        let const_val = format!("{}/const_val", self.tensor_ns());
        let scale = format!("{}/scale", self.tensor_ns());
        let reorder = format!("{}/reorder", self.tensor_ns());
        vec![
            // Resize: NCHW → NCHW (target size)
            Proto::node("Resize", &[&self.input_source, "", "", &sizes], &[&resize_out],
                &[Proto::attribute_string("mode", "linear"),
                  Proto::attribute_string("coordinate_transformation_mode", "asymmetric")]),
            // Transpose NCHW → NHWC
            Proto::node("Transpose", &[&resize_out], &[&transpose_out],
                &[Proto::attribute_ints("perm", &[0, 2, 3, 1])]),
            // Pad alpha channel: [N,H,W,3] → [N,H,W,4]
            Proto::node("Pad", &[&transpose_out, &pads, &const_val], &[&pad_out],
                &[Proto::attribute_string("mode", "constant")]),
            // Reorder last dim: (R,G,B,A) → (B,G,R,A) for BGRA output
            Proto::node("Gather", &[&pad_out, &reorder], &[&gather_out],
                &[Proto::attribute_int("axis", 3)]),
            // Mul(255) → uint8 range
            Proto::node("Mul", &[&gather_out, &scale], &[&mul_out], &[]),
            // Cast to UINT8
            Proto::node("Cast", &[&mul_out], &[&self.frame_tensor],
                &[Proto::attribute_int("to", 2)]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/scale", self.tensor_ns()), 255.0),
            Proto::tensor_proto_float_scalar(&format!("{}/const_val", self.tensor_ns()), 1.0),
            Proto::tensor_proto_int64(&format!("{}/pads", self.tensor_ns()), &[0, 0, 0, 0, 0, 0, 0, 1]),
            Proto::tensor_proto_int64(&format!("{}/reorder", self.tensor_ns()), &[2, 1, 0, 3]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/sizes", self.tensor_ns()), 7, vec![4])] // int64[4]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &[
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_param("height"),
            Proto::tensor_dim_param("width"),
            Proto::tensor_dim_value(4),
        ], 2)) // UINT8
    }
}

// ---------------------------------------------------------------------------
// Register all built-in blocks
// ---------------------------------------------------------------------------

pub fn register_builtin_blocks() {
    info!("Registered built-in ISP processing blocks");
}
