//! UnpackCfaBlock — fused packed INT32 → CFA FLOAT quadrants.
//!
//! Input:  INT32[1,1,H,W/2]    (packed: pixel_even | (pixel_odd << 16))
//! Output: FLOAT[1,4,H/2,W/2]  (4 Bayer channels: R, Gr, Gb, B)
//!
//! Fuses UnpackBlock + NormalizeBlock + CfaBlock into a single ONNX graph:
//!   - Extract even (low 16 bits) and odd (high 16 bits) in INT32
//!   - Cast to FLOAT, divide by sensor_max (normalize)
//!   - Stack into [1,2,H,W/2], Conv(stride=(2,1), kernel=(2,1)) → 4 channels
//!   - No full-resolution interleave! Saves the expensive Reshape+Concat+Reshape.

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct UnpackCfaBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,  // FULL width (original W), used for output dims
    pub sensor_max: f32,
}

impl Default for UnpackCfaBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl UnpackCfaBlock {
    pub fn new() -> Self {
        Self {
            id: "unpack_cfa".into(),
            prev: None,
            next: None,
            frame_tensor: "UnpackCfaBlock/frame".into(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
            sensor_max: 65535.0,
        }
    }

    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }

    pub fn with_sensor_max(mut self, sm: f32) -> Self {
        self.sensor_max = sm;
        self
    }
}

impl IspBlock for UnpackCfaBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "UnpackCfaBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn input_elem_type(&self) -> i32 { 6 } // INT32
    fn output_elem_type(&self) -> i32 { 1 } // FLOAT

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let pw = self.concrete_w.map(|w| w / 2);
        let dims = match (self.concrete_h, pw) {
            (Some(h), Some(pw)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h), Proto::tensor_dim_value(pw),
            ],
            (None, Some(pw)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"), Proto::tensor_dim_value(pw),
            ],
            _ => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W2"),
            ],
        };
        Some(Proto::value_info(&self.input_source, &dims, 6))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(h / 2), Proto::tensor_dim_value(w / 2),
            ],
            _ => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H2"), Proto::tensor_dim_param("W2"),
            ],
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();

        vec![
            // 1–2: Extract even (low 16 bits) via INT16 truncation
            Proto::node("Cast", &[&self.input_source], &[&format!("{}/even_i16", ns)],
                &[Proto::attribute_int("to", 5)]),
            Proto::node("Cast", &[&format!("{}/even_i16", ns)], &[&format!("{}/even", ns)],
                &[Proto::attribute_int("to", 6)]),

            // 3–4: Cast even to FLOAT, normalize
            Proto::node("Cast", &[&format!("{}/even", ns)], &[&format!("{}/even_float", ns)],
                &[Proto::attribute_int("to", 1)]),
            Proto::node("Div", &[&format!("{}/even_float", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/even_norm", ns)], &[]),

            // 5: Extract odd (high 16 bits) via integer Div by 65536
            Proto::node("Div", &[&self.input_source, &format!("{}/div_65536", ns)],
                &[&format!("{}/odd", ns)], &[]),

            // 6–7: Cast odd to FLOAT, normalize
            Proto::node("Cast", &[&format!("{}/odd", ns)], &[&format!("{}/odd_float", ns)],
                &[Proto::attribute_int("to", 1)]),
            Proto::node("Div", &[&format!("{}/odd_float", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/odd_norm", ns)], &[]),

            // 8: Stack even + odd into [1,2,H,W/2]
            Proto::node("Concat",
                &[&format!("{}/even_norm", ns), &format!("{}/odd_norm", ns)],
                &[&format!("{}/stacked", ns)],
                &[Proto::attribute_int("axis", 1)]),

            // 9: Conv stride=(2,1) kernel=(2,1) → [1,4,H/2,W/2]
            { // separate filters for RGGB pattern
                Proto::node("Conv",
                    &[&format!("{}/stacked", ns), &format!("{}/cfa_w", ns), &format!("{}/cfa_b", ns)],
                    &[&self.frame_tensor],
                    &[
                        Proto::attribute_ints("kernel_shape", &[2, 1]),
                        Proto::attribute_ints("strides", &[2, 1]),
                        Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                        Proto::attribute_int("group", 1),
                    ])
            },
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let sm = self.sensor_max;

        // Conv weights [4, 2, 2, 1] — 4 filters, 2 in channels, kernel 2×1
        // Filter 0 (R):  in_ch=0 (even), picks top row → [[1],[0]]
        // Filter 1 (Gr): in_ch=1 (odd),  picks top row → [[1],[0]]
        // Filter 2 (Gb): in_ch=0 (even), picks bottom row → [[0],[1]]
        // Filter 3 (B):  in_ch=1 (odd),  picks bottom row → [[0],[1]]
        let w = vec![
            // out_ch=0 (R)
            1f32, 0f32,  // in_ch=0 (even): picks top row
            0f32, 0f32,  // in_ch=1 (odd):  none
            // out_ch=1 (Gr)
            0f32, 0f32,  // in_ch=0 (even): none
            1f32, 0f32,  // in_ch=1 (odd):  picks top row
            // out_ch=2 (Gb)
            0f32, 1f32,  // in_ch=0 (even): picks bottom row
            0f32, 0f32,  // in_ch=1 (odd):  none
            // out_ch=3 (B)
            0f32, 0f32,  // in_ch=0 (even): none
            0f32, 1f32,  // in_ch=1 (odd):  picks bottom row
        ];

        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/max_val", ns), sm),
            Proto::tensor_proto_int32_scalar(&format!("{}/div_65536", ns), 65536),
            Proto::tensor_proto_float(&format!("{}/cfa_w", ns), &[4, 2, 2, 1], &w),
            Proto::tensor_proto_float(&format!("{}/cfa_b", ns), &[4], &[0f32; 4]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![
            (format!("{}/max_val", self.tensor_ns()), 1, vec![]),
            (format!("{}/div_65536", self.tensor_ns()), 6, vec![]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_unpack_cfa_generates_nodes() {
        let block = UnpackCfaBlock::new().with_concrete_dims(48, 64);
        let nodes = block.nodes();
        // 2 Cast + 1 Cast + 1 Div + 1 Div + 1 Cast + 1 Div + 1 Concat + 1 Conv = 9
        assert_eq!(nodes.len(), 9, "UnpackCfaBlock should produce 9 nodes");
        let inits = block.initializers();
        assert_eq!(inits.len(), 4, "UnpackCfaBlock should have 4 initializers");
    }

    #[test]
    fn test_unpack_cfa_value_info() {
        let block = UnpackCfaBlock::new().with_concrete_dims(48, 64);
        let in_vi = block.input_value_info().unwrap();
        let out_vi = block.output_value_info().unwrap();
        assert!(!in_vi.is_empty());
        assert!(!out_vi.is_empty());
    }

    #[test]
    fn test_unpack_cfa_pipeline_integration() {
        // Build: RawInput(packed INT32) -> UnpackCfaBlock -> BlcBlock
        let b1: Box<dyn IspBlock> = Box::new(crate::blocks::RawInputBlock::new()
            .with_elem_type(6)   // INT32 input
            .with_concrete_dims(48, 32));  // packed width = 32
        let b2: Box<dyn IspBlock> = Box::new(UnpackCfaBlock::new().with_concrete_dims(48, 64));
        let b3: Box<dyn IspBlock> = Box::new(crate::blocks::BlcBlock::new());

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(result.is_ok(), "UnpackCfaBlock pipeline should compose: {:?}", result.err());
        let model = result.unwrap();
        assert!(!model.is_empty(), "Model should not be empty");
    }
}
