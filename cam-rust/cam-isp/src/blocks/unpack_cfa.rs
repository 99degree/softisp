//! UnpackCfaBlock — fused packed INT32 or native INT16 Bayer input → CFA FLOAT quadrants.
//!
//! PackedInt32 mode:
//!   Input:  INT32[1,1,H,W/2]    (packed: pixel_even | (pixel_odd << 16))
//!   Output: FLOAT[1,4,H/2,W/2]  (4 Bayer channels: R, Gr, Gb, B)
//!
//! NativeInt16 mode:
//!   Input:  INT16[1,1,H,W]      (raw Bayer samples)
//!   Output: FLOAT[1,4,H/2,W/2]  (4 Bayer channels: R, Gr, Gb, B)
//!
//! Fuses UnpackBlock + NormalizeBlock + CfaBlock (+ optionally BlcBlock)
//! into a single ONNX graph / MNN session. Avoids full-resolution interleave.
//!
//! When use_blc=true: adds Sub(blc_vals) + Clip(0,1) after the CFA Conv.

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnpackMode {
    /// Legacy Android packed buffer: two 16-bit pixels per INT32 word.
    PackedInt32,
    /// Raw sensor samples: one INT16 sample per pixel.
    NativeInt16,
}

pub struct UnpackCfaBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,  // FULL width (original W)
    pub sensor_max: f32,
    pub use_blc: bool,            // fuse black level correction
    pub stride_w: i64,            // 1=no downscale, 2=2× stride in width
    pub mode: UnpackMode,
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
            use_blc: false,
            stride_w: 1,
            mode: UnpackMode::PackedInt32,
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

    pub fn with_blc(mut self, enable: bool) -> Self {
        self.use_blc = enable;
        self
    }

    /// Select raw INT16 input or legacy packed INT32 input.
    pub fn with_mode(mut self, mode: UnpackMode) -> Self {
        self.mode = mode;
        self
    }

    /// Set only the concrete width (height stays symbolic).
    pub fn with_concrete_width(mut self, w: i64) -> Self {
        self.concrete_w = Some(w);
        self
    }

    /// Enable 2× width downscale fused into the CFA Conv.
    /// PackedInt32: stride_w=2 averages over 2 packed columns = 4 actual pixels.
    /// NativeInt16: stride_w=2 averages over 4 actual columns.
    /// Output width = W/2/stride_w in actual pixels.
    pub fn with_downscale(mut self, factor: i64) -> Self {
        self.stride_w = factor.max(1);
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

    fn input_elem_type(&self) -> i32 {
        match self.mode {
            UnpackMode::PackedInt32 => 6, // INT32
            UnpackMode::NativeInt16 => 5, // INT16
        }
    }
    fn output_elem_type(&self) -> i32 { 1 } // FLOAT

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let input_w = self.concrete_w.map(|w| {
            if self.mode == UnpackMode::NativeInt16 {
                w
            } else {
                w / 2
            }
        });
        let dims = match (self.concrete_h, input_w) {
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
                Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W2"),
            ],
        };
        Some(Proto::value_info(&self.input_source, &dims, self.input_elem_type()))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let sw = self.stride_w;
        let dims = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(h / 2), Proto::tensor_dim_value(w / 2 / sw),
            ],
            _ => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H2"), Proto::tensor_dim_param("W2"),
            ],
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        if self.mode == UnpackMode::NativeInt16 {
            return self.native_nodes();
        }
        self.packed_nodes()
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let sm = self.sensor_max;
        let sw = self.stride_w;

        let mut inits = vec![
            Proto::tensor_proto_float_scalar(&format!("{}/max_val", ns), sm),
        ];

        match self.mode {
            UnpackMode::PackedInt32 => {
                // Conv weights [4, 2, 2, sw] — 4 filters, 2 in channels, kernel 2×sw
                // sw=1 (default): kernel=(2,1), each out_ch picks one Bayer position from 1 packed col
                // sw=2 (downscale): kernel=(2,2), each out_ch averages 2 adjacent packed cols
                //
                // Filter 0 (R):  in_ch=0 (even), picks top row
                // Filter 1 (Gr): in_ch=1 (odd),  picks top row
                // Filter 2 (Gb): in_ch=0 (even), picks bottom row
                // Filter 3 (B):  in_ch=1 (odd),  picks bottom row
                //
                // For stride_w=1 (kernel=(2,1)):  flat order: [oc,ic,kh,0]
                //   oc=0,ic=0: [1, 0]
                //   oc=1,ic=1: [1, 0]
                //   oc=2,ic=0: [0, 1]
                //   oc=3,ic=1: [0, 1]
                //
                // For stride_w=2 (kernel=(2,2)):  flat order: [oc,ic,kh,kw]
                //   oc=0,ic=0: [0.5, 0.5, 0, 0]  ← avg R over 2 packed cols
                //   oc=1,ic=1: [0.5, 0.5, 0, 0]  ← avg Gr over 2 packed cols
                //   oc=2,ic=0: [0, 0, 0.5, 0.5]  ← avg Gb over 2 packed cols
                //   oc=3,ic=1: [0, 0, 0.5, 0.5]  ← avg B over 2 packed cols
                let w: Vec<f32> = if sw == 1 {
                    vec![
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
                    ]
                } else {
                    // stride_w=2: kernel=(2,2), average over 2 packed cols
                    vec![
                        // oc=0 (R), ic=0 (even): avg over cols 0,1 top row
                        0.5, 0.5, 0.0, 0.0,
                        // oc=0 (R), ic=1 (odd): none
                        0.0, 0.0, 0.0, 0.0,
                        // oc=1 (Gr), ic=0 (even): none
                        0.0, 0.0, 0.0, 0.0,
                        // oc=1 (Gr), ic=1 (odd): avg over cols 0,1 top row
                        0.5, 0.5, 0.0, 0.0,
                        // oc=2 (Gb), ic=0 (even): avg over cols 0,1 bottom row
                        0.0, 0.0, 0.5, 0.5,
                        // oc=2 (Gb), ic=1 (odd): none
                        0.0, 0.0, 0.0, 0.0,
                        // oc=3 (B), ic=0 (even): none
                        0.0, 0.0, 0.0, 0.0,
                        // oc=3 (B), ic=1 (odd): avg over cols 0,1 bottom row
                        0.0, 0.0, 0.5, 0.5,
                    ]
                };

                let w_shape = if sw == 1 {
                    vec![4i64, 2, 2, 1]
                } else {
                    vec![4i64, 2, 2, 2]
                };

                inits.push(Proto::tensor_proto_int32_scalar(&format!("{}/div_65536", ns), 65536));
                inits.push(Proto::tensor_proto_int32_scalar(&format!("{}/mod_65536", ns), 65536));
                inits.push(Proto::tensor_proto_float(&format!("{}/cfa_w", ns), &w_shape, &w));
                inits.push(Proto::tensor_proto_float(&format!("{}/cfa_b", ns), &[4], &[0f32; 4]));
            }
            UnpackMode::NativeInt16 => {
                // Native input is already one Bayer sample per pixel. Conv kernel is
                // [4, 1, 2, 2*stride_w] and selects/averages the RGGB positions.
                let kw = (2 * sw).max(2);
                let w: Vec<f32> = if sw == 1 {
                    vec![
                        // R:  top-left
                        1.0, 0.0, 0.0, 0.0,
                        // Gr: top-right
                        0.0, 1.0, 0.0, 0.0,
                        // Gb: bottom-left
                        0.0, 0.0, 1.0, 0.0,
                        // B:  bottom-right
                        0.0, 0.0, 0.0, 1.0,
                    ]
                } else {
                    vec![
                        // R:  top row cols 0,2
                        0.5, 0.0, 0.5, 0.0,
                        // Gr: top row cols 1,3
                        0.0, 0.5, 0.0, 0.5,
                        // Gb: bottom row cols 0,2
                        0.5, 0.0, 0.5, 0.0,
                        // B:  bottom row cols 1,3
                        0.0, 0.5, 0.0, 0.5,
                    ]
                };

                inits.push(Proto::tensor_proto_float(
                    &format!("{}/cfa_w", ns),
                    &[4i64, 1, 2, kw],
                    &w,
                ));
                inits.push(Proto::tensor_proto_float(&format!("{}/cfa_b", ns), &[4], &[0f32; 4]));
            }
        }

        if self.use_blc {
            inits.push(Proto::tensor_proto_float(&format!("{}/blc_vals", ns), &[1, 4, 1, 1], &[0.0, 0.0, 0.0, 0.0]));
            inits.push(Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0));
            inits.push(Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0));
        }
        inits
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        let mut extras = vec![
            (format!("{}/max_val", ns), 1, vec![]),
        ];
        if self.mode == UnpackMode::PackedInt32 {
            extras.push((format!("{}/div_65536", ns), 6, vec![]));
            extras.push((format!("{}/mod_65536", ns), 6, vec![]));
        }
        if self.use_blc {
            extras.push((format!("{}/blc_vals", ns), 1, vec![1, 4, 1, 1]));
        }
        extras
    }
}

impl UnpackCfaBlock {
    fn packed_nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();

        vec![
            // Extract even (low 16 bits) via integer Mod by 65536, then cast to FLOAT.
            Proto::node("Mod", &[&self.input_source, &format!("{}/mod_65536", ns)],
                &[&format!("{}/even", ns)], &[]),
            Proto::node("Cast", &[&format!("{}/even", ns)], &[&format!("{}/even_float", ns)],
                &[Proto::attribute_int("to", 1)]),
            Proto::node("Div", &[&format!("{}/even_float", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/even_norm", ns)], &[]),

            // Extract odd (high 16 bits) via integer Div by 65536, then cast to FLOAT.
            Proto::node("Div", &[&self.input_source, &format!("{}/div_65536", ns)],
                &[&format!("{}/odd", ns)], &[]),
            Proto::node("Cast", &[&format!("{}/odd", ns)], &[&format!("{}/odd_float", ns)],
                &[Proto::attribute_int("to", 1)]),
            Proto::node("Div", &[&format!("{}/odd_float", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/odd_norm", ns)], &[]),

            // Stack even + odd into [1,2,H,W/2]
            Proto::node("Concat",
                &[&format!("{}/even_norm", ns), &format!("{}/odd_norm", ns)],
                &[&format!("{}/stacked", ns)],
                &[Proto::attribute_int("axis", 1)]),

            // 9: Conv stride=(2, stride_w) kernel=(2, stride_w) → [1,4,H/2,W/2/stride_w]
            { // separate filters for RGGB pattern
                let cfa_out = if self.use_blc {
                    format!("{}/cfa_out", ns)
                } else {
                    self.frame_tensor.clone()
                };
                Proto::node("Conv",
                    &[&format!("{}/stacked", ns), &format!("{}/cfa_w", ns), &format!("{}/cfa_b", ns)],
                    &[&cfa_out],
                    &[
                        Proto::attribute_ints("kernel_shape", &[2, self.stride_w]),
                        Proto::attribute_ints("strides", &[2, self.stride_w]),
                        Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                        Proto::attribute_int("group", 1),
                    ])
            },
        ].into_iter().chain(
            // Optional BLC: Sub(blc_vals) + Clip(0,1)
            if self.use_blc {
                vec![
                    Proto::node("Sub",
                        &[&format!("{}/cfa_out", ns), &format!("{}/blc_vals", ns)],
                        &[&format!("{}/subbed", ns)],
                        &[]),
                    Proto::node("Clip",
                        &[&format!("{}/subbed", ns), &format!("{}/zero", ns), &format!("{}/one", ns)],
                        &[&self.frame_tensor],
                        &[]),
                ]
            } else {
                vec![]
            }
        ).collect()
    }

    fn native_nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let cast_float = format!("{}/input_float", ns);
        let cfa_out = if self.use_blc {
            format!("{}/cfa_out", ns)
        } else {
            self.frame_tensor.clone()
        };
        let stride = (2 * self.stride_w).max(2);

        let mut nodes = vec![
            // INT16 raw Bayer → FLOAT normalized Bayer
            Proto::node("Cast", &[&self.input_source], &[&cast_float],
                &[Proto::attribute_int("to", 1)]),
            // Conv kernel [4,1,2,2*stride_w], stride=(2,2*stride_w) → [1,4,H/2,W/2/stride_w]
            Proto::node("Conv",
                &[&cast_float, &format!("{}/cfa_w", ns), &format!("{}/cfa_b", ns)],
                &[&cfa_out],
                &[
                    Proto::attribute_ints("kernel_shape", &[2, stride]),
                    Proto::attribute_ints("strides", &[2, stride]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                    Proto::attribute_int("group", 1),
                ]),
        ];

        if self.use_blc {
            nodes.push(Proto::node("Sub",
                &[&format!("{}/cfa_out", ns), &format!("{}/blc_vals", ns)],
                &[&format!("{}/subbed", ns)],
                &[]));
            nodes.push(Proto::node("Clip",
                &[&format!("{}/subbed", ns), &format!("{}/zero", ns), &format!("{}/one", ns)],
                &[&self.frame_tensor],
                &[]));
        }

        nodes
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_unpack_cfa_generates_nodes_packed() {
        let block = UnpackCfaBlock::new().with_concrete_dims(48, 64);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 8, "Packed UnpackCfaBlock (no BLC) should produce 8 nodes");
        let inits = block.initializers();
        assert_eq!(inits.len(), 5, "Packed UnpackCfaBlock (no BLC) should have 5 initializers");

        let block2 = UnpackCfaBlock::new().with_concrete_dims(48, 64).with_blc(true);
        assert_eq!(block2.nodes().len(), 10, "Packed UnpackCfaBlock + BLC should produce 10 nodes");
        assert_eq!(block2.initializers().len(), 8, "Packed UnpackCfaBlock + BLC should have 8 initializers");
        assert_eq!(block2.extra_inputs().len(), 4, "Packed UnpackCfaBlock + BLC should have 4 extra inputs");
    }

    #[test]
    fn test_unpack_cfa_generates_nodes_native() {
        let block = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 2, "Native UnpackCfaBlock (no BLC) should produce 2 nodes");
        let inits = block.initializers();
        assert_eq!(inits.len(), 3, "Native UnpackCfaBlock (no BLC) should have 3 initializers");
        assert_eq!(block.input_elem_type(), 5);
        assert_eq!(block.extra_inputs().len(), 1);

        let block2 = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_blc(true);
        assert_eq!(block2.nodes().len(), 4, "Native UnpackCfaBlock + BLC should produce 4 nodes");
        assert_eq!(block2.initializers().len(), 6, "Native UnpackCfaBlock + BLC should have 6 initializers");
        assert_eq!(block2.extra_inputs().len(), 2);
    }

    #[test]
    fn test_unpack_cfa_value_info() {
        let block = UnpackCfaBlock::new().with_concrete_dims(48, 64);
        let in_vi = block.input_value_info().unwrap();
        let out_vi = block.output_value_info().unwrap();
        assert!(!in_vi.is_empty());
        assert!(!out_vi.is_empty());

        let native = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64);
        let native_in_vi = native.input_value_info().unwrap();
        assert!(!native_in_vi.is_empty());
    }

    #[test]
    fn test_unpack_cfa_pipeline_integration_packed() {
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
        assert!(result.is_ok(), "Packed UnpackCfaBlock pipeline should compose: {:?}", result.err());
        let model = result.unwrap();
        assert!(!model.is_empty(), "Model should not be empty");
    }

    #[test]
    fn test_unpack_cfa_pipeline_integration_native() {
        let b1: Box<dyn IspBlock> = Box::new(crate::blocks::RawInputBlock::new()
            .with_elem_type(5)   // INT16 input
            .with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64));

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(result.is_ok(), "Native UnpackCfaBlock pipeline should compose: {:?}", result.err());
    }
}
