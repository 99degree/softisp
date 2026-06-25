//! UnpackCfaBlock — Bayer input → 4-channel CFA quadrants via SpaceToDepth.
//!
//! PackedInt32 mode (legacy):
//!   Input:  INT32[1,1,H,W/2]    (packed: pixel_even | (pixel_odd << 16))
//!   Uses:   integer split → Cast → Conv(2×2,stride=2) → [1,4,H/2,W/2]
//!
//! NativeInt16 mode (default):
//!   Input:  INT16[1,1,H,W]      (raw Bayer samples)
//!   Uses:   SpaceToDepth(blocksize=2) → [1,4,H/2,W/2]
//!           When use_blc|use_wb: Cast(INT16→F32) → SpaceToDepth → F32[1,4,H/2,W/2]
//!           Otherwise: SpaceToDepth → INT16[1,4,H/2,W/2] (1 node, no Cast)
//!   BLC + WB are fused into DemosaicCcmBlock's Conv1×1 weights.
//!
//! Fuses UnpackBlock + NormalizeBlock + CfaBlock (+ optionally BlcBlock, BayerWbBlock)
//! into a single ONNX graph / MNN session. Avoids full-resolution interleave.

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
    pub use_wb: bool,             // fuse white balance gains
    pub stride_w: i64,            // 1=no downscale, 2=2× stride in width (PackedInt32 only)
    pub mode: UnpackMode,
    /// Use Conv-based unpack (fast) instead of SpaceToDepth.
    /// This replaces multi-dispatch Raster with a single VulkanConvolution dispatch.
    pub use_fast_unpack: bool,
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
            use_wb: false,
            stride_w: 1,
            mode: UnpackMode::PackedInt32,
            use_fast_unpack: false,
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

    pub fn with_wb(mut self, enable: bool) -> Self {
        self.use_wb = enable;
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

    /// Enable Conv-based fast unpack. When enabled, Cast(INT16→F32) + SpaceToDepth
    /// is replaced with Cast(INT16→F32) + Conv(4ch, 2×2, stride=2), avoiding the
    /// multi-dispatch Raster overhead of standard SpaceToDepth.
    pub fn with_fast_unpack(mut self, enable: bool) -> Self {
        self.use_fast_unpack = enable;
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
    fn output_elem_type(&self) -> i32 {
        if self.use_fast_unpack || ((self.use_blc || self.use_wb) && self.mode == UnpackMode::NativeInt16) {
            1 // FLOAT — fast Conv path or BLC/WB fused downstream
        } else {
            5 // INT16 — Bayer domain integer
        }
    }

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
        let elem_type = self.output_elem_type();
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
        Some(Proto::value_info(&self.frame_tensor, &dims, elem_type))
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
                // SpaceToDepth(blocksize=2) — no Conv weights needed for standard path.
                // For fast path, Conv weights extract 4 Bayer positions from 2×2 blocks.
                if self.use_fast_unpack {
                    // Conv weights [4, 1, 2, 2] — 4 out_ch, 1 in_ch, kernel 2×2
                    // Each out_ch picks one of 4 positions in each 2×2 block:
                    // out_ch 0 (R):  TL → (kh=0, kw=0) = 1
                    // out_ch 1 (Gr): TR → (kh=0, kw=1) = 1
                    // out_ch 2 (Gb): BL → (kh=1, kw=0) = 1
                    // out_ch 3 (B):  BR → (kh=1, kw=1) = 1
                    let unpack_w = vec![
                        1.0, 0.0, 0.0, 0.0, // oc=0: TL
                        0.0, 1.0, 0.0, 0.0, // oc=1: TR
                        0.0, 0.0, 1.0, 0.0, // oc=2: BL
                        0.0, 0.0, 0.0, 1.0, // oc=3: BR
                    ];
                    inits.push(Proto::tensor_proto_float(
                        &format!("{}/unpack_w", ns), &[4, 1, 2, 2], &unpack_w));
                    inits.push(Proto::tensor_proto_float(
                        &format!("{}/unpack_b", ns), &[4], &[0.0; 4]));
                }
            }
        }

        if self.use_blc || self.use_wb {
            // BLC/WB values are kept as initializers for the controller
            // to use when computing DemosaicCcmBlock's fused weights.
            // The ONNX nodes do NOT reference these — they are NOT graph inputs.
            // But having them here doesn't hurt (unused initializers are ignored).
            if self.use_blc {
                inits.push(Proto::tensor_proto_float(&format!("{}/blc_vals", ns), &[1, 4, 1, 1], &[0.0, 0.0, 0.0, 0.0]));
            }
            if self.use_wb {
                inits.push(Proto::tensor_proto_float(&format!("{}/wb_gains", ns), &[1, 4, 1, 1], &[1.0, 1.0, 1.0, 1.0]));
            }
            // PackedInt32 mode still needs zero/one for its Clip node
            if self.mode == UnpackMode::PackedInt32 {
                inits.push(Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0));
                inits.push(Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0));
            }
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
        if self.use_wb && self.mode == UnpackMode::NativeInt16 {
            extras.push((format!("{}/wb_gains", ns), 1, vec![1, 4, 1, 1]));
        }
        extras
    }
}

impl UnpackCfaBlock {
    fn packed_nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let cfa_out = format!("{}/cfa_out", ns);
        let after_blc = format!("{}/after_blc", ns);

        let mut nodes = vec![
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

            // Conv stride=(2, stride_w) → [1,4,H/2,W/2/stride_w]
            Proto::node("Conv",
                &[&format!("{}/stacked", ns), &format!("{}/cfa_w", ns), &format!("{}/cfa_b", ns)],
                &[&cfa_out],
                &[
                    Proto::attribute_ints("kernel_shape", &[2, self.stride_w]),
                    Proto::attribute_ints("strides", &[2, self.stride_w]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                    Proto::attribute_int("group", 1),
                ]),
        ];

        // Optional BLC: Sub(blc_vals) + Clip(0,1), then denormalize + Cast to INT16
        if self.use_blc {
            nodes.push(Proto::node("Sub",
                &[&cfa_out, &format!("{}/blc_vals", ns)],
                &[&after_blc],
                &[]));
            nodes.push(Proto::node("Clip",
                &[&after_blc, &format!("{}/zero", ns), &format!("{}/one", ns)],
                &[&format!("{}/clip_out", ns)],
                &[]));
            // Denormalize: Mul(sensor_max) → Cast(INT16)
            nodes.push(Proto::node("Mul",
                &[&format!("{}/clip_out", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/denorm", ns)],
                &[]));
            nodes.push(Proto::node("Cast", &[&format!("{}/denorm", ns)], &[&self.frame_tensor],
                &[Proto::attribute_int("to", 5)]));
        } else {
            // Denormalize: Mul(sensor_max) → Cast(INT16)
            nodes.push(Proto::node("Mul",
                &[&cfa_out, &format!("{}/max_val", ns)],
                &[&format!("{}/denorm", ns)],
                &[]));
            nodes.push(Proto::node("Cast", &[&format!("{}/denorm", ns)], &[&self.frame_tensor],
                &[Proto::attribute_int("to", 5)]));
        }

        nodes
    }

    fn native_nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let has_process = self.use_blc || self.use_wb;

        if self.use_fast_unpack {
            // Fast path: Conv(4ch, 2x2, stride=2) replaces SpaceToDepth + Raster.
            // Uses VulkanConvolution's optimized inner-product path instead of Raster's
            // multi-dispatch strided access.
            let nodes = if has_process {
                vec![
                    // Need Cast to F32 first, then Conv
                    Proto::node("Cast", &[&self.input_source], &[&format!("{}/input_f32", ns)],
                        &[Proto::attribute_int("to", 1)]),
                    Proto::node("Conv",
                        &[&format!("{}/input_f32", ns), &format!("{}/unpack_w", ns), &format!("{}/unpack_b", ns)],
                        &[&self.frame_tensor],
                        &[
                            Proto::attribute_ints("kernel_shape", &[2, 2]),
                            Proto::attribute_ints("strides", &[2, 2]),
                            Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                            Proto::attribute_int("group", 1),
                        ]),
                ]
            } else {
                vec![
                    // No processing needed - Cast + Conv
                    Proto::node("Cast", &[&self.input_source], &[&format!("{}/input_f32", ns)],
                        &[Proto::attribute_int("to", 1)]),
                    Proto::node("Conv",
                        &[&format!("{}/input_f32", ns), &format!("{}/unpack_w", ns), &format!("{}/unpack_b", ns)],
                        &[&self.frame_tensor],
                        &[
                            Proto::attribute_ints("kernel_shape", &[2, 2]),
                            Proto::attribute_ints("strides", &[2, 2]),
                            Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                            Proto::attribute_int("group", 1),
                        ]),
                ]
            };
            
            return nodes;
        }

        // Original SpaceToDepth path (standard Raster-based)
        if has_process {
            // Cast(INT16→F32) then SpaceToDepth → F32 [1,4,H/2,W/2]
            // No BLC/WB/Clip nodes — those are fused into DemosaicCcmBlock's Conv.
            vec![
                Proto::node("Cast", &[&self.input_source], &[&format!("{}/input_f32", ns)],
                    &[Proto::attribute_int("to", 1)]),
                Proto::node("SpaceToDepth", &[&format!("{}/input_f32", ns)],
                    &[&self.frame_tensor],
                    &[Proto::attribute_int("blocksize", 2)]),
            ]
        } else {
            // Direct SpaceToDepth → INT16 [1,4,H/2,W/2].
            // No Cast, no processing — just pure memory rearrangement.
            vec![
                Proto::node("SpaceToDepth", &[&self.input_source],
                    &[&self.frame_tensor],
                    &[Proto::attribute_int("blocksize", 2)]),
            ]
        }
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
        assert_eq!(nodes.len(), 10, "Packed UnpackCfaBlock (no BLC) should produce 10 nodes (+Mul+Cast to INT16)");
        let inits = block.initializers();
        assert_eq!(inits.len(), 5, "Packed UnpackCfaBlock (no BLC) should have 5 initializers");

        let block2 = UnpackCfaBlock::new().with_concrete_dims(48, 64).with_blc(true);
        assert_eq!(block2.nodes().len(), 12, "Packed UnpackCfaBlock + BLC should produce 12 nodes (+Sub+Clip+Mul+Cast)");
        assert_eq!(block2.initializers().len(), 8, "Packed UnpackCfaBlock + BLC should have 8 initializers");
        assert_eq!(block2.extra_inputs().len(), 4, "Packed UnpackCfaBlock + BLC should have 4 extra inputs");
    }

    #[test]
    fn test_unpack_cfa_generates_nodes_native() {
        // Without BLC/WB: just SpaceToDepth → 1 node, no extra initializers
        let block = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 1, "Native no-BLC/WB: SpaceToDepth only = 1 node");
        let inits = block.initializers();
        assert_eq!(inits.len(), 1, "Native no-BLC/WB: just max_val = 1 initializer");
        assert_eq!(block.input_elem_type(), 5);
        assert_eq!(block.output_elem_type(), 5);
        assert_eq!(block.extra_inputs().len(), 1);

        // With BLC only: Cast → SpaceToDepth = 2 nodes (F32 output, no Sub/Clip/Cast)
        let block2 = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_blc(true);
        assert_eq!(block2.nodes().len(), 2, "Native + BLC: Cast+SpaceToDepth = 2");
        assert_eq!(block2.output_elem_type(), 1, "Native + BLC: output F32");
        // max_val + blc_vals = 2 (no zero/one for NativeInt16 PackedInt32 mode)
        assert_eq!(block2.initializers().len(), 2, "Native + BLC: max_val+blc_vals = 2");
        // max_val + blc_vals
        assert_eq!(block2.extra_inputs().len(), 2, "Native + BLC: 2 extra inputs");

        // With WB only: Cast → SpaceToDepth = 2 nodes
        let block3 = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_wb(true);
        assert_eq!(block3.nodes().len(), 2, "Native + WB: Cast+SpaceToDepth = 2");
        assert_eq!(block3.output_elem_type(), 1, "Native + WB: output F32");
        // max_val + wb_gains = 2
        assert_eq!(block3.initializers().len(), 2, "Native + WB: max_val+wb_gains = 2");
        // max_val + wb_gains
        assert_eq!(block3.extra_inputs().len(), 2, "Native + WB: 2 extra inputs");

        // With BLC + WB: Cast → SpaceToDepth = 2 nodes (same)
        let block4 = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_blc(true)
            .with_wb(true);
        assert_eq!(block4.nodes().len(), 2, "Native + BLC+WB: Cast+SpaceToDepth = 2");
        assert_eq!(block4.output_elem_type(), 1, "Native + BLC+WB: output F32");
        // max_val + blc_vals + wb_gains = 3
        assert_eq!(block4.initializers().len(), 3, "Native + BLC+WB: max_val+blc_vals+wb_gains = 3");
        // max_val + blc_vals + wb_gains
        assert_eq!(block4.extra_inputs().len(), 3, "Native + BLC+WB: 3 extra inputs");
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

    #[test]
    fn test_unpack_cfa_fast_path() {
        // Test the Conv-based fast unpack replaces SpaceToDepth with Conv
        let fast = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_fast_unpack(true);
        
        // Fast path: Cast(INT16→F32) + Conv(4ch,2×2,stride=2) = 2 nodes
        let nodes = fast.nodes();
        assert_eq!(nodes.len(), 2, "Fast unpack: Cast+Conv = 2 nodes");
        assert_eq!(fast.output_elem_type(), 1, "Fast unpack: FLOAT output");
        
        // Check Conv weights exist
        let inits = fast.initializers();
        assert!(inits.len() >= 3, "Fast unpack: max_val + unpack_w + unpack_b >= 3");
        
        // Verify pipeline integration
        let b1: Box<dyn IspBlock> = Box::new(crate::blocks::RawInputBlock::new()
            .with_elem_type(5)
            .with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_fast_unpack(true));
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(result.is_ok(), "Fast unpack pipeline should compose: {:?}", result.err());
    }
}
