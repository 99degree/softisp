//! UnpackCfaBlock — Bayer input → 4-channel CFA quadrants via SpaceToDepth.
//!
//! PackedInt32 mode (legacy):
//!   Input:  INT32`[1,1,H,W/2]`    (packed: pixel_even | (pixel_odd << 16))
//!   Uses:   integer split → Cast → Conv(2×2,stride=2) → `[1,4,H/2,W/2]`
//!
//! NativeInt16 mode (default):
//!   Input:  INT16`[1,1,H,W]`      (raw Bayer samples)
//!   Uses:   SpaceToDepth(blocksize=2) → `[1,4,H/2,W/2]`
//!           When use_blc|use_wb: Cast(INT16→F32) → SpaceToDepth → F32`[1,4,H/2,W/2]`
//!           Otherwise: SpaceToDepth → INT16`[1,4,H/2,W/2]` (1 node, no Cast)
//!   BLC + WB are fused into DemosaicCcmBlock's Conv1×1 weights.
//!
//! Fuses UnpackBlock + NormalizeBlock + CfaBlock (+ optionally BlcBlock, BayerWbBlock)
//! into a single ONNX graph / MNN session. Avoids full-resolution interleave.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

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
    pub concrete_w: Option<i64>, // FULL width (original W)
    pub sensor_max: f32,
    pub valid_bits: i64, // 10 or 12 valid bits in sensor data
    pub use_blc: bool,   // fuse black level correction
    pub use_wb: bool,    // fuse white balance gains
    pub stride_w: i64,   // 1=no downscale, 2=2× stride in width (PackedInt32 only)
    pub mode: UnpackMode,
    /// Use Conv-based unpack (fast) instead of SpaceToDepth.
    /// This replaces multi-dispatch Raster with a single VulkanConvolution dispatch.
    pub use_fast_unpack: bool,
    /// Extra height downscale after CFA Conv (1=none, 2=half height).
    /// Fuses Resize(H/2) into the UnpackCfa output for 4K→FHD-like pipelines.
    pub height_downscale: i64,
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
            valid_bits: 10,
            use_blc: false,
            use_wb: false,
            stride_w: 1,
            mode: UnpackMode::PackedInt32,
            use_fast_unpack: false,
            height_downscale: 1,
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

    /// Set valid bits: 10 or 12. If 12, values are shifted right by 2 to get
    /// 10-bit mantissa (perfect fp16 precision). sensor_max should be 1023.0
    /// for both 10-bit and 12-bit (the shift normalizes to 10-bit range).
    pub fn with_valid_bits(mut self, bits: i64) -> Self {
        self.valid_bits = bits;
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

    /// Enable extra height downscale (2×) after the CFA Conv.
    /// Fuses a Resize(H/2) node into the output, giving `[1,4,H/4,W/2/stride_w]`.
    /// Useful for 4K→FHD pipelines where FCS/LDCI/EE run at lower resolution.
    pub fn with_height_downscale(mut self, factor: i64) -> Self {
        self.height_downscale = factor.max(1);
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
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "UnpackCfaBlock".to_string()
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

    fn input_elem_type(&self) -> i32 {
        match self.mode {
            UnpackMode::PackedInt32 => 6, // INT32
            UnpackMode::NativeInt16 => 5, // INT16 — Cast to F32 fused by IspChainFusion
        }
    }
    fn output_elem_type(&self) -> i32 {
        1 // FLOAT32 — Conv output is always float
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
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ],
            (None, Some(w)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_value(w),
            ],
            _ => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W2"),
            ],
        };
        Some(Proto::value_info(
            &self.input_source,
            &dims,
            self.input_elem_type(),
        ))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let sw = self.stride_w;
        let hd = self.height_downscale.max(1);
        let elem_type = self.output_elem_type();
        let dims = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => {
                // PackedInt32: input width is W/2, Conv outputs H/2 × W/2/sw
                // NativeInt16: input width is W, Conv outputs H/2 × W/sw
                let (out_h, out_w) = if self.mode == UnpackMode::PackedInt32 {
                    (h / 2 / hd, w / 2 / sw)
                } else {
                    (h / 2 / hd, w / sw)
                };
                vec![
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_value(4),
                    Proto::tensor_dim_value(out_h),
                    Proto::tensor_dim_value(out_w),
                ]
            }
            _ => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H2"),
                Proto::tensor_dim_param("W2"),
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
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        let sw = self.stride_w;
        let mut inputs = vec![(format!("{}/max_val", ns).to_string(), 1, vec![])];
        match self.mode {
            UnpackMode::PackedInt32 => {
                let w_shape = if sw == 1 {
                    vec![4i64, 2, 2, 1]
                } else {
                    vec![4i64, 2, 2, 2]
                };
                inputs.push((format!("{}/div_65536", ns).to_string(), 6, vec![]));
                inputs.push((format!("{}/mod_65536", ns).to_string(), 6, vec![]));
                inputs.push((format!("{}/cfa_w", ns).to_string(), 1, w_shape));
                inputs.push((format!("{}/cfa_b", ns).to_string(), 1, vec![4]));
                if self.valid_bits == 12 {
                    inputs.push((format!("{}/shr_4", ns).to_string(), 1, vec![]));
                }
            }
            UnpackMode::NativeInt16 => {
                if self.use_fast_unpack {
                    let w_shape = vec![4i64, 1, 2, sw];
                    inputs.push((format!("{}/unpack_w", ns).to_string(), 1, w_shape));
                    inputs.push((format!("{}/unpack_b", ns).to_string(), 1, vec![4]));
                }
            }
        }
        if self.use_blc {
            inputs.push((format!("{}/blc_vals", ns).to_string(), 1, vec![1, 4, 1, 1]));
        }
        if self.use_wb {
            inputs.push((format!("{}/wb_gains", ns).to_string(), 1, vec![1, 4, 1, 1]));
        }
        if (self.use_blc || self.use_wb) && matches!(self.mode, UnpackMode::PackedInt32) {
            inputs.push((format!("{}/zero", ns).to_string(), 1, vec![]));
            inputs.push((format!("{}/one", ns).to_string(), 1, vec![]));
        }
        if self.height_downscale > 1 {
            inputs.push((format!("{}/scale_h", ns).to_string(), 1, vec![4]));
        }
        inputs
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        let sm = self.sensor_max;
        let sw = self.stride_w;
        let mut defaults = vec![(
            format!("{}/max_val", ns).to_string(),
            sm.to_ne_bytes().to_vec(),
        )];
        match self.mode {
            UnpackMode::PackedInt32 => {
                let w: Vec<f32> = if sw == 1 {
                    vec![
                        1f32, 0f32, 0f32, 0f32, 0f32, 1f32, 0f32, 0f32, 0f32, 0f32, 1f32, 0f32,
                        0f32, 0f32, 0f32, 1f32,
                    ]
                } else {
                    vec![
                        0.5f32, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5,
                    ]
                };
                defaults.push((
                    format!("{}/div_65536", ns).to_string(),
                    (65536i32).to_ne_bytes().to_vec(),
                ));
                defaults.push((
                    format!("{}/mod_65536", ns).to_string(),
                    (65536i32).to_ne_bytes().to_vec(),
                ));
                defaults.push((
                    format!("{}/cfa_w", ns).to_string(),
                    w.iter().flat_map(|v| v.to_ne_bytes()).collect::<Vec<u8>>(),
                ));
                defaults.push((
                    format!("{}/cfa_b", ns).to_string(),
                    [0.0f32; 4]
                        .iter()
                        .flat_map(|v| v.to_ne_bytes())
                        .collect::<Vec<u8>>(),
                ));
                if self.valid_bits == 12 {
                    defaults.push((
                        format!("{}/shr_4", ns).to_string(),
                        (4.0f32).to_ne_bytes().to_vec(),
                    ));
                }
            }
            UnpackMode::NativeInt16 => {
                if self.use_fast_unpack {
                    let w: Vec<f32> = if sw <= 1 {
                        vec![1.0f32, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
                    } else {
                        let mut w2 = vec![0.0f32; 4 * 2 * sw as usize];
                        w2[0] = 1.0;
                        w2[sw as usize / 2] = 1.0;
                        w2[sw as usize] = 1.0;
                        w2[sw as usize + sw as usize / 2] = 1.0;
                        w2
                    };
                    defaults.push((
                        format!("{}/unpack_w", ns).to_string(),
                        w.iter().flat_map(|v| v.to_ne_bytes()).collect::<Vec<u8>>(),
                    ));
                    defaults.push((
                        format!("{}/unpack_b", ns).to_string(),
                        [0.0f32; 4]
                            .iter()
                            .flat_map(|v| v.to_ne_bytes())
                            .collect::<Vec<u8>>(),
                    ));
                }
            }
        }
        if self.use_blc {
            defaults.push((
                format!("{}/blc_vals", ns).to_string(),
                [0.0f32, 0.0, 0.0, 0.0]
                    .iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect::<Vec<u8>>(),
            ));
        }
        if self.use_wb {
            defaults.push((
                format!("{}/wb_gains", ns).to_string(),
                [1.0f32, 1.0, 1.0, 1.0]
                    .iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect::<Vec<u8>>(),
            ));
        }
        if (self.use_blc || self.use_wb) && matches!(self.mode, UnpackMode::PackedInt32) {
            defaults.push((
                format!("{}/zero", ns).to_string(),
                (0.0f32).to_ne_bytes().to_vec(),
            ));
            defaults.push((
                format!("{}/one", ns).to_string(),
                (1.0f32).to_ne_bytes().to_vec(),
            ));
        }
        if self.height_downscale > 1 {
            defaults.push((
                format!("{}/scale_h", ns).to_string(),
                [1.0f32, 1.0, 0.5, 1.0]
                    .iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect::<Vec<u8>>(),
            ));
        }
        defaults
    }
}

impl UnpackCfaBlock {
    fn packed_nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let cfa_out = format!("{}/cfa_out", ns);
        let after_blc = format!("{}/after_blc", ns);

        let mut nodes = vec![
            // Extract even (low 16 bits) via integer Mod by 65536, then cast to FLOAT.
            Proto::node(
                "Mod",
                &[&self.input_source, &format!("{}/mod_65536", ns)],
                &[&format!("{}/even", ns)],
                &[],
            ),
            Proto::node(
                "Cast",
                &[&format!("{}/even", ns)],
                &[&format!("{}/even_float", ns)],
                &[Proto::attribute_int("to", 1)],
            ),
            // Extract odd (high 16 bits) via integer Div by 65536, then cast to FLOAT.
            Proto::node(
                "Div",
                &[&self.input_source, &format!("{}/div_65536", ns)],
                &[&format!("{}/odd", ns)],
                &[],
            ),
            Proto::node(
                "Cast",
                &[&format!("{}/odd", ns)],
                &[&format!("{}/odd_float", ns)],
                &[Proto::attribute_int("to", 1)],
            ),
        ];

        // 12-bit data: shift right by 2 to get 10-bit mantissa
        // SHR(x,2) = Div(x, 4.0) — perfect fp16 precision for [0,1] output
        if self.valid_bits == 12 {
            let shr_init = format!("{}/shr_4", ns);
            nodes.push(Proto::node(
                "Div",
                &[&format!("{}/even_float", ns), &shr_init],
                &[&format!("{}/even_shr", ns)],
                &[],
            ));
            nodes.push(Proto::node(
                "Div",
                &[&format!("{}/odd_float", ns), &shr_init],
                &[&format!("{}/odd_shr", ns)],
                &[],
            ));
            // Normalize to [0,1]: sensor_max = 1023.0 (10-bit after shift)
            nodes.push(Proto::node(
                "Div",
                &[&format!("{}/even_shr", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/even_norm", ns)],
                &[],
            ));
            nodes.push(Proto::node(
                "Div",
                &[&format!("{}/odd_shr", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/odd_norm", ns)],
                &[],
            ));
        } else {
            // 10-bit data: normalize directly to [0,1]
            nodes.push(Proto::node(
                "Div",
                &[&format!("{}/even_float", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/even_norm", ns)],
                &[],
            ));
            nodes.push(Proto::node(
                "Div",
                &[&format!("{}/odd_float", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/odd_norm", ns)],
                &[],
            ));
        }

        // Stack even + odd into [1,2,H,W/2]
        nodes.push(Proto::node(
            "Concat",
            &[&format!("{}/even_norm", ns), &format!("{}/odd_norm", ns)],
            &[&format!("{}/stacked", ns)],
            &[Proto::attribute_int("axis", 1)],
        ));

        // Conv stride=(2, stride_w) → [1,4,H/2,W/2/stride_w]
        nodes.push(Proto::node(
            "Conv",
            &[
                &format!("{}/stacked", ns),
                &format!("{}/cfa_w", ns),
                &format!("{}/cfa_b", ns),
            ],
            &[&cfa_out],
            &[
                Proto::attribute_ints("kernel_shape", &[2, self.stride_w]),
                Proto::attribute_ints("strides", &[2, self.stride_w]),
                Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                Proto::attribute_int("group", 1),
            ],
        ));

        // Optional BLC: Sub(blc_vals) + Clip(0,1), then denormalize
        if self.use_blc {
            nodes.push(Proto::node(
                "Sub",
                &[&cfa_out, &format!("{}/blc_vals", ns)],
                &[&after_blc],
                &[],
            ));
            nodes.push(Proto::node(
                "Clip",
                &[&after_blc, &format!("{}/zero", ns), &format!("{}/one", ns)],
                &[&format!("{}/clip_out", ns)],
                &[],
            ));
            // Denormalize: Mul(sensor_max) → output as float (Vulkan compatible)
            nodes.push(Proto::node(
                "Mul",
                &[&format!("{}/clip_out", ns), &format!("{}/max_val", ns)],
                &[&self.frame_tensor],
                &[],
            ));
        } else {
            // Denormalize: Mul(sensor_max) → output as float (Vulkan compatible)
            nodes.push(Proto::node(
                "Mul",
                &[&cfa_out, &format!("{}/max_val", ns)],
                &[&self.frame_tensor],
                &[],
            ));
        }

        // Extra height downscale: Resize(H/2) after CFA output
        if self.height_downscale > 1 {
            let resized = format!("{}/resized_h", ns);
            let scale_h = format!("{}/scale_h", ns);
            // Replace frame_tensor with resized output
            let prev_out = self.frame_tensor.clone();
            nodes.push(Proto::node(
                "Resize",
                &[&prev_out, "", &scale_h],
                &[&resized],
                &[Proto::attribute_int("mode", 0)],
            )); // 0 = nearest
                // Final identity to rename to frame_tensor
            nodes.push(Proto::node(
                "Identity",
                &[&resized],
                &[&self.frame_tensor],
                &[],
            ));
        }

        nodes
    }

    fn native_nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let sw = self.stride_w;

        // Cast(INT16→F32) + Conv — IspChainFusion fuses into single GPU shader
        vec![
            Proto::node(
                "Cast",
                &[&self.input_source],
                &[&format!("{}/input_f32", ns)],
                &[Proto::attribute_int("to", 1)],
            ),
            Proto::node(
                "Conv",
                &[
                    &format!("{}/input_f32", ns),
                    &format!("{}/unpack_w", ns),
                    &format!("{}/unpack_b", ns),
                ],
                &[&self.frame_tensor],
                &[
                    Proto::attribute_ints("kernel_shape", &[2, sw]),
                    Proto::attribute_ints("strides", &[2, sw]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                    Proto::attribute_int("group", 1),
                ],
            ),
        ]
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
        assert_eq!(
            nodes.len(),
            9,
            "Packed UnpackCfaBlock (no BLC) should produce 9 nodes (no Cast to INT16)"
        );
        let inits = block.extra_input_defaults();
        assert_eq!(
            inits.len(),
            5,
            "Packed UnpackCfaBlock (no BLC) should have 5 initializers"
        );

        let block2 = UnpackCfaBlock::new()
            .with_concrete_dims(48, 64)
            .with_blc(true);
        assert_eq!(
            block2.nodes().len(),
            11,
            "Packed UnpackCfaBlock + BLC should produce 11 nodes (+Sub+Clip+Mul)"
        );
        assert_eq!(
            block2.extra_input_defaults().len(),
            8,
            "Packed UnpackCfaBlock + BLC should have 8 initializers"
        );
        assert_eq!(
            block2.extra_inputs().len(),
            8,
            "Packed UnpackCfaBlock + BLC should have 8 extra inputs"
        );
    }

    #[test]
    fn test_unpack_cfa_generates_nodes_native() {
        // Without BLC/WB: just SpaceToDepth → 1 node, no extra initializers
        let block = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 2, "Native no-BLC/WB: Cast+Conv = 2 nodes");
        let inits = block.extra_input_defaults();
        assert_eq!(
            inits.len(),
            1,
            "Native no-BLC/WB: just max_val = 1 initializer"
        );
        assert_eq!(block.input_elem_type(), 5, "Native input should be INT16");
        assert_eq!(block.output_elem_type(), 1, "Native no-BLC/WB: output F32");
        assert_eq!(block.extra_inputs().len(), 1);

        // With BLC only: Conv = 1 node (F32 output, no Sub/Clip/Cast)
        let block2 = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_blc(true);
        assert_eq!(block2.nodes().len(), 2, "Native + BLC: Cast+Conv = 2 nodes");
        assert_eq!(block2.output_elem_type(), 1, "Native + BLC: output F32");
        // max_val + blc_vals = 2 (no zero/one for NativeInt16 PackedInt32 mode)
        assert_eq!(
            block2.extra_input_defaults().len(),
            2,
            "Native + BLC: max_val+blc_vals = 2"
        );
        // max_val + blc_vals
        assert_eq!(
            block2.extra_inputs().len(),
            2,
            "Native + BLC: 2 extra inputs"
        );

        // With WB only: Conv = 1 node
        let block3 = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_wb(true);
        assert_eq!(block3.nodes().len(), 2, "Native + WB: Cast+Conv = 2 nodes");
        assert_eq!(block3.output_elem_type(), 1, "Native + WB: output F32");
        // max_val + wb_gains = 2
        assert_eq!(
            block3.extra_input_defaults().len(),
            2,
            "Native + WB: max_val+wb_gains = 2"
        );
        // max_val + wb_gains
        assert_eq!(
            block3.extra_inputs().len(),
            2,
            "Native + WB: 2 extra inputs"
        );

        // With BLC + WB: Conv = 1 node (same)
        let block4 = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_blc(true)
            .with_wb(true);
        assert_eq!(
            block4.nodes().len(),
            2,
            "Native + BLC+WB: Cast+Conv = 2 nodes"
        );
        assert_eq!(block4.output_elem_type(), 1, "Native + BLC+WB: output F32");
        // max_val + blc_vals + wb_gains = 3
        assert_eq!(
            block4.extra_input_defaults().len(),
            3,
            "Native + BLC+WB: max_val+blc_vals+wb_gains = 3"
        );
        // max_val + blc_vals + wb_gains
        assert_eq!(
            block4.extra_inputs().len(),
            3,
            "Native + BLC+WB: 3 extra inputs"
        );
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
        let b1: Box<dyn IspBlock> =
            Box::new(crate::blocks::RawInputBlock::new().with_concrete_dims(48, 32)); // packed width = 32
        let b2: Box<dyn IspBlock> = Box::new(UnpackCfaBlock::new().with_concrete_dims(48, 64));
        let b3: Box<dyn IspBlock> = Box::new(crate::blocks::BlcBlock::new());

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(
            result.is_ok(),
            "Packed UnpackCfaBlock pipeline should compose: {:?}",
            result.err()
        );
        let model = result.unwrap();
        assert!(!model.is_empty(), "Model should not be empty");
    }

    #[test]
    fn test_unpack_cfa_pipeline_integration_native() {
        let b1: Box<dyn IspBlock> =
            Box::new(crate::blocks::RawInput16Block::new().with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(
            UnpackCfaBlock::new()
                .with_mode(UnpackMode::NativeInt16)
                .with_concrete_dims(48, 64),
        );

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(
            result.is_ok(),
            "Native UnpackCfaBlock pipeline should compose: {:?}",
            result.err()
        );
    }

    #[test]
    fn test_unpack_cfa_fast_path() {
        // Test the Conv-based fast unpack replaces SpaceToDepth with Conv
        let fast = UnpackCfaBlock::new()
            .with_mode(UnpackMode::NativeInt16)
            .with_concrete_dims(48, 64)
            .with_fast_unpack(true);

        // Fast path: Cast(INT16→F32) + Conv(4ch,2×sw,stride=2×sw) = 2 nodes
        let nodes = fast.nodes();
        assert_eq!(nodes.len(), 2, "Fast unpack: Cast+Conv = 2 nodes");
        assert_eq!(fast.output_elem_type(), 1, "Fast unpack: FLOAT output");

        // Check Conv weights exist
        let inits = fast.extra_input_defaults();
        assert!(
            inits.len() >= 3,
            "Fast unpack: max_val + unpack_w + unpack_b >= 3"
        );

        // Verify pipeline integration
        let b1: Box<dyn IspBlock> =
            Box::new(crate::blocks::RawInput16Block::new().with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(
            UnpackCfaBlock::new()
                .with_mode(UnpackMode::NativeInt16)
                .with_concrete_dims(48, 64)
                .with_fast_unpack(true),
        );
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(
            result.is_ok(),
            "Fast unpack pipeline should compose: {:?}",
            result.err()
        );
    }
}
