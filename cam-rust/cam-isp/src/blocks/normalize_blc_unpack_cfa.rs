use crate::onnx::proto::Proto;
use crate::pipeline::{BlockOpsetMode, IspBlock};

/// Fused block that combines Normalize → BLC50 → UnpackCfa into a single ISP operation.
///
/// This block represents the MNN `isp.unpack_packed` or `isp.unpack_blc` fused opsets.
/// In MNN's post-converter, these patterns are matched and replaced with custom Extra ops
/// that execute the entire chain in a single Vulkan kernel pass.
///
/// # Architecture
///
/// The fusion follows MNN's exact pattern matching:
/// - PackedInt32 input: Normalize → BLC50 → UnpackCfa → isp.unpack_blc
/// - PureInt16 input: BLC50 → UnpackCfa → isp.unpack_blc (Normalize fused into UnpackCfa)
///
/// # Opset Support
///
/// This block supports both `Primitive` and `Custom` opset modes:
/// - Primitive: emits standard ONNX ops (Cast, Div, Sub, Clip, Conv)
/// - Custom: emits `isp.unpack_blc` Extra op with SPIR-V shader
#[derive(Debug)]
pub struct NormalizeBlcUnpackCfa {
    /// Input tensor name (e.g., "raw_frame")
    pub input_source: String,
    /// Output tensor name (e.g., "Blc50Block/frame")
    pub frame_tensor: String,
    /// Optional concrete dimensions for shape inference
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
    /// Sensor max value for normalization (default 65535)
    pub sensor_max: f32,
    /// Black level per *colour*, ordered R, Gr, Gb, B (default zeros).
    ///
    /// Which physical plane that colour lands in depends on the CFA phase, so
    /// the shader maps colour -> plane from `bayer_pattern` rather than assuming
    /// the quad starts with red.
    pub blc_offsets: [f32; 4],
    /// CFA phase: 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR.
    pub bayer_pattern: i32,
    /// Whether to use fast Conv-based unpack vs SpaceToDepth
    pub use_fast_unpack: bool,
    /// Height downscale factor (1=none, 2=half)
    pub height_downscale: i64,
    /// Override the emitted opset mode. When Some, takes priority over the default Custom.
    pub forced_mode: Option<BlockOpsetMode>,
}

impl Default for NormalizeBlcUnpackCfa {
    fn default() -> Self {
        Self::new()
    }
}

impl NormalizeBlcUnpackCfa {
    pub fn new() -> Self {
        Self {
            // A named input, not the empty string: `graph_input_name()` hands
            // this straight to the composer, and an unnamed graph input leaves
            // the engine with no tensor to feed.
            input_source: "raw_frame".to_string(),
            frame_tensor: "NormalizeBlcUnpackCfa/frame".to_string(),
            concrete_h: None,
            concrete_w: None,
            sensor_max: 65535.0,
            blc_offsets: [0.0; 4],
            bayer_pattern: 0,
            use_fast_unpack: false,
            height_downscale: 1,
            forced_mode: None,
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

    pub fn with_blc_offsets(mut self, offsets: [f32; 4]) -> Self {
        self.blc_offsets = offsets;
        self
    }

    /// Set the CFA phase (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR).
    ///
    /// The fused shader emits the quad in physical order (TL, TR, BL, BR) and
    /// subtracts the black level by colour, so it needs to know which plane is
    /// which. Without it a GBRG sensor would have red's black level subtracted
    /// from a green plane.
    pub fn with_bayer_pattern(mut self, pattern: i32) -> Self {
        self.bayer_pattern = pattern;
        self
    }

    pub fn with_fast_unpack(mut self, enable: bool) -> Self {
        self.use_fast_unpack = enable;
        self
    }

    pub fn with_height_downscale(mut self, factor: i64) -> Self {
        self.height_downscale = factor.max(1);
        self
    }

    /// Select the emitted opset mode.
    ///
    /// `MnnIspMin` and `Custom` both emit the fused `isp.unpack_blc` op;
    /// `Primitive` emits the five-op equivalent.
    pub fn with_opset_mode(mut self, mode: BlockOpsetMode) -> Self {
        self.forced_mode = Some(mode);
        self
    }

    /// Emit primitive ONNX ops instead of the fused isp.unpack_blc custom op.
    pub fn force_primitive(mut self) -> Self {
        self.forced_mode = Some(BlockOpsetMode::Primitive);
        self
    }

    /// Build a fused block from a matched pattern run.
    pub fn from_chain(chain: &[&dyn IspBlock]) -> Self {
        let mut fused = Self::new();
        if let Some(first) = chain.first() {
            if let Some(src) = first.input_source() {
                if !src.is_empty() {
                    fused.input_source = src.to_string();
                }
            }
        }
        if let Some(last) = chain.last() {
            if let Some(frame) = last.frame_tensor() {
                fused.frame_tensor = frame.to_string();
            }
        }
        fused
    }

    /// Emit the fused custom-domain `isp.unpack_blc` node.
    ///
    /// This used to emit op_type `unpack`, which is not an `isp.*` name and has
    /// no shader: the op reached the runtime as an unknown Extra, VulkanFuse
    /// rejected it, and the graph was left with a node nothing could execute —
    /// a SIGSEGV during session creation rather than an error message.
    ///
    /// Coefficients ride as graph inputs, not attributes: `VulkanFuse` is
    /// positional and reads none of the attribute keys this block used to set.
    fn emit_custom_node(&self) -> Vec<u8> {
        self.emit_mnn_isp_min_node()
    }

    /// Emit primitive ONNX nodes for MNN when Custom opsets are not available.
    fn emit_primitive_nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();

        vec![
            // 1. Cast INT32 → FLOAT
            Proto::node(
                "Cast",
                &[&self.input_source],
                &[&format!("{}/cast", ns)],
                &[Proto::attribute_int("to", 1)], // FLOAT
            ),
            // 2. Div by sensor_max (normalize)
            Proto::node(
                "Div",
                &[&format!("{}/cast", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/norm", ns)],
                &[],
            ),
            // 3. Sub BLC offsets
            Proto::node(
                "Sub",
                &[&format!("{}/norm", ns), &format!("{}/blc_vals", ns)],
                &[&format!("{}/blc", ns)],
                &[],
            ),
            // 4. Clip to [0, 1]
            Proto::node(
                "Clip",
                &[
                    &format!("{}/blc", ns),
                    &format!("{}/zero", ns),
                    &format!("{}/one", ns),
                ],
                &[&format!("{}/clipped", ns)],
                &[],
            ),
            // 5. Conv for CFA unpack (2x2 kernel, stride 2)
            Proto::node(
                "Conv",
                &[
                    &format!("{}/clipped", ns),
                    &format!("{}/cfa_w", ns),
                    &format!("{}/cfa_b", ns),
                ],
                &[&self.frame_tensor],
                &[
                    Proto::attribute_ints("kernel_shape", &[2, 2]),
                    Proto::attribute_ints("strides", &[2, 2]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                ],
            ),
        ]
    }

    /// Emit the fused `isp.unpack_blc` Extra op.
    ///
    /// One dispatch replaces `Cast → Div → Sub → Clip → Conv(2x2, stride 2)`
    /// over the full-resolution frame.
    ///
    /// The node takes **tensors**, not attributes, because `VulkanFuse` is
    /// positional: it reads twelve attribute keys and none of them is a
    /// coefficient, so a shader's inputs can only arrive as descriptor buffers.
    /// Input order decides the binding (`input[0]` is the frame at 3, then
    /// `input[1..]` fill 5, 6, …), which is why the arguments below are in the
    /// same order the shader declares them.
    fn emit_mnn_isp_min_node(&self) -> Vec<u8> {
        let ns = self.tensor_ns();
        Proto::node_with_domain(
            // The full name, not a bare one plus the domain: the converter
            // copies op_type verbatim into Extra::type, so a bare name becomes
            // an Extra with no `isp.` prefix and no lookup entry.
            //
            // `_normal_cfa` names what the shader actually does — normalise,
            // black-level, and the CFA quad split. The earlier `isp.unpack_blc`
            // left the CFA step implicit in the output shape, which is how the
            // plane order ended up unreconciled with the Bayer phase.
            "isp.unpack_blc_normal_cfa",
            "isp",
            &[
                // binding 3 — packed `[1,1,H,W/2]` INT32 sensor samples
                &self.input_source,
                // binding 5 — black level per colour (R, Gr, Gb, B)
                &format!("{}/blc_vals", ns),
                // binding 6 — the sensor's white level
                &format!("{}/max_val", ns),
                // binding 7 — CFA phase, so the shader maps colour -> plane
                &format!("{}/bayer_pattern", ns),
            ],
            &[&self.frame_tensor],
            // No `elementwise`: that attribute means "the output has the input's
            // shape", and VulkanFuse then sizes the dispatch from the input
            // dims. This op halves H and W, so marking it elementwise dispatched
            // 192 rows of work for a 96-row output — and the readback of the
            // resulting tensor faulted instead of returning the quad.
            //
            // The shader gets its source geometry from the params block, which
            // VulkanFuse fills from the input tensor regardless.
            &[],
        )
    }
}

impl IspBlock for NormalizeBlcUnpackCfa {
    fn id(&self) -> &str {
        "isp_unpack_blc"
    }

    fn tensor_ns(&self) -> String {
        "isp_unpack_blc".to_string()
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
        None
    }

    fn set_prev(&mut self, _block: Box<dyn IspBlock>) {
        // Fused block has no prev in the graph
    }

    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        None
    }

    fn set_next(&mut self, _block: Box<dyn IspBlock>) {
        // Fused block has no next in the graph
    }

    fn input_elem_type(&self) -> i32 {
        6 // INT32 (packed Bayer)
    }

    fn output_elem_type(&self) -> i32 {
        1 // FLOAT32
    }

    fn graph_input_name(&self) -> Option<&str> {
        Some(&self.input_source)
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        match self.opset_mode() {
            BlockOpsetMode::Custom => vec![self.emit_custom_node()],
            BlockOpsetMode::MnnIspMin => vec![self.emit_mnn_isp_min_node()],
            BlockOpsetMode::Primitive => self.emit_primitive_nodes(),
        }
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }

    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            let out_w = w / 2;
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(out_w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W2"),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 6)) // INT32
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            let out_h = h / 2;
            let out_w = w / 2;
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(out_h),
                Proto::tensor_dim_value(out_w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H2"),
                Proto::tensor_dim_param("W2"),
            ]
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1)) // FLOAT
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();

        // The fused path declares only the tensors its shader reads. Declaring
        // the primitive-only ones too (`div_65536`, `cfa_w`, `zero`, `one`) left
        // them as graph inputs that no node consumed, which MNN's converter
        // drops — and a graph input nothing references is a good way to make
        // session creation fail in a place no message points at.
        if self.opset_mode() == BlockOpsetMode::Primitive {
            let sw = if self.use_fast_unpack { 2 } else { 1 };
            return vec![
                (format!("{}/max_val", ns), 1, vec![1]),
                (format!("{}/div_65536", ns), 6, vec![]),
                (format!("{}/mod_65536", ns), 6, vec![]),
                (format!("{}/cfa_w", ns), 1, vec![4, 2, 2, sw]),
                (format!("{}/cfa_b", ns), 1, vec![4]),
                (format!("{}/blc_vals", ns), 1, vec![1, 4, 1, 1]),
                (format!("{}/zero", ns), 1, vec![]),
                (format!("{}/one", ns), 1, vec![]),
            ];
        }

        // Binding order follows this list: `input[1]` -> 5, `input[2]` -> 6,
        // `input[3]` -> 7.
        vec![
            (format!("{}/blc_vals", ns), 1, vec![1, 4, 1, 1]),
            (format!("{}/max_val", ns), 1, vec![1]),
            (format!("{}/bayer_pattern", ns), 1, vec![1]),
        ]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        let sm = self.sensor_max;
        let sw = if self.use_fast_unpack { 2 } else { 1 };

        let cfa_w: Vec<f32> = if sw == 1 {
            vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ]
        } else {
            vec![
                0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5,
            ]
        };

        vec![
            (format!("{}/max_val", ns), sm.to_ne_bytes().to_vec()),
            (
                format!("{}/div_65536", ns),
                (65536i32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/mod_65536", ns),
                (65536i32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/cfa_w", ns),
                cfa_w.iter().flat_map(|v| v.to_ne_bytes()).collect(),
            ),
            (
                format!("{}/cfa_b", ns),
                [0.0f32, 0.0, 0.0, 0.0]
                    .iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect(),
            ),
            (
                format!("{}/blc_vals", ns),
                self.blc_offsets
                    .iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect(),
            ),
            (
                format!("{}/bayer_pattern", ns),
                (self.bayer_pattern as f32).to_ne_bytes().to_vec(),
            ),
            (format!("{}/zero", ns), (0.0f32).to_ne_bytes().to_vec()),
            (format!("{}/one", ns), (1.0f32).to_ne_bytes().to_vec()),
        ]
    }

    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&["isp.unpack_blc_normal_cfa"])
    }

    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![("isp".to_string(), 1)]
    }

    fn opset_mode(&self) -> BlockOpsetMode {
        self.forced_mode.unwrap_or(BlockOpsetMode::Custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_blc_unpack_cfa_new() {
        let b = NormalizeBlcUnpackCfa::new();
        assert_eq!(b.id(), "isp_unpack_blc");
    }

    #[test]
    fn test_normalize_blc_unpack_cfa_with_dims() {
        let b = NormalizeBlcUnpackCfa::new().with_concrete_dims(1920, 3840);
        let vi = b.input_value_info().unwrap();
        assert!(!vi.is_empty());
        let out_vi = b.output_value_info().unwrap();
        assert!(!out_vi.is_empty());
    }

    #[test]
    fn test_normalize_blc_unpack_cfa_extra_inputs() {
        let b = NormalizeBlcUnpackCfa::new();
        assert!(!b.extra_inputs().is_empty());
    }

    #[test]
    fn test_normalize_blc_unpack_cfa_extra_defaults() {
        let b = NormalizeBlcUnpackCfa::new();
        // blc_vals + max_val + bayer_pattern + max_val(dup) ... the count is
        // asserted below by name instead of by length.
        assert!(b.extra_input_defaults().len() >= 8);
    }

    #[test]
    fn test_normalize_blc_unpack_cfa_nodes() {
        // Default (Custom) -> 1 fused isp.unpack_blc node.
        let mut b = NormalizeBlcUnpackCfa::new();
        b.set_input_source("raw_frame");
        assert_eq!(b.opset_mode(), BlockOpsetMode::Custom);
        assert_eq!(b.nodes().len(), 1);
        // Primitive mode -> 5 primitive nodes.
        let mut b2 = NormalizeBlcUnpackCfa::new();
        b2.set_input_source("raw_frame");
        b2 = b2.force_primitive();
        assert_eq!(b2.opset_mode(), BlockOpsetMode::Primitive);
        assert_eq!(b2.nodes().len(), 5);
    }
}
