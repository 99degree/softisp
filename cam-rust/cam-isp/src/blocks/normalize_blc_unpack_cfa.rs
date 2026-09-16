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
    /// BLC offsets [R, G, B, G] - default zeros
    pub blc_offsets: [f32; 4],
    /// Whether to use fast Conv-based unpack vs SpaceToDepth
    pub use_fast_unpack: bool,
    /// Height downscale factor (1=none, 2=half)
    pub height_downscale: i64,
}

impl Default for NormalizeBlcUnpackCfa {
    fn default() -> Self {
        Self::new()
    }
}

impl NormalizeBlcUnpackCfa {
    pub fn new() -> Self {
        Self {
            input_source: String::new(),
            frame_tensor: "NormalizeBlcUnpackCfa/frame".to_string(),
            concrete_h: None,
            concrete_w: None,
            sensor_max: 65535.0,
            blc_offsets: [0.0; 4],
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

    pub fn with_blc_offsets(mut self, offsets: [f32; 4]) -> Self {
        self.blc_offsets = offsets;
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
        self.emit_primitive_nodes()
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
        let sw = if self.use_fast_unpack { 2 } else { 1 };
        vec![
            (format!("{}/max_val", ns), 1, vec![1]),
            (format!("{}/div_65536", ns), 6, vec![]),
            (format!("{}/mod_65536", ns), 6, vec![]),
            (format!("{}/cfa_w", ns), 1, vec![4, 2, 2, sw]),
            (format!("{}/cfa_b", ns), 1, vec![4]),
            (format!("{}/blc_vals", ns), 1, vec![1, 4, 1, 1]),
            (format!("{}/zero", ns), 1, vec![]),
            (format!("{}/one", ns), 1, vec![]),
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
            (format!("{}/zero", ns), (0.0f32).to_ne_bytes().to_vec()),
            (format!("{}/one", ns), (1.0f32).to_ne_bytes().to_vec()),
        ]
    }

    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&["isp.unpack_blc"])
    }

    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![("isp".to_string(), 1)]
    }

    fn opset_mode(&self) -> BlockOpsetMode {
        BlockOpsetMode::Custom
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
        assert_eq!(b.extra_input_defaults().len(), 8);
    }

    #[test]
    fn test_normalize_blc_unpack_cfa_nodes() {
        let mut b = NormalizeBlcUnpackCfa::new();
        b.set_input_source("raw_frame");
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 5);
    }
}
