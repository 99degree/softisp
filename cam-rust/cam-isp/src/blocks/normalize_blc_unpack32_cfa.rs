//! `isp.unpack32_blc_normal_cfa` — normalize + BLC + quad unpack from packed INT32 RAW.
//!
//! One dispatch replaces `RawInput16Block → UnpackCfaBlock → NormalizeBlock → BlcBlock`
//! (four blocks) with a single GPU pass. A RAW16 sensor plane is W*H contiguous
//! u16 samples, which on a little-endian host is exactly W/2 int32 words per row,
//! so the graph input is `[1,1,H,W/2]` INT32 and the sensor buffer is handed over
//! verbatim — no host-side split, no widening pass. The shader splits each word
//! into its two int16 lanes, converts to float, divides by sensor_max, subtracts
//! BLC, and extracts the Bayer CFA quad — all in one GPU pass.
//!
//! Bindings match the shader in `~/MNN/source/backend/vulkan/buffer/execution/isp_spv/`
//! and VulkanFuse's universal `isp.*` descriptor map (frame in@3, frame out@4,
//! extras from 5):
//!   binding 0 = Uniforms {width, height, ...}
//!   binding 3 = input INT32 packed plane (readonly)  ← InBuf
//!   binding 4 = output float quad planes              ← OutBuf
//!   binding 5 = black level per colour (R, Gr, Gb, B)
//!   binding 6 = sensor white level
//!   binding 7 = CFA phase

use crate::onnx::proto::Proto;
use crate::pipeline::{BlockOpsetMode, IspBlock};

#[derive(Debug)]
pub struct NormalizeBlcUnpack32Cfa {
    /// Input tensor name (e.g., "raw_frame")
    pub input_source: String,
    /// Output tensor name (e.g., "NormalizeBlcUnpack32Cfa/frame")
    pub frame_tensor: String,
    /// Optional concrete dimensions for shape inference
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
    /// Sensor max value for normalization (default 65535)
    pub sensor_max: f32,
    /// Black level per *colour*, ordered R, Gr, Gb, B (default zeros).
    pub blc_offsets: [f32; 4],
    /// CFA phase: 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR.
    pub bayer_pattern: i32,
    /// Override the emitted opset mode.
    pub forced_mode: Option<BlockOpsetMode>,
}

impl Default for NormalizeBlcUnpack32Cfa {
    fn default() -> Self {
        Self::new()
    }
}

impl NormalizeBlcUnpack32Cfa {
    pub fn new() -> Self {
        Self {
            input_source: "raw_frame".to_string(),
            frame_tensor: "NormalizeBlcUnpack32Cfa/frame".to_string(),
            concrete_h: None,
            concrete_w: None,
            sensor_max: 65535.0,
            blc_offsets: [0.0; 4],
            bayer_pattern: 0,
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

    pub fn with_bayer_pattern(mut self, pattern: i32) -> Self {
        self.bayer_pattern = pattern;
        self
    }

    pub fn with_opset_mode(mut self, mode: BlockOpsetMode) -> Self {
        self.forced_mode = Some(mode);
        self
    }

    /// Emit the fused `isp.unpack32_blc_normal_cfa` Extra op.
    ///
    /// One dispatch replaces `UnpackCfa → Normalize → Blc` over the
    /// full-resolution FLOAT32 frame. The node takes **tensors**, not
    /// attributes, because `VulkanFuse` is positional.
    ///
    /// Input order decides the binding (`input[0]` is the frame, then the
    /// extras follow in order at the slots after the output), matching the
    /// shader's declarations: InBuf@3, OutBuf@4, BlcBuf@5, MaxBuf@6,
    /// BayerBuf@7.
    fn emit_mnn_isp_min_node(&self) -> Vec<u8> {
        let ns = self.tensor_ns();
        // The full name, not a bare one plus the domain: the converter copies
        // op_type verbatim into Extra::type, so a bare name becomes an Extra
        // with no `isp.` prefix and no lookup entry.
        Proto::node_with_domain(
            "isp.unpack32_blc_normal_cfa",
            "isp",
            &[
                // The frame — bound at InBuf@3 by VulkanFuse's universal map.
                &self.input_source,
                // binding 5 — black level per colour (R, Gr, Gb, B)
                &format!("{}/blc_vals", ns),
                // binding 6 — the sensor's white level
                &format!("{}/max_val", ns),
                // binding 7 — CFA phase, so the shader maps colour -> plane
                &format!("{}/bayer_pattern", ns),
            ],
            &[&self.frame_tensor],
            &[],
        )
    }

    fn emit_primitive_nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Primitive fallback: Div by sensor_max, Sub BLC, CFA conv.
        vec![
            Proto::node(
                "Div",
                &[&self.input_source, &format!("{}/max_val", ns)],
                &[&format!("{}/norm", ns)],
                &[],
            ),
            Proto::node(
                "Sub",
                &[&format!("{}/norm", ns), &format!("{}/blc_vals", ns)],
                &[&format!("{}/blc", ns)],
                &[],
            ),
            Proto::node(
                "Conv",
                &[
                    &format!("{}/blc", ns),
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

impl IspBlock for NormalizeBlcUnpack32Cfa {
    fn id(&self) -> &str {
        "isp_unpack32_blc"
    }

    fn tensor_ns(&self) -> String {
        "isp_unpack32_blc".to_string()
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
        6 // INT32 (onnx.TensorProto_DataType_INT32) — packed u16 pairs
    }

    fn output_elem_type(&self) -> i32 {
        1 // FLOAT32
    }

    fn graph_input_name(&self) -> Option<&str> {
        Some(&self.input_source)
    }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }

    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        // `[1,1,H,W/2]` INT32. A RAW16 sensor plane is W*H contiguous u16
        // samples, which on a little-endian host is exactly W/2 int32 words per
        // row, so the sensor buffer is the graph input verbatim: no split, no
        // widen, no per-frame CPU pass. The shader splits each word into its two
        // int16 lanes and converts to float on the GPU.
        let dims = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w / 2),
            ],
            (None, Some(w)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_value(w / 2),
            ],
            _ => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W/2"),
            ],
        };
        // elem_type 6 = onnx.TensorProto_DataType_INT32. This field is the ONNX
        // TensorProto enum the converter reads, not MNN's `data_type()` code.
        Some(Proto::value_info(&self.input_source, &dims, 6))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let h = self.concrete_h.unwrap_or(0) / 2;
        let w = self.concrete_w.unwrap_or(0) / 2;
        let dims = if h > 0 && w > 0 {
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
                Proto::tensor_dim_param("H/2"),
                Proto::tensor_dim_param("W/2"),
            ]
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        // `emit_mnn_isp_min_node` hands the shader three coefficient tensors, so
        // they have to exist on the graph boundary or MNN has nothing to bind.
        // Without this, `extra_input_defaults` warns "no tensor in the session",
        // the shader reads unbound memory, and the quad comes back un-normalised
        // (BLC and sensor_max never applied).
        vec![
            (format!("{}/blc_vals", ns), 1, vec![1, 4, 1, 1]),
            (format!("{}/max_val", ns), 1, vec![1]),
            (format!("{}/bayer_pattern", ns), 1, vec![1]),
        ]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let blc_data: Vec<u8> = self
            .blc_offsets
            .iter()
            .flat_map(|&v| v.to_le_bytes())
            .collect();
        let max_data: Vec<u8> = self.sensor_max.to_le_bytes().to_vec();
        let pattern_data: Vec<u8> = self.bayer_pattern.to_le_bytes().to_vec();

        vec![
            (format!("{}/blc_vals", self.tensor_ns()), blc_data),
            (format!("{}/max_val", self.tensor_ns()), max_data),
            (format!("{}/bayer_pattern", self.tensor_ns()), pattern_data),
        ]
    }

    fn opset_mode(&self) -> BlockOpsetMode {
        self.forced_mode.unwrap_or(BlockOpsetMode::Custom)
    }

    /// Declares this block as fusion-capable, which is what makes
    /// `MnnEngine::chain_has_fused_op` force fp32 tensor storage for the session.
    ///
    /// The shader declares `float[]` storage buffers; MNN's Vulkan backend
    /// allocates two bytes per element when fp16 is on, so without fp32 the
    /// op reads halves as floats.
    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&["isp.unpack32_blc_normal_cfa"])
    }

    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![("isp".to_string(), 1)]
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        match self.opset_mode() {
            BlockOpsetMode::Custom => vec![self.emit_mnn_isp_min_node()],
            BlockOpsetMode::MnnIspMin => vec![self.emit_mnn_isp_min_node()],
            BlockOpsetMode::Primitive => self.emit_primitive_nodes(),
        }
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    /// The fused head reads the sensor buffer itself: `isp.unpack32_blc_normal_cfa`
    /// takes `[1,1,H,W]` FLOAT32 straight off the graph boundary. A
    /// `RawInput16Block` in front of it would be a second declaration of the
    /// boundary this block already owns.
    fn is_graph_input_placeholder_for(&self) -> bool {
        self.opset_mode() == BlockOpsetMode::MnnIspMin
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unpack32_blc_id() {
        let b = NormalizeBlcUnpack32Cfa::new();
        assert_eq!(b.id(), "isp_unpack32_blc");
    }

    #[test]
    fn test_unpack32_blc_input_elem_type() {
        let b = NormalizeBlcUnpack32Cfa::new();
        assert_eq!(b.input_elem_type(), 6); // INT32 (ONNX TensorProto 6)
    }

    #[test]
    fn test_unpack32_blc_output_elem_type() {
        let b = NormalizeBlcUnpack32Cfa::new();
        assert_eq!(b.output_elem_type(), 1); // FLOAT32
    }

    #[test]
    fn test_unpack32_blc_concrete_dims() {
        let b = NormalizeBlcUnpack32Cfa::new().with_concrete_dims(2160, 3840);
        assert_eq!(b.concrete_h, Some(2160));
        assert_eq!(b.concrete_w, Some(3840));
    }

    #[test]
    fn test_unpack32_blc_output_dims() {
        let b = NormalizeBlcUnpack32Cfa::new().with_concrete_dims(2160, 3840);
        let info = b.output_value_info().unwrap();
        assert!(!info.is_empty());
    }

    #[test]
    fn test_unpack32_blc_extra_inputs() {
        let b = NormalizeBlcUnpack32Cfa::new()
            .with_blc_offsets([1.0, 2.0, 3.0, 4.0])
            .with_sensor_max(1023.0)
            .with_bayer_pattern(2);
        let extras = b.extra_input_defaults();
        assert_eq!(extras.len(), 3);
        let names: Vec<&str> = extras.iter().map(|(n, _)| n.as_str()).collect();
        assert!(names.iter().any(|n| n.ends_with("/blc_vals")));
        assert!(names.iter().any(|n| n.ends_with("/max_val")));
        assert!(names.iter().any(|n| n.ends_with("/bayer_pattern")));
    }
}
