//! ToneLutBlock — 7-point tone curve LUT + parametric tone via per-pixel interpolation.
//!
//! Applies a piecewise-linear tone mapping curve defined by 7 control points
//! at fixed x-positions [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1], followed by parametric
//! contrast/brightness/gamma.
//!
//! The block can also apply parametric contrast/brightness/gamma, but in the
//! hybrid pipeline it runs directly after [`ToneBlock`] with identity defaults
//! for those three: the rule-based controller sets them there once, and the
//! neural controller supplies only the 7-point LUT.
//!
//! ONNX subgraph (per-pixel only):
//! 1. Piecewise-linear LUT from 7 control points at x = [0, 1/6, ..., 1].
//!    For segment i (0..5), `seg_out = clip(x - t_i, 0, seg_width) * s_i`;
//!    `lut_out` sums those segments onto `y_0`.
//! 2. Apply parametric: `lut_out * contrast + brightness`.
//! 3. Apply gamma: `result ^ (1/gamma)`.
//!
//! The slopes `s_i = (y_{i+1} - y_i) / seg_width` are scalar derivations, so
//! they are computed on the CPU by the controller's tensor-feed callback and
//! arrive as `s0..s5` inputs. No scalar/sub-reduction node lives in this graph.
//!
//! At x = k/6: LUT result = y_0 + sum_{j<k} s_j * seg_width = y_k  ✓
//!
//! The neural controller computes the 7 LUT values; in the hybrid pipeline this
//! block applies them per-pixel after the parametric tone stage.

use crate::onnx::proto::Proto;
use crate::pipeline::{BlockOpsetMode, IspBlock};

/// Segment width shared by the 7 control points at `x = i/6`.
pub const TONE_LUT_SEGMENT_WIDTH: f32 = 1.0 / 6.0;

/// Derive the 6 piecewise slopes from a 7-point tone curve on the CPU.
///
/// These are scalars, not per-pixel work, so they are fed to inference as
/// tensor inputs rather than recomputed as graph nodes.
pub fn tone_curve_slopes(lut: &[f32; 7]) -> [f32; 6] {
    std::array::from_fn(|i| (lut[i + 1] - lut[i]) / TONE_LUT_SEGMENT_WIDTH)
}

/// 7-point tone curve LUT block with parametric contrast/brightness/gamma.
#[derive(Debug)]
pub struct ToneLutBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// 7 tone curve control points at x = [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1].
    /// Default: identity LUT [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1].
    tone_curve_lut: [f32; 7],
    /// Contrast factor (1.0 = neutral)
    contrast: f32,
    /// Brightness offset (-1.0 to 1.0)
    brightness: f32,
    /// Gamma (1.0 = linear, 2.2 = sRGB-like)
    gamma: f32,
    /// Fused/primitive switch, set by `profile_builder` from `FuseStages`.
    forced_mode: Option<BlockOpsetMode>,
}

impl Default for ToneLutBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl ToneLutBlock {
    pub fn new() -> Self {
        Self {
            id: "tone_lut".into(),
            prev: None,
            next: None,
            frame_tensor: "ToneLutBlock/frame".into(),
            input_source: String::new(),
            tone_curve_lut: [
                0.0,
                1.0 / 6.0,
                2.0 / 6.0,
                3.0 / 6.0,
                4.0 / 6.0,
                5.0 / 6.0,
                1.0,
            ],
            contrast: 1.0,
            brightness: 0.0,
            gamma: 1.0,
            forced_mode: None,
        }
    }

    /// Build with an explicit opset mode (used by the fused profile).
    pub fn with_opset_mode(mut self, mode: BlockOpsetMode) -> Self {
        self.forced_mode = Some(mode);
        self
    }

    /// Set the 7-point tone curve LUT.
    pub fn set_tone_curve_lut(&mut self, lut: [f32; 7]) {
        self.tone_curve_lut = lut;
    }

    /// Get current tone curve LUT.
    pub fn tone_curve_lut(&self) -> [f32; 7] {
        self.tone_curve_lut
    }

    /// Set contrast (1.0 = neutral, >1.0 = more contrast).
    pub fn set_contrast(&mut self, contrast: f32) {
        self.contrast = contrast;
    }

    /// Set brightness offset (-1.0 to 1.0).
    pub fn set_brightness(&mut self, brightness: f32) {
        self.brightness = brightness;
    }

    /// Set gamma (1.0 = linear).
    pub fn set_gamma(&mut self, gamma: f32) {
        self.gamma = gamma;
    }

    /// Get current contrast.
    pub fn contrast(&self) -> f32 {
        self.contrast
    }

    /// Get current brightness.
    pub fn brightness(&self) -> f32 {
        self.brightness
    }

    /// Get current gamma.
    pub fn gamma(&self) -> f32 {
        self.gamma
    }
}

impl IspBlock for ToneLutBlock {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "ToneLutBlock".to_string()
    }

    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&["isp.tonelut"])
    }

    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![("isp".to_string(), 1)]
    }

    fn opset_mode(&self) -> BlockOpsetMode {
        self.forced_mode.unwrap_or(BlockOpsetMode::Primitive)
    }

    fn set_opset_mode(&mut self, mode: BlockOpsetMode) {
        self.forced_mode = Some(mode);
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
        Some(Proto::value_info(
            &self.input_source,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let x = &self.input_source;

        if matches!(
            self.opset_mode(),
            BlockOpsetMode::MnnIspMin | BlockOpsetMode::Custom
        ) {
            // One fused Vulkan op. Only the frame, the y0 seed, the six
            // CPU-derived slopes, and the three parametric coefficients are
            // graph inputs; segment starts and widths are baked into the MNN
            // Vulkan shader. No scalar derivation lives in the graph.
            let mut inputs: Vec<String> = vec![x.clone(), format!("{}/y0", ns)];
            for i in 0..6 {
                inputs.push(format!("{}/s{}", ns, i));
            }
            inputs.push(format!("{}/contrast", ns));
            inputs.push(format!("{}/brightness", ns));
            inputs.push(format!("{}/inv_gamma", ns));
            let input_refs: Vec<&str> = inputs.iter().map(|s| s.as_str()).collect();

            return vec![Proto::node_with_domain(
                "isp.tonelut",
                "isp",
                &input_refs,
                &[&self.frame_tensor],
                &[Proto::attribute_int("elementwise", 1)],
            )];
        }

        let mut nodes = Vec::new();

        // Accumulator starts at y_0. Every remaining node operates on the
        // frame tensor element-wise; the scalar slopes arrive as s0..s5.
        let mut acc = format!("{}/lut_y0", ns);
        nodes.push(Proto::node(
            "Identity",
            &[&format!("{}/y0", ns)],
            &[&acc],
            &[],
        ));

        // For each segment i: acc += clip(x - t_i, 0, seg_width) * s_i
        for i in 0..6 {
            let seg_in = format!("{}/seg{}_in", ns, i);
            let seg_clamped = format!("{}/seg{}_clamped", ns, i);
            let seg_scaled = format!("{}/seg{}_scaled", ns, i);
            let acc_out = format!("{}/lut_acc{}", ns, i + 1);

            // seg_in = x - t_i
            nodes.push(Proto::node(
                "Sub",
                &[x, &format!("{}/t{}", ns, i)],
                &[&seg_in],
                &[],
            ));

            // seg_clamped = clip(seg_in, 0, seg_width)
            nodes.push(Proto::node(
                "Clip",
                &[
                    &seg_in,
                    &format!("{}/zero", ns),
                    &format!("{}/seg_width", ns),
                ],
                &[&seg_clamped],
                &[],
            ));

            // seg_scaled = seg_clamped * s_i (s_i derived on the CPU)
            nodes.push(Proto::node(
                "Mul",
                &[&seg_clamped, &format!("{}/s{}", ns, i)],
                &[&seg_scaled],
                &[],
            ));

            // acc_out = acc + seg_scaled
            nodes.push(Proto::node("Add", &[&acc, &seg_scaled], &[&acc_out], &[]));
            acc = acc_out;
        }

        // Apply parametric contrast: acc * contrast
        let contrasted = format!("{}/lut_contrasted", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&acc, &format!("{}/contrast", ns)],
            &[&contrasted],
            &[],
        ));

        // Apply brightness: contrasted + brightness
        let brightened = format!("{}/lut_brightened", ns);
        nodes.push(Proto::node(
            "Add",
            &[&contrasted, &format!("{}/brightness", ns)],
            &[&brightened],
            &[],
        ));

        // Clip to [0, 1]
        let clipped = format!("{}/lut_clipped", ns);
        nodes.push(Proto::node(
            "Clip",
            &[&brightened, &format!("{}/zero", ns), &format!("{}/one", ns)],
            &[&clipped],
            &[],
        ));

        // Apply gamma: clipped ^ (1/gamma) -> pow(clipped, inv_gamma)
        let gamma_corrected = format!("{}/lut_gamma", ns);
        nodes.push(Proto::node(
            "Pow",
            &[&clipped, &format!("{}/inv_gamma", ns)],
            &[&gamma_corrected],
            &[],
        ));

        // Final: gamma_corrected -> output
        nodes.push(Proto::node(
            "Identity",
            &[&gamma_corrected],
            &[&self.frame_tensor],
            &[],
        ));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        let mut inputs = Vec::new();

        // Accumulator seed: y_0 (the only LUT point still needed in the graph).
        inputs.push((format!("{}/y0", ns), 1, vec![1]));

        // 6 CPU-derived segment slopes: s_i = (y_{i+1} - y_i) / seg_width.
        for i in 0..6 {
            inputs.push((format!("{}/s{}", ns, i), 1, vec![1]));
        }

        // 6 segment start positions: t_i = i/6 (for Sub x - t_i)
        for i in 0..6 {
            inputs.push((format!("{}/t{}", ns, i), 1, vec![1]));
        }

        // Shared segment width (1/6) for the per-pixel Clip upper bound.
        inputs.push((format!("{}/seg_width", ns), 1, vec![1]));

        // zero constant for Clip lower bound
        inputs.push((format!("{}/zero", ns), 1, vec![1]));

        // one constant for Clip upper bound
        inputs.push((format!("{}/one", ns), 1, vec![1]));

        // Contrast, brightness, inv_gamma (parametric tone)
        inputs.push((format!("{}/contrast", ns), 1, vec![1]));
        inputs.push((format!("{}/brightness", ns), 1, vec![1]));
        inputs.push((format!("{}/inv_gamma", ns), 1, vec![1]));

        inputs
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        // Accumulator seed y_0 and 6 CPU-derived slopes for the built-in LUT.
        inits.push((
            format!("{}/y0", ns),
            self.tone_curve_lut[0].to_ne_bytes().to_vec(),
        ));
        for (i, slope) in tone_curve_slopes(&self.tone_curve_lut).iter().enumerate() {
            inits.push((format!("{}/s{}", ns, i), slope.to_ne_bytes().to_vec()));
        }

        // 6 segment start positions: t_i = i/6
        for i in 0..6 {
            let ti: f32 = i as f32 / 6.0;
            inits.push((format!("{}/t{}", ns, i), ti.to_ne_bytes().to_vec()));
        }

        // Shared segment width (1/6)
        inits.push((
            format!("{}/seg_width", ns),
            TONE_LUT_SEGMENT_WIDTH.to_ne_bytes().to_vec(),
        ));

        // zero constant
        inits.push((format!("{}/zero", ns), 0.0f32.to_ne_bytes().to_vec()));

        // one constant
        inits.push((format!("{}/one", ns), 1.0f32.to_ne_bytes().to_vec()));

        // Contrast, brightness, inv_gamma defaults
        inits.push((
            format!("{}/contrast", ns),
            self.contrast.to_ne_bytes().to_vec(),
        ));
        inits.push((
            format!("{}/brightness", ns),
            self.brightness.to_ne_bytes().to_vec(),
        ));
        inits.push((
            format!("{}/inv_gamma", ns),
            (1.0 / self.gamma.max(1e-4)).to_ne_bytes().to_vec(),
        ));

        inits
    }

    /// Signals that ToneLutBlock can make use of aux blocks.
    fn signals_aux(&self) -> Vec<String> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tone_lut_id() {
        assert_eq!(ToneLutBlock::new().id(), "tone_lut");
    }

    #[test]
    fn test_tone_lut_default_lut() {
        let b = ToneLutBlock::new();
        let lut = b.tone_curve_lut();
        assert_eq!(lut[0], 0.0);
        assert_eq!(lut[6], 1.0);
        assert!((lut[1] - 1.0 / 6.0).abs() < 1e-6);
        assert!((lut[3] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_tone_lut_set_lut() {
        let mut b = ToneLutBlock::new();
        let new_lut = [0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0];
        b.set_tone_curve_lut(new_lut);
        assert_eq!(b.tone_curve_lut(), new_lut);
    }

    #[test]
    fn test_tone_curve_slopes() {
        let identity = [
            0.0,
            1.0 / 6.0,
            2.0 / 6.0,
            3.0 / 6.0,
            4.0 / 6.0,
            5.0 / 6.0,
            1.0,
        ];
        for slope in tone_curve_slopes(&identity) {
            assert!((slope - 1.0).abs() < 1e-5);
        }

        let curved = [0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0];
        let slopes = tone_curve_slopes(&curved);
        for (i, slope) in slopes.iter().enumerate() {
            let expected = (curved[i + 1] - curved[i]) / TONE_LUT_SEGMENT_WIDTH;
            assert!((slope - expected).abs() < 1e-6);
        }
    }

    #[test]
    fn test_tone_lut_emit_onnx() {
        let mut b = ToneLutBlock::new();
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // initial Identity (y0) = 1
        // 6 segments × (Sub + Clip + Mul + Add) = 24
        // contrast Mul + brightness Add + Clip + Pow + final Identity = 5
        assert_eq!(nodes.len(), 30, "Expected 30 nodes, got {}", nodes.len());
    }

    #[test]
    fn test_tone_lut_graph_has_no_scalar_derivation() {
        let nodes = ToneLutBlock::new().nodes();
        assert!(
            nodes.iter().all(|n| !n.windows(3).any(|w| w == b"Div")),
            "tone-curve slopes must be derived on the CPU, not as graph nodes"
        );
        assert!(
            nodes
                .iter()
                .all(|n| !n.windows(b"seg0_diff".len()).any(|w| w == b"seg0_diff")),
            "scalar LUT differences must not appear in the graph"
        );
    }

    #[test]
    fn test_tone_lut_extra_inputs_count() {
        let b = ToneLutBlock::new();
        let inputs = b.extra_inputs();
        // y0 + 6 slopes + 6 segment starts + 1 segment width
        // + zero + one + contrast + brightness + inv_gamma = 19
        assert_eq!(
            inputs.len(),
            19,
            "Expected 19 extra inputs, got {}",
            inputs.len()
        );
        let names: std::collections::HashSet<_> =
            inputs.iter().map(|(name, _, _)| name.as_str()).collect();
        assert_eq!(names.len(), inputs.len(), "duplicate ToneLutBlock input");
        assert!(names.contains("ToneLutBlock/y0"));
        for i in 0..6 {
            assert!(names.contains(format!("ToneLutBlock/s{}", i).as_str()));
        }
        assert!(!names.iter().any(|n| n.contains("/slope")));
        for i in 1..7 {
            assert!(!names.contains(format!("ToneLutBlock/y{}", i).as_str()));
        }
    }

    #[test]
    fn test_tone_lut_extra_input_defaults_count() {
        let b = ToneLutBlock::new();
        let inits = b.extra_input_defaults();
        assert_eq!(
            inits.len(),
            19,
            "Expected 19 initializers, got {}",
            inits.len()
        );
        let names: std::collections::HashSet<_> =
            inits.iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(
            names.len(),
            inits.len(),
            "duplicate ToneLutBlock initializer"
        );
    }

    #[test]
    fn test_tone_lut_tensor_ns() {
        let b = ToneLutBlock::new();
        assert_eq!(b.tensor_ns(), "ToneLutBlock");
    }

    #[test]
    fn test_tone_lut_graph_output() {
        let b = ToneLutBlock::new();
        assert!(b.graph_output_name().is_some());
    }

    fn assert_contains(bytes: &[u8], needle: &[u8]) {
        assert!(
            bytes.windows(needle.len()).any(|w| w == needle),
            "missing {:?} in fused node",
            String::from_utf8_lossy(needle)
        );
    }

    #[test]
    fn test_tone_lut_fused_emits_single_isp_tonelut_node() {
        let mut b = ToneLutBlock::new();
        b.set_input_source("in/frame");
        b.set_opset_mode(BlockOpsetMode::MnnIspMin);
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 1);
        assert_contains(&nodes[0], b"isp.tonelut");
        assert_contains(&nodes[0], b"isp"); // domain
        assert_contains(&nodes[0], b"ToneLutBlock/y0");
        for i in 0..6 {
            assert_contains(&nodes[0], format!("ToneLutBlock/s{}", i).as_bytes());
        }
        assert_contains(&nodes[0], b"ToneLutBlock/contrast");
        assert_contains(&nodes[0], b"ToneLutBlock/brightness");
        assert_contains(&nodes[0], b"ToneLutBlock/inv_gamma");
        assert!(!nodes[0].windows(b"spirv".len()).any(|w| w == b"spirv"));
        // Segment starts/width are baked into the shader, not graph inputs.
        assert!(!nodes[0]
            .windows(b"ToneLutBlock/t0".len())
            .any(|w| w == b"ToneLutBlock/t0"));
        assert_eq!(b.custom_op_types(), Some(&["isp.tonelut"][..]));
    }

    #[test]
    fn test_tone_lut_fused_has_no_scalar_derivation() {
        let mut b = ToneLutBlock::new();
        b.set_opset_mode(BlockOpsetMode::MnnIspMin);
        let nodes = b.nodes();
        assert!(
            nodes.iter().all(|n| !n.windows(3).any(|w| w == b"Div")),
            "fused tone-lut graph must stay parallel-only"
        );
    }

    #[test]
    fn test_tone_lut_primitive_mode_unchanged() {
        let b = ToneLutBlock::new();
        assert_eq!(b.opset_mode(), BlockOpsetMode::Primitive);
        assert_eq!(b.nodes().len(), 30);
    }
}
