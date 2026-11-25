use crate::onnx::proto::Proto;
use crate::pipeline::{BlockOpsetMode, IspBlock};

/// ToneBlock — exposure × contrast + brightness, clip, then gamma encode.
///
/// Simple per-pixel tone curve: `pow(clip(x * exposure * contrast + brightness),
/// 1/gamma)`. All parameters are hot-swappable at runtime via tensor inputs.
///
/// Gamma lives here (not in `DisplayBlock`) so the display format conversion
/// never applies the curve twice — see
/// `docs/isp/20261126-isp-display-absorption-audit.md`.
///
/// In `MnnIspMin` / `Custom` mode the primitives collapse to one `isp.tone`
/// fused Vulkan op with the same per-frame coefficient inputs.
#[derive(Debug)]
pub struct ToneBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Contrast factor (1.0 = neutral)
    contrast: f32,
    /// Brightness offset (-1.0 to 1.0)
    brightness: f32,
    /// Gamma (1.0 = linear, 2.2 = sRGB-like)
    gamma: f32,
    /// Fused/primitive switch, set by `profile_builder` from `FuseStages`.
    forced_mode: Option<BlockOpsetMode>,
}
impl Default for ToneBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl ToneBlock {
    pub fn new() -> Self {
        Self {
            id: "tone".into(),
            prev: None,
            next: None,
            frame_tensor: "ToneBlock/frame".into(),
            input_source: String::new(),
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

    /// Build with a display gamma. `1.0` leaves the tone curve linear.
    pub fn with_gamma(mut self, gamma: f32) -> Self {
        self.gamma = gamma;
        self
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
impl IspBlock for ToneBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "ToneBlock".to_string()
    }

    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&["isp.tone"])
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
        match self.opset_mode() {
            BlockOpsetMode::MnnIspMin | BlockOpsetMode::Custom => {
                // One fused Vulkan op replaces the primitive nodes. The SPIR-V
                // lives in libMNN_Vulkan.so's embedded lookup; softisp emits
                // only the op and its tensors. `inv_gamma` is derived CPU-side.
                vec![Proto::node_with_domain(
                    "isp.tone",
                    "isp",
                    &[
                        &self.input_source,
                        &format!("{}/exposure", ns),
                        &format!("{}/contrast", ns),
                        &format!("{}/brightness", ns),
                        &format!("{}/inv_gamma", ns),
                    ],
                    &[&self.frame_tensor],
                    &[
                        // Elementwise so VulkanFuse hot-swaps W/H into binding 0
                        // and derives the dispatch from the frame tensor.
                        Proto::attribute_int("elementwise", 1),
                    ],
                )]
            }
            BlockOpsetMode::Primitive => vec![
                // Exposure first: gain scales the signal, then contrast shapes it.
                Proto::node(
                    "Mul",
                    &[&self.input_source, &format!("{}/exposure", ns)],
                    &[&format!("{}/exposed", ns)],
                    &[],
                ),
                Proto::node(
                    "Mul",
                    &[&format!("{}/exposed", ns), &format!("{}/contrast", ns)],
                    &[&format!("{}/contrasted", ns)],
                    &[],
                ),
                Proto::node(
                    "Add",
                    &[&format!("{}/contrasted", ns), &format!("{}/brightness", ns)],
                    &[&format!("{}/brightened", ns)],
                    &[],
                ),
                Proto::node(
                    "Clip",
                    &[
                        &format!("{}/brightened", ns),
                        &format!("{}/zero", ns),
                        &format!("{}/one", ns),
                    ],
                    &[&format!("{}/clipped", ns)],
                    &[],
                ),
                Proto::node(
                    "Pow",
                    &[&format!("{}/clipped", ns), &format!("{}/inv_gamma", ns)],
                    &[&self.frame_tensor],
                    &[],
                ),
            ],
        }
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![
            // `contrast` and `brightness` are node inputs (Mul/Add), so they
            // must be declared as graph inputs — an undeclared input is
            // dropped by MNN's ONNX importer ("has empty input") and the
            // optimizer then trips over the half-connected op.
            (format!("{}/contrast", ns).to_string(), 1, vec![1]),
            (format!("{}/brightness", ns).to_string(), 1, vec![1]),
            // Exposure gain from the AE loop. It multiplies the frame before
            // the contrast/brightness stage, so a bright or dark scene is
            // corrected at the right point in the tone curve instead of being
            // folded into an additive brightness offset.
            (format!("{}/exposure", ns).to_string(), 1, vec![1]),
            (format!("{}/zero", ns).to_string(), 1, vec![]),
            (format!("{}/one", ns).to_string(), 1, vec![]),
            // Display gamma exponent (1/γ), derived on the CPU; both the
            // primitive Pow and the fused shader consume it.
            (format!("{}/inv_gamma", ns).to_string(), 1, vec![1]),
        ]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        vec![
            (
                format!("{}/contrast", ns).to_string(),
                self.contrast.to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/brightness", ns).to_string(),
                self.brightness.to_ne_bytes().to_vec(),
            ),
            // Neutral gain: the AE loop overwrites this once it has stats.
            (
                format!("{}/exposure", ns).to_string(),
                1.0f32.to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/zero", ns).to_string(),
                (0.0f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/one", ns).to_string(),
                (1.0f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/inv_gamma", ns).to_string(),
                if self.gamma > 0.0 {
                    1.0f32 / self.gamma
                } else {
                    1.0f32
                }
                .to_ne_bytes()
                .to_vec(),
            ),
        ]
    }

    /// Signals that ToneBlock can make use of FCS, LDCI, and EE aux blocks.
    fn signals_aux(&self) -> Vec<String> {
        vec!["fcs".into(), "ldci".into(), "ee".into()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tone_id() {
        assert_eq!(ToneBlock::new().id(), "tone");
    }

    #[test]
    fn test_tone_emit_onnx() {
        let mut b = ToneBlock::new();
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // exposure Mul + contrast Mul + brightness Add + Clip + Pow = 5 nodes
        assert_eq!(nodes.len(), 5);
    }

    #[test]
    fn test_tone_signals_aux() {
        let b = ToneBlock::new();
        assert_eq!(b.signals_aux(), vec!["fcs", "ldci", "ee"]);
    }

    #[test]
    fn test_tone_initializers() {
        let b = ToneBlock::new();
        let inits = b.extra_input_defaults();
        // contrast + brightness + exposure + zero/one + inv_gamma
        assert_eq!(inits.len(), 6);
        let names: Vec<&str> = inits.iter().map(|(n, _)| n.as_str()).collect();
        assert!(names.contains(&"ToneBlock/contrast"));
        assert!(names.contains(&"ToneBlock/brightness"));
        assert!(names.contains(&"ToneBlock/exposure"));
        assert!(names.contains(&"ToneBlock/inv_gamma"));
    }

    #[test]
    fn test_tone_tensor_ns() {
        let b = ToneBlock::new();
        assert!(!b.tensor_ns().is_empty());
    }

    fn assert_contains(bytes: &[u8], needle: &[u8]) {
        assert!(
            bytes.windows(needle.len()).any(|w| w == needle),
            "missing {:?} in fused node",
            String::from_utf8_lossy(needle)
        );
    }

    #[test]
    fn test_tone_fused_emits_single_isp_tone_node() {
        let mut b = ToneBlock::new();
        b.set_input_source("in/frame");
        b.set_opset_mode(BlockOpsetMode::MnnIspMin);
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 1);
        assert_contains(&nodes[0], b"isp.tone");
        assert_contains(&nodes[0], b"isp"); // domain
        assert_contains(&nodes[0], b"ToneBlock/exposure");
        assert_contains(&nodes[0], b"ToneBlock/contrast");
        assert_contains(&nodes[0], b"ToneBlock/brightness");
        assert_contains(&nodes[0], b"ToneBlock/inv_gamma");
        assert_contains(&nodes[0], b"elementwise");
        assert!(!nodes[0].windows(b"spirv".len()).any(|w| w == b"spirv"));
        assert_eq!(b.custom_op_types(), Some(&["isp.tone"][..]));
    }

    #[test]
    fn test_tone_fused_custom_mode_also_emits() {
        let mut b = ToneBlock::new();
        b.set_opset_mode(BlockOpsetMode::Custom);
        assert_eq!(b.nodes().len(), 1);
    }

    #[test]
    fn test_tone_primitive_mode_unchanged() {
        let b = ToneBlock::new();
        assert_eq!(b.opset_mode(), BlockOpsetMode::Primitive);
        assert_eq!(b.nodes().len(), 5);
    }
}
