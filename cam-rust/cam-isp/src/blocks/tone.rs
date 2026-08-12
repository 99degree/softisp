use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// ToneBlock — Mul+Add+Clip tone mapping.
///
/// Simple per-pixel tone curve: `clip(x * mul + add)`. All parameters
/// are hot-swappable at runtime via `hot_swap_const_buffer()`.
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
        }
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
        vec![
            Proto::node(
                "Mul",
                &[&self.input_source, &format!("{}/contrast", ns)],
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
                &[&self.frame_tensor],
                &[],
            ),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![
            (format!("{}/zero", ns).to_string(), 1, vec![]),
            (format!("{}/one", ns).to_string(), 1, vec![]),
        ]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        vec![
            (
                format!("{}/zero", ns).to_string(),
                (0.0f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/one", ns).to_string(),
                (1.0f32).to_ne_bytes().to_vec(),
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
        // Mul + Add + Clip = 3 nodes
        assert_eq!(nodes.len(), 3);
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
        assert_eq!(inits.len(), 2);
    }

    #[test]
    fn test_tone_tensor_ns() {
        let b = ToneBlock::new();
        assert!(!b.tensor_ns().is_empty());
    }
}
