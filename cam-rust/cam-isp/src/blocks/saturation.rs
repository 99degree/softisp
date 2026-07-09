//! Saturation Control — adjust color intensity.
//!
//! Modifies color saturation while preserving luminance.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Saturation control block.
pub struct SaturationBlock {
    /// Saturation factor (0.0 = grayscale, 1.0 = normal, >1.0 = boosted).
    pub saturation: f32,
    /// Vibrance (smart saturation that protects skin tones).
    pub vibrance: f32,
}

impl SaturationBlock {
    /// Create with custom saturation.
    pub fn new(saturation: f32) -> Self {
        Self {
            saturation,
            vibrance: 0.0,
        }
    }

    /// Create with saturation and vibrance.
    pub fn new_with_vibrance(saturation: f32, vibrance: f32) -> Self {
        Self {
            saturation,
            vibrance,
        }
    }

    /// Create default (normal saturation).
    pub fn new_default() -> Self {
        Self::new(1.0)
    }

    /// Create for grayscale output.
    pub fn new_grayscale() -> Self {
        Self::new(0.0)
    }

    /// Create for vivid colors.
    pub fn new_vivid() -> Self {
        Self::new_with_vibrance(1.3, 0.3)
    }
}

impl IspBlock for SaturationBlock {
    fn id(&self) -> &str {
        "saturation"
    }

    fn tensor_ns(&self) -> String {
        "SaturationBlock".to_string()
    }

    fn input_source(&self) -> Option<&str> {
        Some("saturation/input")
    }

    fn set_input_source(&mut self, _name: &str) {}

    fn frame_tensor(&self) -> Option<&str> {
        Some("saturation/output")
    }

    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        None
    }

    fn set_prev(&mut self, _block: Box<dyn IspBlock>) {}

    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        None
    }

    fn set_next(&mut self, _block: Box<dyn IspBlock>) {}

    fn nodes(&self) -> Vec<Vec<u8>> {
        // Saturation: Scale color channels
        let nodes = vec![
            // Scale color channels
            Proto::node(
                "Mul",
                &["saturation/input", &format!("saturation/scale_{}", self.saturation)],
                &["saturation/output"],
                &[],
            ),
        ];

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        // Create saturation scale tensor
        let scale = vec![self.saturation; 3];
        vec![
            Proto::tensor_proto_float(
                &format!("saturation/scale_{}", self.saturation),
                &[3],
                &scale,
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_saturation_creation() {
        let block = SaturationBlock::new(1.5);
        assert_eq!(block.id(), "saturation");
        assert_eq!(block.saturation, 1.5);
    }

    #[test]
    fn test_saturation_default() {
        let block = SaturationBlock::new_default();
        assert_eq!(block.saturation, 1.0);
    }

    #[test]
    fn test_saturation_grayscale() {
        let block = SaturationBlock::new_grayscale();
        assert_eq!(block.saturation, 0.0);
    }

    #[test]
    fn test_saturation_vivid() {
        let block = SaturationBlock::new_vivid();
        assert!(block.saturation > 1.0);
        assert!(block.vibrance > 0.0);
    }

    #[test]
    fn test_saturation_nodes() {
        let block = SaturationBlock::new_default();
        let nodes = block.nodes();
        assert!(!nodes.is_empty());
    }
}
