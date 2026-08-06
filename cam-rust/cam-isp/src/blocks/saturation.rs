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
    /// Input tensor name (set by wire_blocks).
    input_source: String,
}

impl SaturationBlock {
    /// Create with custom saturation.
    pub fn new(saturation: f32) -> Self {
        Self {
            saturation,
            vibrance: 0.0,
            input_source: String::new(),
        }
    }

    /// Create with saturation and vibrance.
    pub fn new_with_vibrance(saturation: f32, vibrance: f32) -> Self {
        Self {
            saturation,
            vibrance,
            input_source: String::new(),
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
        if self.input_source.is_empty() {
            Some("saturation/input")
        } else {
            Some(&self.input_source)
        }
    }

    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.into();
    }

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

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            self.input_source().unwrap_or("saturation/input"),
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
            self.frame_tensor().unwrap_or("saturation/output"),
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
        let input = if self.input_source.is_empty() {
            "saturation/input"
        } else {
            &self.input_source
        };
        vec![Proto::node(
            "Mul",
            &[input, "saturation/scale"],
            &["saturation/output"],
            &[],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let scale = vec![self.saturation; 3];
        // Shape [1,3,1,1] for MNN broadcast — [3] causes runSession code=3.
        vec![Proto::tensor_proto_float(
            "saturation/scale",
            &[1, 3, 1, 1],
            &scale,
        )]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![("saturation/scale".into(), 1, vec![1, 3, 1, 1])]
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
