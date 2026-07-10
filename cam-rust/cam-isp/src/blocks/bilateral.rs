//! Bilateral Filter — edge-preserving noise reduction.
//!
//! Smooths images while preserving edges, ideal for low-light denoising.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Bilateral filter block — edge-preserving smoothing.
pub struct BilateralBlock {
    /// Spatial sigma (controls smoothing radius).
    pub sigma_spatial: f32,
    /// Range sigma (controls edge preservation).
    pub sigma_range: f32,
    /// Kernel size (must be odd).
    pub kernel_size: u32,
    /// Input tensor name (set by wire_blocks).
    input_source: String,
}

impl BilateralBlock {
    /// Create with custom parameters.
    pub fn new(sigma_spatial: f32, sigma_range: f32, kernel_size: u32) -> Self {
        Self {
            sigma_spatial,
            sigma_range,
            kernel_size,
            input_source: String::new(),
        }
    }

    /// Create with default parameters.
    pub fn new_default() -> Self {
        Self::new(3.0, 0.1, 5)
    }

    /// Create for light denoising.
    pub fn new_light() -> Self {
        Self::new(2.0, 0.15, 3)
    }

    /// Create for heavy denoising.
    pub fn new_heavy() -> Self {
        Self::new(5.0, 0.08, 7)
    }
}

impl IspBlock for BilateralBlock {
    fn id(&self) -> &str {
        "bilateral"
    }

    fn tensor_ns(&self) -> String {
        "BilateralBlock".to_string()
    }

    fn input_source(&self) -> Option<&str> {
        if self.input_source.is_empty() { Some("bilateral/input") } else { Some(&self.input_source) }
    }

    fn set_input_source(&mut self, name: &str) { self.input_source = name.into(); }

    fn frame_tensor(&self) -> Option<&str> {
        Some("bilateral/output")
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
        // Bilateral filter: Approximate with weighted Gaussian + edge mask
        let input = if self.input_source.is_empty() { "bilateral/input" } else { &self.input_source };
        let nodes = vec![
            // Gaussian blur for spatial smoothing
            Proto::node(
                "GaussianBlur",
                &[input],
                &["bilateral/blurred"],
                &[
                    Proto::attribute_float("sigma", self.sigma_spatial),
                    Proto::attribute_int("kernel_size", self.kernel_size as i64),
                ],
            ),
            // Edge detection for range weighting
            Proto::node(
                "Sobel",
                &[input],
                &["bilateral/edges"],
                &[],
            ),
            // Weighted blend based on edge strength
            Proto::node(
                "Mul",
                &["bilateral/blurred", "bilateral/edges"],
                &["bilateral/output"],
                &[],
            ),
        ];

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bilateral_creation() {
        let block = BilateralBlock::new(3.0, 0.1, 5);
        assert_eq!(block.id(), "bilateral");
        assert_eq!(block.sigma_spatial, 3.0);
    }

    #[test]
    fn test_bilateral_default() {
        let block = BilateralBlock::new_default();
        assert!(block.sigma_spatial > 0.0);
        assert!(block.sigma_range > 0.0);
    }

    #[test]
    fn test_bilateral_nodes() {
        let block = BilateralBlock::new_default();
        let nodes = block.nodes();
        assert!(!nodes.is_empty());
    }

    #[test]
    fn test_bilateral_shapes() {
        let block = BilateralBlock::new_default();
        assert!(block.input_source().is_some());
        assert!(block.frame_tensor().is_some());
    }
}
