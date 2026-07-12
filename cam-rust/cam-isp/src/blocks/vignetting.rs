//! Vignetting Correction — correct lens brightness falloff.
//!
//! Compensates for natural vignetting in camera lenses.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Vignetting correction block.
pub struct VignettingBlock {
    /// Image width.
    pub width: u32,
    /// Image height.
    pub height: u32,
    /// Lens center X (normalized 0.0-1.0).
    pub center_x: f32,
    /// Lens center Y (normalized 0.0-1.0).
    pub center_y: f32,
    /// Vignetting strength (0.0 = no correction, 1.0 = full correction).
    pub strength: f32,
    /// Falloff exponent (higher = sharper falloff).
    pub falloff: f32,
    /// Input tensor name (set by wire_blocks).
    input_source: String,
}

impl VignettingBlock {
    /// Create with custom parameters.
    pub fn new(width: u32, height: u32, strength: f32) -> Self {
        Self {
            width,
            height,
            center_x: 0.5,
            center_y: 0.5,
            strength,
            falloff: 2.0,
            input_source: String::new(),
        }
    }

    /// Create with default parameters.
    pub fn new_default(width: u32, height: u32) -> Self {
        Self::new(width, height, 0.5)
    }

    /// Create with custom center.
    pub fn with_center(mut self, cx: f32, cy: f32) -> Self {
        self.center_x = cx;
        self.center_y = cy;
        self
    }

    /// Create with custom falloff.
    pub fn with_falloff(mut self, falloff: f32) -> Self {
        self.falloff = falloff;
        self
    }
}

impl IspBlock for VignettingBlock {
    fn id(&self) -> &str {
        "vignetting"
    }

    fn tensor_ns(&self) -> String {
        "VignettingBlock".to_string()
    }

    fn input_source(&self) -> Option<&str> {
        if self.input_source.is_empty() {
            Some("vignetting/input")
        } else {
            Some(&self.input_source)
        }
    }

    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.into();
    }

    fn frame_tensor(&self) -> Option<&str> {
        Some("vignetting/output")
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
        // Vignetting correction: Apply pre-computed gain map
        // The gain_map is stored as an initializer, so we just multiply
        let input = if self.input_source.is_empty() {
            "vignetting/input"
        } else {
            &self.input_source
        };

        let nodes = vec![
            // Apply vignetting gain (gain_map is an initializer)
            Proto::node(
                "Mul",
                &[input, "vignetting/gain_map"],
                &[self.frame_tensor().unwrap_or("vignetting/output")],
                &[],
            ),
        ];

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        // Pre-compute vignetting gain map
        let mut gain_map = Vec::with_capacity((self.width * self.height) as usize);

        let cx = self.center_x * self.width as f32;
        let cy = self.center_y * self.height as f32;
        let max_dist =
            ((self.width as f32 / 2.0).powi(2) + (self.height as f32 / 2.0).powi(2)).sqrt();

        for y in 0..self.height {
            for x in 0..self.width {
                let dx = x as f32 - cx;
                let dy = y as f32 - cy;
                let r = (dx * dx + dy * dy).sqrt() / max_dist;

                // Vignetting correction: boost edges
                let vignette = 1.0 - self.strength * r.powf(self.falloff);
                let gain = 1.0 / vignette.max(0.1); // Prevent division by zero

                gain_map.push(gain);
            }
        }

        vec![Proto::tensor_proto_float(
            "vignetting/gain_map",
            &[1, 1, self.height as i64, self.width as i64],
            &gain_map,
        )]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vignetting_creation() {
        let block = VignettingBlock::new(1920, 1080, 0.5);
        assert_eq!(block.id(), "vignetting");
        assert_eq!(block.width, 1920);
        assert_eq!(block.strength, 0.5);
    }

    #[test]
    fn test_vignetting_default() {
        let block = VignettingBlock::new_default(640, 480);
        assert_eq!(block.center_x, 0.5);
        assert_eq!(block.falloff, 2.0);
    }

    #[test]
    fn test_vignetting_builder() {
        let block = VignettingBlock::new(1920, 1080, 0.3)
            .with_center(0.4, 0.5)
            .with_falloff(2.5);
        assert_eq!(block.center_x, 0.4);
        assert_eq!(block.falloff, 2.5);
    }

    #[test]
    fn test_vignetting_gain_map() {
        let block = VignettingBlock::new(100, 100, 0.5);
        let inits = block.initializers();
        assert!(!inits.is_empty());
    }
}
