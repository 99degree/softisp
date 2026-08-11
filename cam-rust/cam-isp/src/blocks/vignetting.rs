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

    /// Compute the vignetting gain map for a given output size.
    ///
    /// Must stay value-identical to what the engine writes at runtime
    /// (see `MnnEngine` vignetting extra-input handling).
    pub fn gain_map(height: u32, width: u32, strength: f32, falloff: f32) -> Vec<f32> {
        let mut gain_map = Vec::with_capacity((height * width) as usize);
        let cx = 0.5 * width as f32;
        let cy = 0.5 * height as f32;
        let max_dist = ((width as f32 / 2.0).powi(2) + (height as f32 / 2.0).powi(2)).sqrt();

        for y in 0..height {
            for x in 0..width {
                let dx = x as f32 - cx;
                let dy = y as f32 - cy;
                let r = (dx * dx + dy * dy).sqrt() / max_dist;

                // Vignetting correction: boost edges
                let vignette = 1.0 - strength * r.powf(falloff);
                let gain = 1.0 / vignette.max(0.1); // Prevent division by zero
                gain_map.push(gain);
            }
        }
        gain_map
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

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            self.input_source().unwrap_or("vignetting/input"),
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
            self.frame_tensor().unwrap_or("vignetting/output"),
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
        // Vignetting correction: Apply gain map.
        // The gain_map is a runtime extra input (not an initializer): a baked
        // [1,1,H,W] constant would break MNN resizeSession whenever the
        // pipeline input size differs from the baked dims (e.g. 32×32 or
        // 4K into an FHD-baked model). The engine resizes + fills it.
        let input = if self.input_source.is_empty() {
            "vignetting/input"
        } else {
            &self.input_source
        };

        vec![Proto::node(
            "Mul",
            &[input, "vignetting/gain_map"],
            &[self.frame_tensor().unwrap_or("vignetting/output")],
            &[],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(
            "vignetting/gain_map".to_string(),
            1, // FLOAT
            vec![1, 1, self.height as i64, self.width as i64],
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
        // gain_map moved to a runtime extra input (resize-safe);
        // the generator helper must produce a full-size map.
        let gain = VignettingBlock::gain_map(100, 100, 0.5, 2.0);
        assert_eq!(gain.len(), 100 * 100);
        assert!(!gain.is_empty());
        let ex = block_extra_inputs();
        assert_eq!(ex.len(), 1);
        assert_eq!(ex[0].0, "vignetting/gain_map");
    }

    fn block_extra_inputs() -> Vec<(String, i64, Vec<i64>)> {
        let block = VignettingBlock::new(100, 100, 0.5);
        block.extra_inputs()
    }
}
