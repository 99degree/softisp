//! Vignetting Correction — correct lens brightness falloff.
//!
//! Compensates for natural vignetting in camera lenses.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Vignetting correction block.
#[derive(Debug)]
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
    /// Instance suffix, so the same physics can appear twice in one chain
    /// (lens shading on the Bayer quad, vignetting on the RGB tail) without
    /// their tensor names colliding.
    pub instance: String,
    /// This instance's output tensor name, kept as a field because
    /// `frame_tensor()` returns a borrow.
    output: String,
    /// Number of planes in the frame this block multiplies.
    ///
    /// Lens shading runs on the post-CFA quad (4 planes of the half-resolution
    /// Bayer grid); the RGB tail has 3. The gain map is `[1, channels, h, w]`
    /// so the `Mul` broadcasts one map across every plane.
    pub channels: u32,
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
            instance: String::new(),
            output: "vignetting/output".to_string(),
            channels: 1,
        }
    }

    /// Give this instance its own tensor namespace.
    pub fn with_instance(mut self, name: &str) -> Self {
        self.instance = name.to_string();
        self.output = if name.is_empty() {
            "vignetting/output".to_string()
        } else {
            format!("vignetting_{}/output", name)
        };
        self
    }

    /// Set how many planes the frame carries (`1` single plane, `4` CFA quad).
    pub fn with_channels(mut self, channels: u32) -> Self {
        self.channels = channels.max(1);
        self
    }

    /// Tensor name of this instance's gain-map graph input.
    pub fn gain_map_tensor(&self) -> String {
        if self.instance.is_empty() {
            "vignetting/gain_map".to_string()
        } else {
            format!("vignetting_{}/gain_map", self.instance)
        }
    }

    /// Tensor name for this instance's frame input.
    pub fn input_tensor(&self) -> String {
        if self.instance.is_empty() {
            "vignetting/input".to_string()
        } else {
            format!("vignetting_{}/input", self.instance)
        }
    }

    /// Tensor name for this instance's frame output.
    pub fn output_tensor(&self) -> String {
        if self.instance.is_empty() {
            "vignetting/output".to_string()
        } else {
            format!("vignetting_{}/output", self.instance)
        }
    }

    /// Gain for one normalised radius, shared by the map builder and tests.
    fn gain_at(radius: f32, strength: f32, falloff: f32) -> f32 {
        let vignette = 1.0 - strength * radius.powf(falloff);
        1.0 / vignette.max(0.1)
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
                gain_map.push(Self::gain_at(r, strength, falloff));
            }
        }
        gain_map
    }

    /// Gain map for a `channels`-plane frame: `[1, channels, h, w]`.
    ///
    /// The falloff is one radial function of position, so every plane shares
    /// it; the channel axis exists only so the `Mul` broadcasts plane by plane
    /// without needing a reshape.
    pub fn gain_map_channels(
        height: u32,
        width: u32,
        strength: f32,
        falloff: f32,
        channels: u32,
    ) -> Vec<f32> {
        let one = Self::gain_map(height, width, strength, falloff);
        if channels <= 1 {
            return one;
        }
        let mut all = Vec::with_capacity(one.len() * channels as usize);
        for _ in 0..channels {
            all.extend_from_slice(&one);
        }
        all
    }
}

impl IspBlock for VignettingBlock {
    fn id(&self) -> &str {
        "vignetting"
    }

    fn tensor_ns(&self) -> String {
        if self.instance.is_empty() {
            "VignettingBlock".to_string()
        } else {
            format!("VignettingBlock_{}", self.instance)
        }
    }

    fn input_source(&self) -> Option<&str> {
        if self.input_source.is_empty() {
            None
        } else {
            Some(&self.input_source)
        }
    }

    /// The frame input, resolved lazily so each instance gets its own name.
    fn graph_input_name(&self) -> Option<&str> {
        None
    }

    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.into();
    }

    fn frame_tensor(&self) -> Option<&str> {
        // Stored rather than formatted on demand: the trait hands back a
        // borrowed str, so the name has to outlive the call.
        Some(self.output.as_str())
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
            self.input_source()
                .unwrap_or(&self.input_tensor())
                .to_string()
                .as_str(),
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.channels as i64),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.output,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.channels as i64),
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
        let input_owned = self.input_tensor();
        let input = if self.input_source.is_empty() {
            input_owned.as_str()
        } else {
            &self.input_source
        };
        let gain_tensor = self.gain_map_tensor();

        vec![Proto::node(
            "Mul",
            &[input, &gain_tensor],
            &[self.frame_tensor().unwrap_or("vignetting/output")],
            &[],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(
            self.gain_map_tensor(),
            1, // FLOAT
            vec![
                1,
                self.channels as i64,
                self.height as i64,
                self.width as i64,
            ],
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
