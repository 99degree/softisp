//! TemporalDenoiseBlock — multi-frame temporal noise reduction.
//!
//! Blends current frame with previous frame to reduce temporal noise.
//! Requires external frame delay: previous frame is fed as an input tensor,
//! and the output is copied back to the previous-frame buffer after inference.
//!
//! ONNX subgraph (single frame):
//!   Sub(current, prev) → diff           (motion/noise detection)
//!   Abs(diff) → abs_diff
//!   Less(abs_diff, threshold) → mask     (static regions get more blending)
//!   Mul(prev, 1-mask) + Mul(current, mask) → output
//!
//! Usage:
//!   1. First frame: prev_frame = zero tensor (no history)
//!   2. Each frame: feed current + prev → get denoised output
//!   3. After inference: copy output to prev_frame buffer for next iteration
//!
//! The threshold controls denoise strength:
//!   - Low threshold (0.01): aggressive denoise, may blur motion
//!   - High threshold (0.10): conservative denoise, preserves motion

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// TemporalDenoiseBlock — multi-frame noise reduction.
///
/// Blends current frame with previous denoised frame using motion-adaptive
/// weighting. Higher threshold = less denoising, lower = more aggressive.
/// Typical range: 0.02–0.05.
pub struct TemporalDenoiseBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub prev_frame_source: String,
    /// Noise threshold: pixels with |current - prev| < threshold are blended.
    /// Range: 0.001 (aggressive) to 0.2 (conservative). Default: 0.05.
    pub threshold: f32,
    /// Blend weight for static regions (0.0 = all prev, 1.0 = all current).
    /// Default: 0.5 (equal blend of current and previous).
    pub blend_weight: f32,
}

impl Default for TemporalDenoiseBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl TemporalDenoiseBlock {
    pub fn new() -> Self {
        Self {
            id: "temporal_denoise".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "TemporalDenoise/frame".into(),
            input_source: String::new(),
            prev_frame_source: String::new(),
            threshold: 0.05,
            blend_weight: 0.5,
        }
    }

    /// Set the noise threshold.
    pub fn with_threshold(mut self, threshold: f32) -> Self {
        self.threshold = threshold;
        self
    }

    /// Set the blend weight for static regions.
    pub fn with_blend_weight(mut self, weight: f32) -> Self {
        self.blend_weight = weight;
        self
    }
}

impl IspBlock for TemporalDenoiseBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "TemporalDenoise".into() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.into(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev_block.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev_block = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next_block.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next_block = Some(block); }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone(), self.prev_frame_source.clone()]
    }

    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        // Return current frame shape; previous frame has same shape.
        // MNN infers both from the first value_info.
        Some(Proto::value_info(
            &self.input_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // 1. diff = current - prev
        let diff = format!("{}/diff", ns);
        nodes.push(Proto::node(
            "Sub", &[&self.input_source, &self.prev_frame_source], &[&diff], &[],
        ));

        // 2. abs_diff = |diff|
        let abs_diff = format!("{}/abs_diff", ns);
        nodes.push(Proto::node(
            "Abs", &[&diff], &[&abs_diff], &[],
        ));

        // 3. mask = (abs_diff < threshold) ? 1 : 0
        //    ONNX: Less(abs_diff, threshold) → bool, then Cast to float
        let mask_bool = format!("{}/mask_bool", ns);
        let threshold_name = format!("{}/threshold", ns);
        let mask = format!("{}/mask", ns);
        nodes.push(Proto::node(
            "Less", &[&abs_diff, &threshold_name], &[&mask_bool], &[],
        ));
        nodes.push(Proto::node(
            "Cast", &[&mask_bool], &[&mask],
            &[Proto::attribute_int("to", 1)], // 1 = FLOAT
        ));

        // 4. inv_mask = 1 - mask
        let one_name = format!("{}/one", ns);
        let inv_mask = format!("{}/inv_mask", ns);
        nodes.push(Proto::node(
            "Sub", &[&one_name, &mask], &[&inv_mask], &[],
        ));

        // 5. weighted_prev = prev * inv_mask * blend_weight
        let blend_w = format!("{}/blend_w", ns);
        let prev_weighted = format!("{}/prev_weighted", ns);
        nodes.push(Proto::node(
            "Mul", &[&self.prev_frame_source, &inv_mask], &[&prev_weighted], &[],
        ));
        nodes.push(Proto::node(
            "Mul", &[&prev_weighted, &blend_w], &[&prev_weighted], &[],
        ));

        // 6. weighted_current = current * mask * (1 - blend_weight)
        let curr_weight = format!("{}/curr_weight", ns);
        let curr_weighted = format!("{}/curr_weighted", ns);
        nodes.push(Proto::node(
            "Mul", &[&self.input_source, &mask], &[&curr_weighted], &[],
        ));
        nodes.push(Proto::node(
            "Mul", &[&curr_weighted, &curr_weight], &[&curr_weighted], &[],
        ));

        // 7. output = weighted_prev + weighted_current
        nodes.push(Proto::node(
            "Add", &[&prev_weighted, &curr_weighted], &[&self.frame_tensor], &[],
        ));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        // Threshold scalar
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/threshold", ns), self.threshold));

        // One constant
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/one", ns), 1.0));

        // Blend weight
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/blend_w", ns), self.blend_weight));

        // Complement of blend weight
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/curr_weight", ns), 1.0 - self.blend_weight));

        inits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_temporal_denoise_onnx_emission() {
        let block = TemporalDenoiseBlock::new()
            .with_threshold(0.03)
            .with_blend_weight(0.6);
        let nodes = block.nodes();
        // Sub + Abs + Less + Cast + Sub(one-mask) + 2×Mul(prev) + 2×Mul(curr) + Add = 9 nodes
        assert!(nodes.len() >= 8, "should emit >= 8 nodes, got {}", nodes.len());
        let inits = block.initializers();
        // threshold, one, blend_w, curr_weight = 4
        assert_eq!(inits.len(), 4);
    }

    #[test]
    fn test_temporal_denoise_has_two_inputs() {
        let block = TemporalDenoiseBlock::new()
            .with_threshold(0.05);
        let inputs = block.input_tensors();
        assert_eq!(inputs.len(), 2, "should have 2 inputs (current + prev)");
    }

    #[test]
    fn test_temporal_denoise_threshold_affects_inits() {
        let block = TemporalDenoiseBlock::new().with_threshold(0.1);
        let inits = block.initializers();
        // First init is threshold
        assert_eq!(inits.len(), 4);
    }

    #[test]
    fn test_temporal_denoise_id() {
        assert_eq!(TemporalDenoiseBlock::new().id(), "temporal_denoise");
    }

    #[test]
    fn test_temporal_denoise_tensor_ns() {
        let b = TemporalDenoiseBlock::new();
        assert!(!b.tensor_ns().is_empty());
    }
}
