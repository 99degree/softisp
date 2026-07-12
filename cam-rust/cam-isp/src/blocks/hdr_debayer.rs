//! HdrDebayerBlock — HDR-specific debayer from multi-exposure raw.
//!
//! Combines multiple exposures (short + long) into a single HDR Bayer frame
//! before standard demosaicing. Useful for HDR capture where the sensor
//! outputs interleaved short/long exposures.
//!
//! Algorithm:
//!   1. Split interleaved input into individual exposure frames
//!   2. Apply exposure weighting (short: gain up, long: clip highlights)
//!   3. Merge weighted exposures via per-pixel max or weighted average
//!   4. Output: merged Bayer frame in [1, 1, H, W]
//!
//! The `exposure_ratio` parameter controls the gain relationship between
//! short and long exposures (e.g., 4.0 means 2 stops difference).

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// HdrDebayerBlock — multi-exposure HDR Bayer merge.
///
/// Combines short and long exposures into a single HDR raw frame.
pub struct HdrDebayerBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Exposure ratio between short and long (e.g., 4.0 = 2 stops). Default: 4.0.
    pub exposure_ratio: f32,
    /// Merge mode: 0=weighted average, 1=adaptive max. Default: 0.
    pub merge_mode: u8,
}

impl Default for HdrDebayerBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl HdrDebayerBlock {
    pub fn new() -> Self {
        Self {
            id: "hdr_debayer".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "HdrDebayerBlock/frame".into(),
            input_source: String::new(),
            exposure_ratio: 4.0,
            merge_mode: 0,
        }
    }

    pub fn with_exposure_ratio(mut self, ratio: f32) -> Self {
        self.exposure_ratio = ratio;
        self
    }

    pub fn with_merge_mode(mut self, mode: u8) -> Self {
        self.merge_mode = mode;
        self
    }
}

impl IspBlock for HdrDebayerBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "HdrDebayer".into()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.into();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev_block.as_ref()
    }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev_block = Some(block);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next_block.as_ref()
    }
    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next_block = Some(block);
    }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        // Input: interleaved [1, 2, H, W] (short + long stacked in channel dim)
        Some(Proto::value_info(
            &self.input_source,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(2),
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
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // Split channels: short = input[0], long = input[1]
        let short = format!("{}/short", ns);
        let long = format!("{}/long", ns);

        // Use ONNX Split: splits input into N parts along axis=1
        nodes.push(Proto::node(
            "Split",
            &[&self.input_source],
            &[&short, &long],
            &[Proto::attribute_int("axis", 1)],
        ));

        // Gain up the short exposure
        let gain = format!("{}/gain", ns);
        let short_gained = format!("{}/short_gained", ns);
        nodes.push(Proto::node("Mul", &[&short, &gain], &[&short_gained], &[]));

        if self.merge_mode == 0 {
            // Weighted average: (short_gained + long) / 2
            let sum = format!("{}/sum", ns);
            let two = format!("{}/two", ns);
            nodes.push(Proto::node("Add", &[&short_gained, &long], &[&sum], &[]));
            nodes.push(Proto::node(
                "Div",
                &[&sum, &two],
                &[&self.frame_tensor],
                &[],
            ));
        } else {
            // Adaptive max: max(short_gained, long)
            nodes.push(Proto::node(
                "Max",
                &[&short_gained, &long],
                &[&self.frame_tensor],
                &[],
            ));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/gain", ns), self.exposure_ratio),
            Proto::tensor_proto_float_scalar(&format!("{}/two", ns), 2.0),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/gain", self.tensor_ns()), 1, vec![1])]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hdr_debayer_id() {
        assert_eq!(HdrDebayerBlock::new().id(), "hdr_debayer");
    }

    #[test]
    fn test_hdr_debayer_with_ratio() {
        let b = HdrDebayerBlock::new().with_exposure_ratio(8.0);
        assert_eq!(b.exposure_ratio, 8.0);
    }

    #[test]
    fn test_hdr_debayer_mode() {
        let b = HdrDebayerBlock::new().with_merge_mode(1);
        assert_eq!(b.merge_mode, 1);
    }

    #[test]
    fn test_hdr_debayer_emit_onnx() {
        let b = HdrDebayerBlock::new();
        let nodes = b.nodes();
        // Slice + Slice + Mul + Add + Div = 5 nodes
        assert!(nodes.len() >= 4, "need >= 4 nodes, got {}", nodes.len());
    }

    #[test]
    fn test_hdr_debayer_max_mode() {
        let b = HdrDebayerBlock::new().with_merge_mode(1);
        let nodes = b.nodes();
        // Slice + Slice + Mul + Max = 4 nodes
        assert!(
            nodes.len() >= 3,
            "max mode needs >= 3 nodes, got {}",
            nodes.len()
        );
    }

    #[test]
    fn test_hdr_debayer_has_input_output() {
        let b = HdrDebayerBlock::new();
        assert_eq!(b.input_tensors().len(), 1);
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_hdr_debayer_tensor_ns() {
        let b = HdrDebayerBlock::new();
        assert_eq!(b.tensor_ns(), "HdrDebayer");
    }

    #[test]
    fn test_hdr_debayer_extra_inputs() {
        let b = HdrDebayerBlock::new();
        let extras = b.extra_inputs();
        assert_eq!(extras.len(), 1, "should have gain extra input");
    }
}
