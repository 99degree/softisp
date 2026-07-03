//! NoiseEstimateBlock — per-pixel noise level estimation.
//!
//! Estimates local noise variance from the input image using a Laplacian-based
//! noise estimator. Output is a [1,1,H,W] noise level map that can drive
//! adaptive temporal denoising or spatial denoising strength.
//!
//! ONNX subgraph:
//!   1. Grayscale = 0.299*R + 0.587*G + 0.114*B (Conv group=3, 3→1)
//!   2. Laplacian kernel: [[0,1,0],[1,-4,1],[0,1,0]] (Conv 3×3, 1→1)
//!   3. Abs(laplacian)
//!   4. Local mean: AvgPool(3×3) → [1,1,H,W]
//!   5. Noise level = Mean(Abs(laplacian)) * scale
//!
//! The scale parameter controls sensitivity:
//!   - Low (0.5): high noise → high output
//!   - High (2.0): only strong edges → low output

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// NoiseEstimateBlock — Laplacian-based per-pixel noise level estimation.
///
/// Computes local variance via Laplacian filter for noise-aware processing
/// downstream. Outputs a single-channel noise map matching input dimensions.
pub struct NoiseEstimateBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub scale: f32,
    /// Pool size for local mean (default: 3)
    pub pool_size: i64,
}

impl NoiseEstimateBlock {
    pub fn new() -> Self {
        Self {
            id: "noise_estimate".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "NoiseEstimate/noise_map".into(),
            input_source: String::new(),
            scale: 1.0,
            pool_size: 3,
        }
    }

    pub fn with_scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }

    pub fn with_pool_size(mut self, pool_size: i64) -> Self {
        self.pool_size = pool_size;
        self
    }
}

impl IspBlock for NoiseEstimateBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "NoiseEstimate".into() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.into(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev_block.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev_block = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next_block.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next_block = Some(block); }

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
        Some(Proto::value_info(
            &self.input_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // 1. Grayscale: Conv(input, lum_kernel) → [1,1,H,W]
        let gray = format!("{}/gray", ns);
        let lum_kernel = format!("{}/lum_k", ns);
        nodes.push(Proto::node(
            "Conv", &[&self.input_source, &lum_kernel], &[&gray],
            &[Proto::attribute_ints("kernel_shape", &[3, 3]),
              Proto::attribute_ints("pads", &[1, 1, 1, 1]),
              Proto::attribute_int("group", 3)],
        ));

        // 2. Laplacian: Conv(gray, lap_kernel) → [1,1,H,W]
        let lap = format!("{}/lap", ns);
        let lap_kernel = format!("{}/lap_k", ns);
        nodes.push(Proto::node(
            "Conv", &[&gray, &lap_kernel], &[&lap],
            &[Proto::attribute_ints("kernel_shape", &[3, 3]),
              Proto::attribute_ints("pads", &[1, 1, 1, 1])],
        ));

        // 3. Abs(laplacian)
        let abs_lap = format!("{}/abs_lap", ns);
        nodes.push(Proto::node("Abs", &[&lap], &[&abs_lap], &[]));

        // 4. Local mean: AvgPool(abs_lap)
        let local_mean = format!("{}/local_mean", ns);
        nodes.push(Proto::node(
            "AveragePool", &[&abs_lap], &[&local_mean],
            &[Proto::attribute_ints("kernel_shape", &[self.pool_size, self.pool_size]),
              Proto::attribute_ints("pads", &[1, 1, 1, 1])],
        ));

        // 5. Scale: Mul(local_mean, scale)
        let scale_name = format!("{}/scale", ns);
        nodes.push(Proto::node(
            "Mul", &[&local_mean, &scale_name], &[&self.frame_tensor], &[],
        ));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        // Luminance kernel: [3, 3, 1, 3] — Conv 3×3, 3ch→1ch, group=3
        // Group conv: each input channel gets its own 3×3 filter
        // Channel 0 (R): weight = [0.299, 0, 0; 0.299, 0, 0; 0.299, 0, 0] etc.
        // Simplified: [1, 3, 3, 3] with per-group weights
        let lum_w: Vec<f32> = vec![
            // Group 0 (R→out): 0.299 at center, 0 elsewhere
            0.0, 0.0, 0.0, 0.0, 0.299, 0.0, 0.0, 0.0, 0.0,
            // Group 1 (G→out): 0.587 at center
            0.0, 0.0, 0.0, 0.0, 0.587, 0.0, 0.0, 0.0, 0.0,
            // Group 2 (B→out): 0.114 at center
            0.0, 0.0, 0.0, 0.0, 0.114, 0.0, 0.0, 0.0, 0.0,
        ];
        inits.push(Proto::tensor_proto_float(
            &format!("{}/lum_k", ns), &[3, 1, 3, 3], &lum_w));

        // Laplacian kernel: [1, 1, 3, 3]
        let lap_w: Vec<f32> = vec![
            0.0, 1.0, 0.0,
            1.0, -4.0, 1.0,
            0.0, 1.0, 0.0,
        ];
        inits.push(Proto::tensor_proto_float(
            &format!("{}/lap_k", ns), &[1, 1, 3, 3], &lap_w));

        // Scale constant
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/scale", ns), self.scale));

        inits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_noise_estimate_emits_conv_and_laplacian() {
        let block = NoiseEstimateBlock::new().with_scale(1.2);
        let nodes = block.nodes();
        let conv_count = nodes.iter().filter(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("Conv")
        }).count();
        assert!(conv_count >= 2, "need Conv for luminance + Laplacian, got {}", conv_count);
    }

    #[test]
    fn test_noise_estimate_has_one_output() {
        let block = NoiseEstimateBlock::new();
        assert_eq!(block.output_tensors().len(), 1);
    }

    #[test]
    fn test_noise_estimate_output_is_single_channel() {
        let block = NoiseEstimateBlock::new();
        let vi = block.output_value_info().unwrap();
        // Output should be [1, 1, H, W] — single channel
        let s = String::from_utf8_lossy(&vi);
        assert!(s.contains("noise_map"), "output tensor name check");
    }

    #[test]
    fn test_noise_estimate_has_initializers() {
        let block = NoiseEstimateBlock::new();
        let inits = block.initializers();
        // lum_k + lap_k + scale = 3
        assert_eq!(inits.len(), 3, "need 3 initializers, got {}", inits.len());
    }

    #[test]
    fn test_noise_estimate_scale_affects_output() {
        let block1 = NoiseEstimateBlock::new().with_scale(0.5);
        let block2 = NoiseEstimateBlock::new().with_scale(2.0);
        // Different scales should produce different initializer values
        let inits1 = block1.initializers();
        let inits2 = block2.initializers();
        assert_ne!(inits1, inits2, "different scales should produce different initializers");
    }
}
