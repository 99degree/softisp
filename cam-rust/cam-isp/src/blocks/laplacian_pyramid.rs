//! LaplacianPyramidBlock — multi-scale Laplacian decomposition.
//!
//! Decomposes the input into N levels of detail (Gaussian pyramid + Laplacian pyramid).
//! Unlike PyramidBlock (single-level nearest-neighbor 2×), this produces
//! a full multi-scale representation for HDR merge, frequency editing,
//! and multi-scale blending.
//!
//! ONNX subgraph:
//!   1. Compute Gaussian pyramid: sequential AvgPool(2×2, stride=2)
//!   2. Compute Laplacian pyramid: level[i] = gauss[i] - Upsample(gauss[i+1])
//!   3. Output: concat all Laplacian levels + residual Gaussian
//!
//! Output format:
//!   - Level 0 (fine): high-frequency detail at full resolution
//!   - Level 1: medium-frequency detail at ½ resolution
//!   - ...
//!   - Level N-1: coarsest detail
//!   - Level N (residual): low-frequency Gaussian at smallest resolution

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// LaplacianPyramidBlock — multi-scale Laplacian pyramid decomposition.
///
/// Decomposes image into frequency bands for HDR merge and multi-scale processing.
pub struct LaplacianPyramidBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Number of pyramid levels (2-5 typical). Default: 3.
    pub levels: usize,
}

impl Default for LaplacianPyramidBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl LaplacianPyramidBlock {
    pub fn new() -> Self {
        Self {
            id: "laplacian_pyramid".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "LaplacianPyramidBlock/frame".into(),
            input_source: String::new(),
            levels: 3,
        }
    }

    pub fn with_levels(mut self, levels: usize) -> Self {
        self.levels = levels.clamp(1, 5);
        self
    }
}

impl IspBlock for LaplacianPyramidBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "LaplacianPyramid".into() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.into(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev_block.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev_block = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next_block.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next_block = Some(block); }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn graph_output_name(&self) -> Option<&str> { Some(&self.frame_tensor) }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.input_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        // Output is concatenated Laplacian levels: [1, 3*(levels+1), H, W]
        // (upsampled to full resolution for concat compatibility)
        Some(Proto::value_info(&self.frame_tensor,
            &[Proto::tensor_dim_value(1),
              Proto::tensor_dim_value(3 * (self.levels as i64 + 1)),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // Build Gaussian pyramid: g0=input, g1=AvgPool(g0), g2=AvgPool(g1), ...
        // Then build Laplacian pyramid:
        //   laplac[i] = gauss[i] - Upsample(gauss[i+1])
        //   laplac[N-1] = gauss[N-1] (residual)
        //
        // Simplified for ONNX: each level uses AvgPool(2x2) + Sub
        // For concat, all levels are at input resolution via Resize

        let mut current = self.input_source.clone();
        let mut gauss_names = vec![current.clone()];

        // Gaussian pyramid: sequential AvgPool
        for i in 0..self.levels {
            let g_out = format!("{}/g{}", ns, i + 1);
            nodes.push(Proto::node("AveragePool",
                &[&current],
                &[&g_out],
                &[Proto::attribute_ints("kernel_shape", &[2, 2]),
                  Proto::attribute_ints("strides", &[2, 2])]));
            gauss_names.push(g_out.clone());
            current = g_out;
        }

        // Laplacian pyramid: laplac[i] = gauss[i] - Resize(gauss[i+1])
        let mut laplac_names = Vec::new();
        for i in 0..self.levels {
            let upsampled = format!("{}/up{}", ns, i + 1);
            let lap_out = format!("{}/lap{}", ns, i);

            // Upsample gauss[i+1] to match gauss[i] resolution
            nodes.push(Proto::node("Resize",
                &[&gauss_names[i + 1]],
                &[&upsampled],
                &[Proto::attribute_string("mode", "nearest")]));

            // laplac[i] = gauss[i] - upsampled(gauss[i+1])
            nodes.push(Proto::node("Sub",
                &[&gauss_names[i], &upsampled],
                &[&lap_out], &[]));

            laplac_names.push(lap_out);
        }
        // Residual: the coarsest Gaussian
        laplac_names.push(gauss_names.last().unwrap().clone());

        // Resize all Laplacian levels to full resolution for concat
        let mut resized_names = Vec::new();
        for (i, lap) in laplac_names.iter().enumerate() {
            let resized = format!("{}/resized{}", ns, i);
            // Use Resize with nearest neighbor to upscale
            nodes.push(Proto::node("Resize",
                &[lap],
                &[&resized],
                &[Proto::attribute_string("mode", "nearest")]));
            resized_names.push(resized);
        }

        // Concat all levels along channel dimension
        let input_refs: Vec<&str> = resized_names.iter().map(|s| s.as_str()).collect();
        nodes.push(Proto::node("Concat",
            &input_refs,
            &[&self.frame_tensor],
            &[Proto::attribute_int("axis", 1)]));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        // No constant initializers needed — all ops use standard attributes
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_laplacian_id() {
        assert_eq!(LaplacianPyramidBlock::new().id(), "laplacian_pyramid");
    }

    #[test]
    fn test_laplacian_with_levels() {
        let b = LaplacianPyramidBlock::new().with_levels(4);
        assert_eq!(b.levels, 4);
    }

    #[test]
    fn test_laplacian_clamps_levels() {
        let b = LaplacianPyramidBlock::new().with_levels(10);
        assert_eq!(b.levels, 5, "should clamp to max 5");
        let b = LaplacianPyramidBlock::new().with_levels(0);
        assert_eq!(b.levels, 1, "should clamp to min 1");
    }

    #[test]
    fn test_laplacian_emit_onnx() {
        let b = LaplacianPyramidBlock::new().with_levels(3);
        let nodes = b.nodes();
        // 3 AvgPool + 3 Resize(upsample) + 3 Sub + 3 Resize(resize) + 1 Concat = 13
        assert!(nodes.len() >= 10, "need >= 10 nodes, got {}", nodes.len());
    }

    #[test]
    fn test_laplacian_has_input_output() {
        let b = LaplacianPyramidBlock::new();
        assert_eq!(b.input_tensors().len(), 1);
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_laplacian_tensor_ns() {
        let b = LaplacianPyramidBlock::new();
        assert_eq!(b.tensor_ns(), "LaplacianPyramid");
    }

    #[test]
    fn test_laplacian_graph_output() {
        let b = LaplacianPyramidBlock::new();
        assert!(b.graph_output_name().is_some());
    }

    #[test]
    fn test_laplacian_default_levels() {
        let b = LaplacianPyramidBlock::new();
        assert_eq!(b.levels, 3);
    }
}
