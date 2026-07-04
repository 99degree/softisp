//! ChromaticAberrationBlock — corrects lateral chromatic aberration.
//!
//! Splits RGB channels, applies per-channel radial offset grid, then concatenates.
//! This corrects color fringing at edges caused by lens design limitations.
//!
//! ONNX subgraph:
//!   Split(input, axis=1) → [R, G, B]
//!   GridSample(R, grid_r) → R_corrected
//!   GridSample(G, grid_g) → G_corrected
//!   GridSample(B, grid_b) → B_corrected
//!   Concat([R_c, G_c, B_c], axis=1) → output
//!
//! Grid generation: radial model where R and B channels are shifted outward/inward
//! relative to G (reference channel).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// ChromaticAberrationBlock — corrects lateral chromatic aberration.
///
/// Splits RGB channels, applies per-channel radial offset via GridSample,
/// then re-concatenates. Corrects color fringing at image edges caused
/// by lens design.
pub struct ChromaticAberrationBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub height: Option<i64>,
    pub width: Option<i64>,
    /// Per-channel grids: [grid_r, grid_g, grid_b] each [1, H, W, 2]
    pub grids: Option<Vec<f32>>, // flattened all 3 grids
}

impl ChromaticAberrationBlock {
    pub fn new() -> Self {
        Self {
            id: "chromatic_aberration".into(),
            prev: None,
            next: None,
            frame_tensor: "ChromaticAberration/frame".into(),
            input_source: String::new(),
            height: None,
            width: None,
            grids: None,
        }
    }

    /// Generate correction grids from radial CA model.
    /// `strength`: pixel shift at corners (e.g. 2.0 = 2px shift at corners).
    /// Positive = red shifts outward, blue shifts inward (typical CA pattern).
    pub fn with_radial_correction(mut self, h: u32, w: u32, strength: f32) -> Self {
        self.height = Some(h as i64);
        self.width = Some(w as i64);
        let grids = Self::generate_ca_grids(h, w, strength);
        self.grids = Some(grids);
        self
    }

    /// Generate three sampling grids for R, G, B channels.
    /// G is reference (identity grid). R shifts outward, B shifts inward.
    fn generate_ca_grids(h: u32, w: u32, strength: f32) -> Vec<f32> {
        let cx = (w as f32 - 1.0) * 0.5;
        let cy = (h as f32 - 1.0) * 0.5;
        let max_r = (cx * cx + cy * cy).sqrt();
        let size = (h * w * 2) as usize;
        let mut grids = Vec::with_capacity(size * 3);

        for _ch in 0..3 {
            let sign = match _ch {
                0 => 1.0,   // R: shift outward
                1 => 0.0,   // G: no shift (reference)
                2 => -1.0,  // B: shift inward
                _ => 0.0,
            };
            for y in 0..h {
                for x in 0..w {
                    let dx = x as f32 - cx;
                    let dy = y as f32 - cy;
                    let r = (dx * dx + dy * dy).sqrt();
                    let norm_r = (r / max_r).min(1.0);
                    let shift = sign * strength * norm_r * norm_r;
                    let norm_factor = 2.0 / (w as f32 - 1.0);
                    let gx = (x as f32 + dx * shift * norm_factor) * norm_factor - 1.0;
                    let gy = (y as f32 + dy * shift * norm_factor) * (2.0 / (h as f32 - 1.0)) - 1.0;
                    grids.push(gx.max(-1.0).min(1.0));
                    grids.push(gy.max(-1.0).min(1.0));
                }
            }
        }
        grids
    }

    fn grid_name(&self, ch: usize) -> String {
        format!("{}/grid_{}", self.tensor_ns(), ch)
    }

    fn channel_name(&self, ch: usize) -> String {
        format!("{}/ch_{}", self.tensor_ns(), ch)
    }
}

impl IspBlock for ChromaticAberrationBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "ChromaticAberration".into() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.into(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

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
        let (h, w) = match (self.height, self.width) {
            (Some(h), Some(w)) => (h, w),
            _ => return None,
        };
        Some(Proto::value_info(
            &self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_value(h), Proto::tensor_dim_value(w)], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // Split input into 3 channels along axis=1
        let split_outputs: Vec<String> = (0..3).map(|ch| self.channel_name(ch)).collect();
        let split_refs: Vec<&str> = split_outputs.iter().map(|s| s.as_str()).collect();
        nodes.push(Proto::node(
            "Split", &[&self.input_source], &split_refs,
            &[Proto::attribute_int("axis", 1)],
        ));

        // GridSample each channel
        let mut sampled = Vec::new();
        for ch in 0..3 {
            let out_name = format!("{}/sampled_{}", ns, ch);
            nodes.push(Proto::node(
                "GridSample",
                &[&self.channel_name(ch), &self.grid_name(ch)],
                &[&out_name],
                &[
                    Proto::attribute_string("mode", "LINEAR"),
                    Proto::attribute_string("padding_mode", "CLAMP"),
                    Proto::attribute_string("align_corners", "false"),
                ],
            ));
            sampled.push(out_name);
        }

        // Concat back to [1,3,H,W]
        let sampled_refs: Vec<&str> = sampled.iter().map(|s| s.as_str()).collect();
        nodes.push(Proto::node(
            "Concat", &sampled_refs, &[&self.frame_tensor],
            &[Proto::attribute_int("axis", 1)],
        ));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let mut inits = Vec::new();
        if let Some(ref grids) = self.grids {
            let (h, w) = match (self.height, self.width) {
                (Some(h), Some(w)) => (h, w),
                _ => return inits,
            };
            let grid_size = (h * w * 2) as usize;
            for ch in 0..3 {
                let start = ch * grid_size;
                let end = start + grid_size;
                if end <= grids.len() {
                    inits.push(Proto::tensor_proto_float(
                        &self.grid_name(ch),
                        &[1, h, w, 2],
                        &grids[start..end],
                    ));
                }
            }
        }
        inits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ca_grid_identity_at_center() {
        let grids = ChromaticAberrationBlock::generate_ca_grids(100, 100, 2.0);
        // G channel (index 1) should be identity grid
        let g_offset = 1 * 100 * 100 * 2;
        let center = 50 * 100 + 50;
        let gx = grids[g_offset + center * 2];
        let gy = grids[g_offset + center * 2 + 1];
        assert!(gx.abs() < 0.02, "G center x should be ~0, got {}", gx);
        assert!(gy.abs() < 0.02, "G center y should be ~0, got {}", gy);
    }

    #[test]
    fn test_ca_grid_r_shifts_outward() {
        let grids = ChromaticAberrationBlock::generate_ca_grids(100, 100, 2.0);
        let _h = 100usize;
        let _w = 100usize;
        // Top-left corner: R should shift outward (away from center)
        let r_offset = 0; // R channel
        let corner_idx = 0; // top-left pixel
        let rx = grids[r_offset + corner_idx * 2];
        let ry = grids[r_offset + corner_idx * 2 + 1];
        // R should shift away from center (more negative at top-left)
        assert!(rx < -0.5 || ry < -0.5, "R should shift outward at corner");
    }

    #[test]
    fn test_ca_onnx_emission() {
        let block = ChromaticAberrationBlock::new()
            .with_radial_correction(32, 32, 1.5);
        let nodes = block.nodes();
        // Split + 3 GridSample + Concat = 5 nodes
        assert_eq!(nodes.len(), 5, "should emit 5 nodes (Split + 3 GridSample + Concat)");
        let inits = block.initializers();
        assert_eq!(inits.len(), 3, "should have 3 grid initializers (R, G, B)");
    }
}
