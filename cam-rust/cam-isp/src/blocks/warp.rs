//! WarpGridBlock — Unified warp + lens shading via GridSample.
//!
//! Fuses EIS/GDC warp (GridSample) with lens shading correction (Mul)
//! into a single ONNX subgraph. Both operations share the same output
//! pixel loop — zero extra GPU cost for vignetting correction.
//!
//! ONNX subgraph:
//!   GridSample(input, grid) → warped
//!   Mul(warped, shading_lut) → output  (if lens shading enabled)
//!
//! GridSample node specs:
//! - Input: [1,3,H,W] float32 RGB image tensor
//! - Grid: [1,OH,OW,2] float32 sampling grid in [-1,1] range
//! - Mode: LINEAR, CLAMP
//! - Target: [1,3,OH,OW] out

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct WarpGridBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub target_width: u32,
    pub target_height: u32,
    pub output_width: u32,
    pub output_height: u32,
    pub rotate_mode: i32,
    pub grid_initializer: Option<Vec<u8>>,
    pub bcs: Option<(f32, f32, f32)>,
    // Lens shading: [1,3,H,W] radial gain LUT (fused into warp output).
    pub shading_lut: Option<Vec<u8>>,
}

impl WarpGridBlock {
    pub fn new(target_width: u32, target_height: u32) -> Self {
        Self {
            id: "warp_grid".into(),
            prev: None,
            next: None,
            frame_tensor: "WarpGrid/frame".into(),
            input_source: String::new(),
            target_width,
            target_height,
            output_width: target_width,
            output_height: target_height,
            rotate_mode: 0,
            grid_initializer: None,
            bcs: None,
            shading_lut: None,
        }
    }

    pub fn with_output_resolution(mut self, width: u32, height: u32) -> Self {
        self.output_width = width;
        self.output_height = height;
        self
    }

    pub fn with_rotate(mut self, mode: i32) -> Self {
        self.rotate_mode = mode;
        self
    }

    pub fn with_grid(mut self, grid: Option<Vec<f32>>) -> Self {
        if let Some(g) = grid {
            let initializer = Proto::tensor_proto_float(
                &format!("{}/grid", self.tensor_ns()),
                &[1, self.output_height as i64, self.output_width as i64, 2],
                &g,
            );
            self.grid_initializer = Some(initializer);
        }
        self
    }

    pub fn with_bcs(mut self, brightness: f32, contrast: f32, saturation: f32) -> Self {
        self.bcs = Some((brightness, contrast, saturation));
        self
    }

    // ── Lens shading ──────────────────────────────────────────────

    /// Fuse lens shading correction into the warp output.
    /// `corner_gain`: multiplier at corners (e.g. 1.5 = 50% boost).
    /// `center_gain`: multiplier at center (usually 1.0).
    pub fn with_lens_shading(mut self, corner_gain: f32, center_gain: f32) -> Self {
        let h = self.output_height;
        let w = self.output_width;
        let lut = Self::generate_radial_lut(h, w, corner_gain, center_gain);
        let name = format!("{}/shading_lut", self.tensor_ns());
        let init = Proto::tensor_proto_float(
            &name,
            &[1, 3, h as i64, w as i64],
            &lut,
        );
        self.shading_lut = Some(init);
        self
    }

    /// Use a pre-built shading LUT (already packed as [1,3,H,W] f32).
    pub fn with_shading_lut(mut self, h: u32, w: u32, lut: Vec<f32>) -> Self {
        let name = format!("{}/shading_lut", self.tensor_ns());
        let init = Proto::tensor_proto_float(
            &name,
            &[1, 3, h as i64, w as i64],
            &lut,
        );
        self.shading_lut = Some(init);
        self
    }

    /// Generate radial gain LUT: gain = center + (corner - center) * r².
    pub fn generate_radial_lut(h: u32, w: u32, corner_gain: f32, center_gain: f32) -> Vec<f32> {
        let cx = (w as f32 - 1.0) * 0.5;
        let cy = (h as f32 - 1.0) * 0.5;
        let max_r_sq = cx * cx + cy * cy;
        let gain_range = corner_gain - center_gain;
        let mut lut = Vec::with_capacity((h * w * 3) as usize);
        for _ in 0..3 {
            for y in 0..h {
                for x in 0..w {
                    let dx = x as f32 - cx;
                    let dy = y as f32 - cy;
                    let t = ((dx * dx + dy * dy) / max_r_sq).min(1.0);
                    lut.push(center_gain + gain_range * t);
                }
            }
        }
        lut
    }

    // ── Rotation helpers ──────────────────────────────────────────

    fn swaps_dims(&self) -> bool {
        matches!(self.rotate_mode, 1 | 3)
    }

    fn needs_hflip(&self) -> bool {
        matches!(self.rotate_mode, 1 | 2 | 4)
    }

    fn needs_vflip(&self) -> bool {
        matches!(self.rotate_mode, 2 | 3 | 5)
    }
}

impl IspBlock for WarpGridBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "WarpGrid".into() }
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
        Some(Proto::value_info(
            &self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_value(self.output_height as i64),
              Proto::tensor_dim_value(self.output_width as i64)], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // ── 1. GridSample ─────────────────────────────────────────
        let grid_input = if self.grid_initializer.is_some() {
            format!("{}/grid", ns)
        } else {
            format!("{}/grid_init", ns)
        };
        let sampled = format!("{}/sampled", ns);
        nodes.push(Proto::node(
            "GridSample",
            &[&self.input_source, &grid_input],
            &[&sampled],
            &[
                Proto::attribute_string("mode", "LINEAR"),
                Proto::attribute_string("padding_mode", "CLAMP"),
                Proto::attribute_string("align_corners", "false"),
            ],
        ));

        let mut prev = sampled;

        // ── 2. Rotate/flip (optional) ─────────────────────────────
        if self.swaps_dims() {
            let t = format!("{}/transposed", ns);
            nodes.push(Proto::node("Transpose", &[&prev], &[&t],
                &[Proto::attribute_ints("perm", &[0, 1, 3, 2])]));
            prev = t;
        }

        if self.needs_hflip() {
            let h = format!("{}/hflipped", ns);
            nodes.push(Proto::node("Slice",
                &[&prev,
                  &format!("{}/hflip_starts", ns),
                  &format!("{}/hflip_ends", ns),
                  &format!("{}/hflip_axes", ns),
                  &format!("{}/hflip_steps", ns)],
                &[&h], &[]));
            prev = h;
        }

        if self.needs_vflip() {
            let v = format!("{}/vflipped", ns);
            nodes.push(Proto::node("Slice",
                &[&prev,
                  &format!("{}/vflip_starts", ns),
                  &format!("{}/vflip_ends", ns),
                  &format!("{}/vflip_axes", ns),
                  &format!("{}/vflip_steps", ns)],
                &[&v], &[]));
            prev = v;
        }

        // ── 3. Lens shading (fused Mul) ───────────────────────────
        if self.shading_lut.is_some() {
            let lut_name = format!("{}/shading_lut", ns);
            let shaded = format!("{}/shaded", ns);
            nodes.push(Proto::node("Mul", &[&prev, &lut_name], &[&shaded], &[]));
            prev = shaded;
        }

        // ── 4. BCS display layer ──────────────────────────────────
        if let Some((bright, contr, _sat)) = self.bcs {
            let scope = format!("{}/bcs", ns);
            let _ = bright;
            let _ = contr;

            let bright_out = format!("{}/bright_out", scope);
            let contr_mul = format!("{}/contr_mul", scope);
            let contr_out = format!("{}/contr_out", scope);
            let luma_vec = format!("{}/luma_out", scope);
            let final_out = format!("{}/final", scope);

            nodes.push(Proto::node("Add", &[&prev, &format!("{}/bright_add", scope)],
                &[&bright_out], &[]));
            nodes.push(Proto::node("Sub", &[&bright_out, &format!("{}/half", scope)],
                &[&contr_mul], &[]));
            nodes.push(Proto::node("Mul", &[&contr_mul, &format!("{}/contr_w", scope)],
                &[&contr_out], &[]));
            nodes.push(Proto::node("Add", &[&contr_out, &format!("{}/half", scope)],
                &[&luma_vec], &[]));
            nodes.push(Proto::node("Mul", &[&luma_vec, &format!("{}/luma_w", scope)],
                &[&final_out], &[]));
            nodes.push(Proto::node("Identity", &[&final_out], &[&self.frame_tensor], &[]));
            return nodes;
        }

        // ── 5. Final output ───────────────────────────────────────
        if prev != self.frame_tensor {
            nodes.push(Proto::node("Identity", &[&prev], &[&self.frame_tensor], &[]));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        // Grid initializer
        if let Some(init) = &self.grid_initializer {
            inits.push(init.clone());
        }

        // Lens shading LUT initializer
        if let Some(lut) = &self.shading_lut {
            inits.push(lut.clone());
        }

        if self.swaps_dims() {
            return inits;
        }

        let (height, width) = (self.output_height as i64, self.output_width as i64);

        if self.needs_hflip() && width > 0 {
            inits.push(Proto::tensor_proto_int64(&format!("{}/hflip_starts", ns), &[width - 1]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/hflip_ends", ns), &[-1]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/hflip_axes", ns), &[3]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/hflip_steps", ns), &[-1]));
        }

        if self.needs_vflip() && height > 0 {
            inits.push(Proto::tensor_proto_int64(&format!("{}/vflip_starts", ns), &[height - 1]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/vflip_ends", ns), &[-1]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/vflip_axes", ns), &[2]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/vflip_steps", ns), &[-1]));
        }

        // BCS constants
        if let Some((bright, contr, sat)) = self.bcs {
            let scope = format!("{}/bcs", ns);
            inits.push(Proto::tensor_proto_float_scalar(&format!("{}/bright_add", scope), bright));
            inits.push(Proto::tensor_proto_float_scalar(&format!("{}/half", scope), 0.5));
            inits.push(Proto::tensor_proto_float_scalar(&format!("{}/contr_w", scope), contr));
            inits.push(Proto::tensor_proto_float(&format!("{}/luma_w", scope), &[3],
                &[0.299, 0.587, 0.114]));
            let sat_w = if sat >= 0.0 { vec![1.0, sat, sat] } else { vec![1.0, 1.4, 0.5] };
            inits.push(Proto::tensor_proto_float(&format!("{}/sat_w", scope), &[3], &sat_w));
        }

        inits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_radial_lut_center_is_identity() {
        let lut = WarpGridBlock::generate_radial_lut(100, 100, 1.5, 1.0);
        let center = lut[5050];
        assert!((center - 1.0).abs() < 0.05, "center gain should be ~1.0, got {}", center);
    }

    #[test]
    fn test_radial_lut_corners_are_brighter() {
        let lut = WarpGridBlock::generate_radial_lut(100, 100, 2.0, 1.0);
        let center = lut[5050];
        let corner = lut[0];
        assert!(corner > center, "corner {} > center {}", corner, center);
        assert!((corner - 2.0).abs() < 0.05, "corner gain should be ~2.0, got {}", corner);
    }

    #[test]
    fn test_warp_with_lens_shading() {
        let grid = vec![0.0f32; 32 * 32 * 2];
        let block = WarpGridBlock::new(32, 32)
            .with_grid(Some(grid))
            .with_lens_shading(1.5, 1.0);
        let nodes = block.nodes();
        // GridSample + Mul(shading) + Identity = 3 nodes
        assert_eq!(nodes.len(), 3, "should have GridSample + Mul + Identity");
        let inits = block.initializers();
        // grid + shading_lut = 2
        assert!(inits.len() >= 2, "should have grid + shading_lut inits");
    }

    #[test]
    fn test_warp_without_lens_shading() {
        let grid = vec![0.0f32; 32 * 32 * 2];
        let block = WarpGridBlock::new(32, 32)
            .with_grid(Some(grid));
        let nodes = block.nodes();
        // GridSample + Identity = 2 nodes
        assert_eq!(nodes.len(), 2);
    }
}
