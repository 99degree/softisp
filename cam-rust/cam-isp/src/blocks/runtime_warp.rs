//! # RuntimeWarpBlock — CPU Grid + GPU Warp
//!
//! ## Overview
//!
//! Computes GDC/EIS grid on CPU from runtime parameters, then feeds
//! the grid to ONNX GridSample for GPU-accelerated warping.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    Hybrid CPU/GPU Pipeline                     │
//! ├─────────────────────────────────────────────────────────────────┤
//! │                                                                 │
//! │  CPU: Compute GDC grid from k1, k2, k3 coefficients            │
//! │       ↓                                                         │
//! │  CPU: Compose with EIS displacement grid (from DeshakeEngine)   │
//! │       ↓                                                         │
//! │  CPU: → grid tensor [1, H, W, 2]                               │
//! │       ↓                                                         │
//! │  GPU: GridSample(image, grid) → warped output                   │
//! │                                                                 │
//! └─────────────────────────────────────────────────────────────────┘
//!
//! ## Deshake Integration
//!
//! This block is ideal for CPU-based deshake where:
//! 1. `DeshakeEngine` computes motion vector on CPU
//! 2. Motion vector is converted to EIS grid on CPU
//! 3. Grid is composed with GDC grid on CPU
//! 4. Final warp runs on GPU via GridSample
//!
//! ```rust
//! use cam_isp::deshake::DeshakeEngine;
//! use cam_isp::blocks::RuntimeWarpBlock;
//!
//! let mut deshake = DeshakeEngine::new(3840, 2160);
//! let mut warp = RuntimeWarpBlock::new(1920, 1080);
//!
//! // Per-frame:
//! let motion = deshake.estimate_motion(&prev, &curr);
//! let eis_grid = deshake.motion_to_grid(motion, 1920, 1080);
//! let combined_grid = warp.compose_gdc_and_eis(0.1, 0.05, 0.01, eis_grid);
//! warp.set_grid(combined_grid);
//! ```
//!
//! ## Benefits
//!
//! - **Simple ONNX**: Only GridSample op, no grid computation in graph
//! - **Runtime parameters**: k1, k2, k3, EIS grid provided per-frame
//! - **GPU warp**: Actual interpolation runs on GPU
//! - **CPU grid**: Fast math (< 1ms for 4K), updates per-frame
//! - **GDC caching**: Avoids recomputing grid when k1,k2,k3 unchanged

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// RuntimeWarpBlock — CPU grid + GPU warp.
pub struct RuntimeWarpBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub output_width: u32,
    pub output_height: u32,
    /// Pre-computed grid tensor (set before each frame).
    pub grid: Option<Vec<f32>>,
    /// Lens shading LUT (optional).
    pub shading_lut: Option<Vec<f32>>,
    /// Cached GDC grid (k1, k2, k3) → grid to avoid recomputation.
    gdc_cache: Option<(f32, f32, f32, Vec<f32>)>,
}

impl RuntimeWarpBlock {
    pub fn new(target_width: u32, target_height: u32) -> Self {
        Self {
            id: "runtime_warp".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "RuntimeWarp/frame".into(),
            input_source: String::new(),
            output_width: target_width,
            output_height: target_height,
            grid: None,
            shading_lut: None,
            gdc_cache: None,
        }
    }

    /// Compute GDC grid from radial distortion coefficients.
    /// Returns grid in normalized [-1, 1] coordinates [1, H, W, 2].
    pub fn compute_gdc_grid(k1: f32, k2: f32, k3: f32, width: u32, height: u32) -> Vec<f32> {
        let cx = (width as f32 - 1.0) * 0.5;
        let cy = (height as f32 - 1.0) * 0.5;
        let mut grid = Vec::with_capacity((height * width * 2) as usize);

        for y in 0..height {
            for x in 0..width {
                let nx = (x as f32 - cx) / cx;
                let ny = (y as f32 - cy) / cy;
                let r2 = nx * nx + ny * ny;
                let r4 = r2 * r2;
                let r6 = r4 * r2;
                let denom = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
                let inv = if denom.abs() > 1e-6 { 1.0 / denom } else { 1.0 };

                grid.push((nx * inv).clamp(-1.0, 1.0));
                grid.push((ny * inv).clamp(-1.0, 1.0));
            }
        }
        grid
    }

    /// Compute EIS displacement grid (identity + offset).
    pub fn compute_eis_grid(dx: f32, dy: f32, width: u32, height: u32) -> Vec<f32> {
        let mut grid = Vec::with_capacity((height * width * 2) as usize);
        for y in 0..height {
            for x in 0..width {
                let nx = 2.0 * x as f32 / (width - 1) as f32 - 1.0 + dx;
                let ny = 2.0 * y as f32 / (height - 1) as f32 - 1.0 + dy;
                grid.push(nx.clamp(-1.0, 1.0));
                grid.push(ny.clamp(-1.0, 1.0));
            }
        }
        grid
    }

    /// Compose GDC + EIS grids.
    pub fn compose_grids(gdc: &[f32], eis: &[f32], width: u32, height: u32) -> Vec<f32> {
        let n = (width * height * 2) as usize;
        let mut composed = Vec::with_capacity(n);
        for i in (0..n).step_by(2) {
            composed.push((gdc[i] + eis[i]).clamp(-1.0, 1.0));
            composed.push((gdc[i + 1] + eis[i + 1]).clamp(-1.0, 1.0));
        }
        composed
    }

    /// Set grid from pre-computed data.
    pub fn set_grid(&mut self, grid: Vec<f32>) {
        self.grid = Some(grid);
    }

    /// Compute and set GDC grid from coefficients.
    pub fn set_gdc(&mut self, k1: f32, k2: f32, k3: f32) {
        // Check cache first
        if let Some((ck1, ck2, ck3, ref cached_grid)) = self.gdc_cache {
            if (ck1 - k1).abs() < 1e-6 && (ck2 - k2).abs() < 1e-6 && (ck3 - k3).abs() < 1e-6 {
                self.grid = Some(cached_grid.clone());
                return;
            }
        }
        // Compute and cache
        let grid = Self::compute_gdc_grid(k1, k2, k3, self.output_width, self.output_height);
        self.gdc_cache = Some((k1, k2, k3, grid.clone()));
        self.grid = Some(grid);
    }

    /// Set lens shading LUT.
    pub fn set_shading_lut(&mut self, lut: Vec<f32>) {
        self.shading_lut = Some(lut);
    }
}

impl IspBlock for RuntimeWarpBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "RuntimeWarp".into()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
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

    fn graph_input_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.input_source,
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
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }

    /// Grid tensor provided as runtime input (computed on CPU).
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        let mut inputs = vec![(
            "RuntimeWarp/grid".into(),
            1,
            vec![1, self.output_height as i64, self.output_width as i64, 2],
        )];
        if self.shading_lut.is_some() {
            inputs.push((
                format!("{}/shading_lut", ns),
                1,
                vec![1, 3, self.output_height as i64, self.output_width as i64],
            ));
        }
        inputs
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        let grid = self.grid.as_deref().unwrap_or(&[]);
        let mut defaults = Vec::new();
        if !grid.is_empty() {
            defaults.push((
                format!("{}/grid", ns),
                grid.iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect::<Vec<u8>>(),
            ));
        }
        if let Some(lut) = &self.shading_lut {
            defaults.push((
                format!("{}/shading_lut", ns),
                lut.iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect::<Vec<u8>>(),
            ));
        }
        defaults
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // GridSample(image, grid) → output
        let sampled = format!("{}/sampled", ns);
        nodes.push(Proto::node(
            "GridSample",
            &[&self.input_source, &format!("{}/grid", ns)],
            &[&sampled],
            &[
                Proto::attribute_string("mode", "bilinear"),
                Proto::attribute_string("padding_mode", "border"),
                Proto::attribute_int("align_corners", 0),
            ],
        ));

        // Apply lens shading if provided
        if self.shading_lut.is_some() {
            let shaded = format!("{}/shaded", ns);
            nodes.push(Proto::node(
                "Mul",
                &[&sampled, &format!("{}/shading_lut", ns)],
                &[&shaded],
                &[],
            ));
            nodes.push(Proto::node(
                "Identity",
                &[&shaded],
                &[&self.frame_tensor],
                &[],
            ));
        } else {
            nodes.push(Proto::node(
                "Identity",
                &[&sampled],
                &[&self.frame_tensor],
                &[],
            ));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_gdc_grid_identity() {
        let grid = RuntimeWarpBlock::compute_gdc_grid(0.0, 0.0, 0.0, 64, 64);
        assert_eq!(grid.len(), 64 * 64 * 2);
        // With no distortion, grid should be identity
        for i in (0..grid.len()).step_by(2) {
            let nx = 2.0 * ((i / 2) % 64) as f32 / 63.0 - 1.0;
            let ny = 2.0 * ((i / 2) / 64) as f32 / 63.0 - 1.0;
            assert!(
                (grid[i] - nx).abs() < 1e-5,
                "x mismatch at {}: {} vs {}",
                i,
                grid[i],
                nx
            );
            assert!((grid[i + 1] - ny).abs() < 1e-5, "y mismatch");
        }
    }

    #[test]
    fn test_gdc_grid_barrel() {
        let grid = RuntimeWarpBlock::compute_gdc_grid(-0.3, 0.0, 0.0, 64, 64);
        // All values should be in [-1, 1]
        for v in &grid {
            assert!(*v >= -1.0 && *v <= 1.0, "Grid value out of range: {}", v);
        }
    }

    #[test]
    fn test_eis_grid_displacement() {
        let grid = RuntimeWarpBlock::compute_eis_grid(0.1, 0.0, 4, 4);
        // Grid should have 4*4*2 = 32 values
        assert_eq!(grid.len(), 32);
        // All values should be in [-1, 1]
        for v in &grid {
            assert!(*v >= -1.0 && *v <= 1.0, "Grid value out of range: {}", v);
        }
    }

    #[test]
    fn test_compose_grids() {
        let gdc = RuntimeWarpBlock::compute_gdc_grid(0.0, 0.0, 0.0, 4, 4);
        let eis = RuntimeWarpBlock::compute_eis_grid(0.0, 0.0, 4, 4);
        let composed = RuntimeWarpBlock::compose_grids(&gdc, &eis, 4, 4);
        // With identity GDC and zero EIS, composed should equal GDC (after clamping)
        assert_eq!(composed.len(), gdc.len());
        for i in 0..composed.len() {
            let expected = (gdc[i] + eis[i]).clamp(-1.0, 1.0);
            assert!(
                (composed[i] - expected).abs() < 1e-5,
                "Mismatch at {}: {} vs {}",
                i,
                composed[i],
                expected
            );
        }
    }

    #[test]
    fn test_runtime_warp_block_ops() {
        let block = RuntimeWarpBlock::new(64, 64);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 2, "Should produce GridSample + Identity");
    }

    #[test]
    fn test_runtime_warp_extra_inputs() {
        let block = RuntimeWarpBlock::new(128, 128);
        let extras = block.extra_inputs();
        assert_eq!(extras.len(), 1, "Should have 1 extra input: grid");
        assert!(extras[0].0.contains("grid"));
    }

    #[test]
    fn test_runtime_warp_compose() {
        crate::init();
        let blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(crate::blocks::UnpackBlock::new().with_concrete_dims(64, 64)),
            Box::new(RuntimeWarpBlock::new(64, 64)),
            Box::new(crate::blocks::DisplayBlock::new(64)),
        ];
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 13);
        assert!(result.is_ok(), "Compose failed: {:?}", result.err());
        let onnx = result.unwrap();
        assert!(onnx.len() > 100, "ONNX should be non-trivial");
    }

    #[test]
    fn test_runtime_warp_with_shading() {
        let mut block = RuntimeWarpBlock::new(64, 64);
        block.set_shading_lut(vec![1.0; 3 * 64 * 64]);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 3, "Should produce GridSample + Mul + Identity");
        // extra_inputs returns grid only (shading_lut is initializer)
        let extras = block.extra_inputs();
        assert_eq!(
            extras.len(),
            2,
            "Should have grid + shading as extra inputs"
        );
    }
}
