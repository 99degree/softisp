//! # GpuWarpBlock — Fully GPU-Accelerated Warp Pipeline
//!
//! ## Overview
//!
//! Computes GDC grid entirely on GPU using ONNX ops, then applies
//! GridSample. Parameters (k1, k2, k3, EIS displacement) are
//! provided as runtime ONNX inputs.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    GPU Warp Pipeline                           │
//! ├─────────────────────────────────────────────────────────────────┤
//! │                                                                 │
//! │  Runtime Inputs: k1, k2, k3, eis_dx, eis_dy                   │
//! │       ↓                                                         │
//! │  GPU: Compute GDC grid from distortion coefficients             │
//! │       ↓                                                         │
//! │  GPU: Compose with EIS displacement grid                        │
//! │       ↓                                                         │
//! │  GPU: GridSample(image, composed_grid) → warped output          │
//! │                                                                 │
//! │  All operations run on GPU — zero CPU involvement.             │
//! │                                                                 │
//! └─────────────────────────────────────────────────────────────────┘
//!
//! ## Deshake Integration
//!
//! For GPU-accelerated deshake, this block can be used with
//! `DeshakeGpuPipeline` which runs the entire stabilization
//! pipeline (grayscale → pyramid → motion estimation → warp) on GPU.
//!
//! ## Comparison with Other Warp Blocks
//!
//! | Feature | WarpGridBlock | RuntimeWarpBlock | GpuWarpBlock |
//! |---------|---------------|------------------|--------------|
//! | Grid source | Pre-computed | CPU (per-frame) | GPU (ONNX ops) |
//! | GridSample | GPU | GPU | GPU |
//! | Latency | ~0.5ms | ~1ms CPU + GPU | GPU only |
//! | Flexibility | High | High | Fixed GDC model |
//! | Lens shading | Yes (fused) | No | No |
//! | Rotation/Flip | Yes | No | No |

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// GpuWarpBlock — fully GPU-accelerated grid computation + warp.
pub struct GpuWarpBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub output_width: u32,
    pub output_height: u32,
}

impl GpuWarpBlock {
    pub fn new(target_width: u32, target_height: u32) -> Self {
        Self {
            id: "gpu_warp".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "GpuWarp/frame".into(),
            input_source: String::new(),
            output_width: target_width,
            output_height: target_height,
        }
    }

    /// Generate ONNX nodes for GPU grid computation + GridSample.
    ///
    /// Grid computation:
    ///   1. Broadcast grid_x, grid_y (normalized coords) to [1,1,H,W]
    ///   2. Compute r² = grid_x² + grid_y²
    ///   3. Compute r⁴, r⁶
    ///   4. denom = 1 + k1*r² + k2*r⁴ + k3*r⁶
    ///   5. inv_denom = 1 / denom
    ///   6. grid_x_gdc = grid_x * inv_denom
    ///   7. grid_y_gdc = grid_y * inv_denom
    ///   8. Compose with EIS: final = gdc + eis
    ///   9. Concat X,Y → grid [1,H,W,2]
    fn generate_grid_nodes(&self, ns: &str) -> Vec<Vec<u8>> {
        let mut nodes = Vec::new();
        let _h = self.output_height as i64;
        let _w = self.output_width as i64;

        // ── Step 1: Broadcast normalized coords ──

        // grid_x [1,1,1,W] + zero [1,1,1,W] → grid_x_2d [1,1,1,W]
        // Then reshape to [1,1,H,W] via broadcasting in subsequent ops
        let grid_x_2d = format!("{}/gx2d", ns);
        nodes.push(Proto::node(
            "Add",
            &[&format!("{}/grid_x", ns), &format!("{}/zero", ns)],
            &[&grid_x_2d],
            &[],
        ));

        let grid_y_2d = format!("{}/gy2d", ns);
        nodes.push(Proto::node(
            "Add",
            &[&format!("{}/grid_y", ns), &format!("{}/zero", ns)],
            &[&grid_y_2d],
            &[],
        ));

        // ── Step 2: Compute r² = grid_x² + grid_y² ──

        let gx_sq = format!("{}/gx_sq", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&grid_x_2d, &grid_x_2d],
            &[&gx_sq],
            &[],
        ));

        let gy_sq = format!("{}/gy_sq", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&grid_y_2d, &grid_y_2d],
            &[&gy_sq],
            &[],
        ));

        let r2 = format!("{}/r2", ns);
        nodes.push(Proto::node("Add", &[&gx_sq, &gy_sq], &[&r2], &[]));

        // ── Step 3: Compute r⁴, r⁶ ──

        let r4 = format!("{}/r4", ns);
        nodes.push(Proto::node("Mul", &[&r2, &r2], &[&r4], &[]));

        let r6 = format!("{}/r6", ns);
        nodes.push(Proto::node("Mul", &[&r4, &r2], &[&r6], &[]));

        // ── Step 4: denom = 1 + k1*r² + k2*r⁴ + k3*r⁶ ──

        // k1*r²
        let k1r2 = format!("{}/k1r2", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&format!("{}/gdc_k1", ns), &r2],
            &[&k1r2],
            &[],
        ));

        // k2*r⁴
        let k2r4 = format!("{}/k2r4", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&format!("{}/gdc_k2", ns), &r4],
            &[&k2r4],
            &[],
        ));

        // k3*r⁶
        let k3r6 = format!("{}/k3r6", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&format!("{}/gdc_k3", ns), &r6],
            &[&k3r6],
            &[],
        ));

        // 1 + k1r2
        let t1 = format!("{}/t1", ns);
        nodes.push(Proto::node(
            "Add",
            &[&format!("{}/one", ns), &k1r2],
            &[&t1],
            &[],
        ));

        // t1 + k2r4
        let t2 = format!("{}/t2", ns);
        nodes.push(Proto::node("Add", &[&t1, &k2r4], &[&t2], &[]));

        // t2 + k3r6 = denom
        let denom = format!("{}/denom", ns);
        nodes.push(Proto::node("Add", &[&t2, &k3r6], &[&denom], &[]));

        // ── Step 5: inv_denom = 1 / denom ──

        let inv = format!("{}/inv", ns);
        nodes.push(Proto::node(
            "Div",
            &[&format!("{}/one", ns), &denom],
            &[&inv],
            &[],
        ));

        // ── Step 6-7: Apply GDC ──

        let gx_gdc = format!("{}/gx_gdc", ns);
        nodes.push(Proto::node("Mul", &[&grid_x_2d, &inv], &[&gx_gdc], &[]));

        let gy_gdc = format!("{}/gy_gdc", ns);
        nodes.push(Proto::node("Mul", &[&grid_y_2d, &inv], &[&gy_gdc], &[]));

        // ── Step 8: Compose with EIS displacement ──

        let final_x = format!("{}/fx", ns);
        nodes.push(Proto::node(
            "Add",
            &[&gx_gdc, &format!("{}/eis_x", ns)],
            &[&final_x],
            &[],
        ));

        let final_y = format!("{}/fy", ns);
        nodes.push(Proto::node(
            "Add",
            &[&gy_gdc, &format!("{}/eis_y", ns)],
            &[&final_y],
            &[],
        ));

        // ── Step 9: Concat X,Y → [1,1,H,W,2] → reshape [1,H,W,2] ──

        let grid_5d = format!("{}/g5d", ns);
        nodes.push(Proto::node(
            "Concat",
            &[&final_x, &final_y],
            &[&grid_5d],
            &[Proto::attribute_int("axis", -1)],
        ));

        let grid_4d = format!("{}/grid", ns);
        nodes.push(Proto::node(
            "Reshape",
            &[&grid_5d, &format!("{}/shape4", ns)],
            &[&grid_4d],
            &[],
        ));

        // ── Step 10: GridSample ──

        let sampled = format!("{}/sampled", ns);
        nodes.push(Proto::node(
            "GridSample",
            &[&self.input_source, &grid_4d],
            &[&sampled],
            &[
                Proto::attribute_string("mode", "bilinear"),
                Proto::attribute_string("padding_mode", "border"),
                Proto::attribute_int("align_corners", 0),
            ],
        ));

        nodes.push(Proto::node(
            "Identity",
            &[&sampled],
            &[&self.frame_tensor],
            &[],
        ));

        nodes
    }
}

impl IspBlock for GpuWarpBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "GpuWarp".into()
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

    /// Runtime inputs: GDC coefficients + EIS displacement.
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![
            ("GpuWarp/gdc_k1".into(), 1, vec![1]),
            ("GpuWarp/gdc_k2".into(), 1, vec![1]),
            ("GpuWarp/gdc_k3".into(), 1, vec![1]),
            (
                "GpuWarp/eis_x".into(),
                1,
                vec![1, 1, self.output_height as i64, self.output_width as i64],
            ),
            (
                "GpuWarp/eis_y".into(),
                1,
                vec![1, 1, self.output_height as i64, self.output_width as i64],
            ),
        ]
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        self.generate_grid_nodes(&self.tensor_ns())
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let h = self.output_height as usize;
        let w = self.output_width as usize;

        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
            Proto::tensor_proto_int64(&format!("{}/shape4", ns), &[1, h as i64, w as i64, 2]),
            Proto::tensor_proto_float(
                &format!("{}/grid_x", ns),
                &[1, 1, 1, w as i64],
                &(0..w)
                    .map(|x| 2.0 * x as f32 / (w - 1) as f32 - 1.0)
                    .collect::<Vec<_>>(),
            ),
            Proto::tensor_proto_float(
                &format!("{}/grid_y", ns),
                &[1, 1, h as i64, 1],
                &(0..h)
                    .map(|y| 2.0 * y as f32 / (h - 1) as f32 - 1.0)
                    .collect::<Vec<_>>(),
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_gpu_warp_nodes() {
        let block = GpuWarpBlock::new(64, 64);
        let nodes = block.nodes();
        // grid compute (many ops) + GridSample + Identity
        assert!(
            nodes.len() >= 15,
            "Should produce many nodes for GPU grid compute"
        );
    }

    #[test]
    fn test_gpu_warp_extra_inputs() {
        let block = GpuWarpBlock::new(128, 128);
        let extras = block.extra_inputs();
        assert_eq!(extras.len(), 5, "Should have k1,k2,k3,eis_x,eis_y");
        assert!(extras[0].0.contains("gdc_k1"));
        assert!(extras[3].0.contains("eis_x"));
        assert!(extras[4].0.contains("eis_y"));
    }

    #[test]
    fn test_gpu_warp_compose() {
        crate::init();
        let blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(crate::blocks::UnpackBlock::new().with_concrete_dims(64, 64)),
            Box::new(GpuWarpBlock::new(64, 64)),
            Box::new(crate::blocks::DisplayBlock::new(64)),
        ];
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 13);
        assert!(result.is_ok(), "Compose failed: {:?}", result.err());
        let onnx = result.unwrap();
        assert!(onnx.len() > 100, "ONNX should be non-trivial");
    }

    #[test]
    fn test_gpu_vs_cpu_warp_same_graph_input() {
        let gpu = GpuWarpBlock::new(64, 64);
        let cpu = crate::blocks::runtime_warp::RuntimeWarpBlock::new(64, 64);
        // Both should produce valid frame tensor names
        assert!(gpu.graph_input_name().is_some());
        assert!(cpu.graph_input_name().is_some());
        // Both should output to same-sized tensor
        assert_eq!(gpu.output_width, cpu.output_width);
        assert_eq!(gpu.output_height, cpu.output_height);
    }
}
