//! WarpGrid ONNX emission test.
//! Builds: Unpack → WarpGrid (identity grid) → Display → ONNX file.
//!
//! Usage: cargo run --example warp_onnx -p cam-isp --features mnn

use cam_isp::blocks::*;
use cam_isp::pipeline::GraphComposer;

fn main() {
    println!("=== WarpGrid ONNX Emission Test ===\n");

    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let warp = WarpGridBlock::new(640, 480)
        .with_grid(Some(identity_grid(480, 640)));
    let display = DisplayBlock::new(640);

    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &warp, &display];
    let onnx_bytes = GraphComposer::compose_from_vec(&blocks, &[], 8)
        .expect("compose failed");

    std::fs::write("target/warp_grid_test.onnx", &onnx_bytes).unwrap();
    println!("✅ ONNX emitted: {} bytes", onnx_bytes.len());
    println!("   Pipeline: Unpack(640×480) → WarpGrid(GridSample) → Display");
    println!("   Grid: identity (640×480×2 float32)");
}

fn identity_grid(h: u32, w: u32) -> Vec<f32> {
    let mut grid = Vec::with_capacity((h * w * 2) as usize);
    for y in 0..h {
        for x in 0..w {
            let gx = 2.0 * x as f32 / (w - 1) as f32 - 1.0;
            let gy = 2.0 * y as f32 / (h - 1) as f32 - 1.0;
            grid.push(gx);
            grid.push(gy);
        }
    }
    grid
}
