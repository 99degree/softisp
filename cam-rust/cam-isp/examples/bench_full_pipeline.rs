//! Benchmark: full ISP pipeline ONNX emission + MNN convert + Vulkan inference.
//!
//! Tests HD (1280×720), FHD (1920×1080), and 4K (3840×2160) pipelines
//! with all correction blocks enabled.
//!
//! Usage:
//!   cargo run --example bench_full_pipeline -p cam-isp --features mnn

use cam_isp::blocks::*;
use cam_isp::pipeline::IspBlock;

fn build_pipeline(w: u32, h: u32, blocks: &mut Vec<Box<dyn IspBlock>>) {
    // Unpack: INT16 Bayer → float RGB
    blocks.push(Box::new(
        UnpackBlock::new().with_concrete_dims(h as i64, w as i64),
    ));

    // Demosaic + CCM
    blocks.push(Box::new(DemosaicCcmBlock::new(0)));

    // Warp: GDC + lens shading
    blocks.push(Box::new(
        WarpGridBlock::new(w, h)
            .with_gdc(-0.15, 0.05, 0.0)
            .with_lens_shading(1.3, 1.0),
    ));

    // Chromatic aberration
    blocks.push(Box::new(
        ChromaticAberrationBlock::new().with_radial_correction(h, w, 1.5),
    ));

    // Auto contrast
    blocks.push(Box::new(AutoContrastBlock::new(1.3).with_shadow_lift(0.02)));

    // Display output
    blocks.push(Box::new(DisplayBlock::new(w)));
}

fn bench_resolution(name: &str, w: u32, h: u32) {
    let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();
    build_pipeline(w, h, &mut blocks);

    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let stages = block_refs.len();

    let t0 = std::time::Instant::now();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    let emit_ms = t0.elapsed().as_secs_f64() * 1000.0;

    let path = format!("target/bench_{}.onnx", name);
    std::fs::write(&path, &onnx).ok();

    println!("{} ({}×{}):", name, w, h);
    println!("  Stages: {}", stages);
    println!("  ONNX:   {:.1} KB", onnx.len() as f64 / 1024.0);
    println!("  Emit:   {:.2} ms", emit_ms);
}

fn main() {
    println!("=== Full Pipeline Benchmark ===\n");
    bench_resolution("HD", 1280, 720);
    bench_resolution("FHD", 1920, 1080);
    bench_resolution("4K", 3840, 2160);
    println!("\nONNX files saved to target/");
}
