//! Full ISP pipeline with all correction blocks.
//!
//! Demonstrates: Unpack → DemosaicCcm → WarpGrid(GDC+LensShading) →
//!   ChromaticAberration → AutoContrast → TemporalDenoise → Display
//!
//! Usage:
//!   cargo run --example full_pipeline -p cam-isp --features mnn

use cam_isp::blocks::*;
use cam_isp::pipeline::GraphComposer;

fn main() {
    println!("=== Full ISP Pipeline ===\n");

    let w: u32 = 1920;
    let h: u32 = 1080;

    // 1. Unpack: INT16 Bayer → float RGB
    let unpack = UnpackBlock::new().with_concrete_dims(h as i64, w as i64);

    // 2. Demosaic + CCM in one fused block
    let demosaic = DemosaicCcmBlock::new(0);

    // 3. WarpGrid: GDC correction + lens shading in one pass
    let warp = WarpGridBlock::new(w, h)
        .with_gdc(-0.15, 0.05, 0.0) // mild barrel correction
        .with_lens_shading(1.3, 1.0); // 30% corner boost

    // 4. Chromatic aberration correction
    let ca = ChromaticAberrationBlock::new().with_radial_correction(h, w, 1.5); // 1.5px shift at corners

    // 5. Auto contrast: S-curve + shadow lift
    let contrast = AutoContrastBlock::new(1.3).with_shadow_lift(0.02);

    // 6. Temporal denoise: blend with previous frame
    let denoise = TemporalDenoiseBlock::new()
        .with_threshold(0.05)
        .with_blend_weight(0.6);

    // 7. Display: RGB float output
    let display = DisplayBlock::new(w);

    // Compose ONNX
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![
        &unpack, &demosaic, &warp, &ca, &contrast, &denoise, &display,
    ];

    let onnx_bytes = GraphComposer::compose_from_vec(&blocks, &[], 8).expect("compose failed");

    std::fs::write("target/full_pipeline.onnx", &onnx_bytes).unwrap();

    println!("✅ Full pipeline ONNX: {} bytes", onnx_bytes.len());
    println!("\nPipeline stages:");
    println!("  1. Unpack({}×{})       — INT16 Bayer → float", w, h);
    println!("  2. DemosaicCcm         — Bayer → RGB + color correction");
    println!("  3. WarpGrid            — GDC(k1=-0.15) + LensShading(1.3×)");
    println!("  4. ChromaticAberration — 1.5px radial correction");
    println!("  5. AutoContrast        — S-curve(1.3) + shadow lift(0.02)");
    println!("  6. TemporalDenoise     — threshold=0.05, blend=0.6");
    println!("  7. Display             — RGB float output");
    println!("\nSaved: target/full_pipeline.onnx");
}
