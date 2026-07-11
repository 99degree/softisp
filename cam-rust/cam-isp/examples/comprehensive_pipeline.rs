//! Comprehensive pipeline: every ISP block type in a single graph.
//!
//! Demonstrates the full diversity of the ISP block library:
//! Raw → Unpack → DemosaicCcm → BLC → WhiteBalance → CCM →
//! ColorSpace(RGB→YUV) → Sharpen → AutoContrast → Gamma →
//! WarpGrid(GDC+LensShading) → ChromaticAberration → TemporalDenoise →
//! Display
//!
//! Usage:
//!   cargo run --example comprehensive_pipeline -p cam-isp --features mnn

use cam_isp::blocks::*;
use cam_isp::pipeline::{IspBlock, GraphComposer};

fn main() {
    println!("=== Comprehensive ISP Pipeline ===\n");

    let w: u32 = 1920;
    let h: u32 = 1080;
    let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();

    // 1. Raw INT16 Bayer input
    blocks.push(Box::new(UnpackBlock::new()
        .with_concrete_dims(h as i64, w as i64)));
    println!(" 1. UnpackBlock       — INT16 Bayer → float RGB");

    // 2. Demosaic + Color Correction Matrix
    blocks.push(Box::new(DemosaicCcmBlock::new(0)));
    println!(" 2. DemosaicCcmBlock  — Bayer demosaic + 3×3 CCM");

    // 3. Gamma (sRGB)
    blocks.push(Box::new(GammaBlock::new(2.2)
        .with_shadow_lift(0.02)));
    println!(" 3. GammaBlock        — sRGB gamma (2.2) + shadow lift");

    // 4. Sharpen
    blocks.push(Box::new(SharpenBlock::new(0.5)));
    println!(" 4. SharpenBlock      — Unsharp mask (strength=0.5)");

    // 5. Auto contrast
    blocks.push(Box::new(AutoContrastBlock::new(1.3)
        .with_shadow_lift(0.02)));
    println!(" 5. AutoContrastBlock — S-curve (1.3) + shadow lift");

    // 6. Warp: GDC + lens shading
    blocks.push(Box::new(WarpGridBlock::new(w, h)
        .with_gdc(-0.15, 0.05, 0.0)
        .with_lens_shading(1.3, 1.0)));
    println!(" 6. WarpGridBlock     — GDC(k1=-0.15) + LensShading(1.3×)");

    // 7. Chromatic aberration
    blocks.push(Box::new(ChromaticAberrationBlock::new()
        .with_radial_correction(h, w, 1.5)));
    println!(" 7. ChromaticAberration — 1.5px radial correction");

    // 8. Temporal denoise
    blocks.push(Box::new(TemporalDenoiseBlock::new()
        .with_threshold(0.05)
        .with_blend_weight(0.6)));
    println!(" 8. TemporalDenoise   — threshold=0.05, blend=0.6");

    // 9. Display output
    blocks.push(Box::new(DisplayBlock::new(w)));
    println!(" 9. DisplayBlock      — RGB float output");

    // Compose ONNX
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let t0 = std::time::Instant::now();
    let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    let emit_ms = t0.elapsed().as_secs_f64() * 1000.0;

    let path = "target/comprehensive_pipeline.onnx";
    std::fs::write(path, &onnx).unwrap();

    println!("\n✅ Pipeline: {} stages → {} bytes ONNX ({:.2} ms)",
        block_refs.len(), onnx.len(), emit_ms);
    println!("   Saved: {}", path);

    // Also test with ColorSpace (YUV conversion)
    println!("\n--- Variant with YUV conversion ---");
    let blocks2: Vec<Box<dyn IspBlock>> = vec![
        Box::new(UnpackBlock::new()
            .with_concrete_dims(h as i64, w as i64)),
        Box::new(DemosaicCcmBlock::new(0)),
        Box::new(ColorSpaceBlock::new(ColorSpace::RgbToYCbCr)),
        Box::new(SharpenBlock::new(0.3)),
        Box::new(ColorSpaceBlock::new(ColorSpace::RgbToHsv)),
        Box::new(DisplayBlock::new(w)),
    ];

    let refs2: Vec<&dyn IspBlock> = blocks2.iter().map(|b| b.as_ref()).collect();
    let onnx2 = GraphComposer::compose_from_vec(&refs2, &[], 8).unwrap();
    println!("✅ YUV variant: {} stages → {} bytes", refs2.len(), onnx2.len());
    std::fs::write("target/comprehensive_yuv.onnx", &onnx2).unwrap();
}
