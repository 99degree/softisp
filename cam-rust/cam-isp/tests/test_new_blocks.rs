//! Integration tests for all new ISP blocks.
//! Tests ONNX emission correctness, shape validation, and block composition.

use cam_isp::blocks::*;
use cam_isp::pipeline::IspBlock;

// ── WarpGridBlock ──────────────────────────────────────────

#[test]
fn warp_grid_emits_grid_sample_node() {
    let block = WarpGridBlock::new(1920, 1080);
    let nodes = block.nodes();
    let has_grid_sample = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("GridSample")
    });
    assert!(has_grid_sample, "WarpGridBlock must emit GridSample");
}

#[test]
fn warp_grid_with_gdc_emits_extra_grid_sample() {
    let block = WarpGridBlock::new(640, 480)
        .with_gdc(-0.2, 0.0, 0.0);
    let nodes = block.nodes();
    let has_grid = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("GridSample")
    });
    assert!(has_grid, "GDC must emit GridSample");
}

#[test]
fn warp_grid_with_lens_shading_emits_mul() {
    let block = WarpGridBlock::new(640, 480)
        .with_lens_shading(1.5, 1.0);
    let nodes = block.nodes();
    let has_mul = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Mul")
    });
    assert!(has_mul, "Lens shading must emit Mul");
}

#[test]
fn warp_grid_with_rotation_emits_transpose() {
    let block = WarpGridBlock::new(640, 480)
        .with_rotate(1); // 90° CW
    let nodes = block.nodes();
    let has_transpose = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Transpose")
    });
    assert!(has_transpose, "Rotation must emit Transpose");
}

#[test]
fn warp_grid_gdc_and_eis_composes_grids() {
    let eis_grid = vec![0.0f32; 64 * 48 * 2];
    let block = WarpGridBlock::new(64, 48)
        .with_gdc_and_eis(-0.1, 0.0, 0.0, eis_grid);
    let nodes = block.nodes();
    let count = nodes.iter().filter(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("GridSample")
    }).count();
    assert!(count >= 1, "GDC+EIS must emit at least 1 GridSample, got {}", count);
}

#[test]
fn warp_grid_produces_valid_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let warp = WarpGridBlock::new(640, 480)
        .with_gdc(-0.15, 0.05, 0.0)
        .with_lens_shading(1.3, 1.0);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &warp, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 100, "valid ONNX produced");
    std::fs::write("target/warp_grid_test.onnx", &onnx).ok();
}

// ── ChromaticAberrationBlock ──────────────────────────────

#[test]
fn ca_emits_split_and_concat() {
    let block = ChromaticAberrationBlock::new()
        .with_radial_correction(1080, 1920, 1.5);
    let nodes = block.nodes();
    let has_split = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Split")
    });
    let has_concat = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Concat")
    });
    assert!(has_split, "CA must emit Split");
    assert!(has_concat, "CA must emit Concat");
}

#[test]
fn ca_emits_three_grid_samples() {
    let block = ChromaticAberrationBlock::new()
        .with_radial_correction(1080, 1920, 2.0);
    let nodes = block.nodes();
    let count = nodes.iter().filter(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("GridSample")
    }).count();
    assert_eq!(count, 3, "CA must emit exactly 3 GridSamples (R, G, B), got {}", count);
}

#[test]
fn ca_has_one_input_one_output() {
    let block = ChromaticAberrationBlock::new()
        .with_radial_correction(480, 640, 1.0);
    assert_eq!(block.input_tensors().len(), 1);
    assert_eq!(block.output_tensors().len(), 1);
}

#[test]
fn ca_produces_valid_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let ca = ChromaticAberrationBlock::new()
        .with_radial_correction(480, 640, 1.5);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &ca, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 100);
}

// ── AutoContrastBlock ─────────────────────────────────────

#[test]
fn auto_contrast_emits_mul_add_nodes() {
    let block = AutoContrastBlock::new(1.5);
    let nodes = block.nodes();
    let has_mul = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Mul")
    });
    let has_add = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Add")
    });
    assert!(has_mul, "AutoContrast must emit Mul");
    assert!(has_add, "AutoContrast must emit Add");
}

#[test]
fn auto_contrast_with_shadow_lift() {
    let block = AutoContrastBlock::new(1.3)
        .with_shadow_lift(0.03);
    let nodes = block.nodes();
    // Should have Mul + Add for contrast, then Add + Mul for shadow lift
    let add_count = nodes.iter().filter(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Add")
    }).count();
    assert!(add_count >= 2, "shadow lift needs 2+ Add ops, got {}", add_count);
}

#[test]
fn auto_contrast_has_initializers() {
    let block = AutoContrastBlock::new(1.2);
    let inits = block.initializers();
    assert!(inits.len() >= 1, "need at least 1 initializer, got {}", inits.len());
}

// ── TemporalDenoiseBlock ──────────────────────────────────

#[test]
fn temporal_denoise_has_two_inputs() {
    let block = TemporalDenoiseBlock::new();
    let inputs = block.input_tensors();
    assert_eq!(inputs.len(), 2, "must have 2 inputs (current + prev)");
}

#[test]
fn temporal_denoise_emits_sub_abs_less() {
    let block = TemporalDenoiseBlock::new();
    let nodes = block.nodes();
    let has_sub = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Sub")
    });
    let has_abs = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Abs")
    });
    let has_less = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Less")
    });
    assert!(has_sub, "must emit Sub");
    assert!(has_abs, "must emit Abs");
    assert!(has_less, "must emit Less");
}

#[test]
fn temporal_denoise_threshold_in_init() {
    let block = TemporalDenoiseBlock::new().with_threshold(0.08);
    let inits = block.initializers();
    assert_eq!(inits.len(), 4, "need threshold + one + blend_w + curr_weight");
}

// ── Composition tests ─────────────────────────────────────

#[test]
fn full_correction_pipeline_produces_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let demosaic = DemosaicCcmBlock::new(0);
    let warp = WarpGridBlock::new(640, 480)
        .with_gdc(-0.1, 0.0, 0.0)
        .with_lens_shading(1.3, 1.0);
    let ca = ChromaticAberrationBlock::new()
        .with_radial_correction(480, 640, 1.5);
    let contrast = AutoContrastBlock::new(1.3)
        .with_shadow_lift(0.02);
    let denoise = TemporalDenoiseBlock::new();
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![
        &unpack, &demosaic, &warp, &ca, &contrast, &denoise, &display,
    ];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 8).unwrap();
    assert!(onnx.len() > 500, "full pipeline ONNX: {} bytes", onnx.len());
    std::fs::write("target/full_correction_test.onnx", &onnx).ok();
    println!("Full correction pipeline: {} bytes", onnx.len());
}

#[test]
fn warp_and_ca_compose() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let warp = WarpGridBlock::new(640, 480)
        .with_gdc(-0.2, 0.0, 0.0);
    let ca = ChromaticAberrationBlock::new()
        .with_radial_correction(480, 640, 1.0);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &warp, &ca, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 200);
}
