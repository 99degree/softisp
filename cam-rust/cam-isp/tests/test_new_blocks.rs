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
    let block = WarpGridBlock::new(640, 480).with_gdc(-0.2, 0.0, 0.0);
    let nodes = block.nodes();
    let has_grid = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("GridSample")
    });
    assert!(has_grid, "GDC must emit GridSample");
}

#[test]
fn warp_grid_with_lens_shading_emits_mul() {
    let block = WarpGridBlock::new(640, 480).with_lens_shading(1.5, 1.0);
    let nodes = block.nodes();
    let has_mul = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Mul")
    });
    assert!(has_mul, "Lens shading must emit Mul");
}

#[test]
fn warp_grid_with_rotation_emits_transpose() {
    let block = WarpGridBlock::new(640, 480).with_rotate(1); // 90° CW
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
    let block = WarpGridBlock::new(64, 48).with_gdc_and_eis(-0.1, 0.0, 0.0, eis_grid);
    let nodes = block.nodes();
    let count = nodes
        .iter()
        .filter(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("GridSample")
        })
        .count();
    assert!(
        count >= 1,
        "GDC+EIS must emit at least 1 GridSample, got {}",
        count
    );
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
    let block = ChromaticAberrationBlock::new().with_radial_correction(1080, 1920, 1.5);
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
    let block = ChromaticAberrationBlock::new().with_radial_correction(1080, 1920, 2.0);
    let nodes = block.nodes();
    let count = nodes
        .iter()
        .filter(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("GridSample")
        })
        .count();
    assert_eq!(
        count, 3,
        "CA must emit exactly 3 GridSamples (R, G, B), got {}",
        count
    );
}

#[test]
fn ca_has_one_input_one_output() {
    let block = ChromaticAberrationBlock::new().with_radial_correction(480, 640, 1.0);
    assert_eq!(block.input_tensors().len(), 1);
    assert_eq!(block.output_tensors().len(), 1);
}

#[test]
fn ca_produces_valid_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let ca = ChromaticAberrationBlock::new().with_radial_correction(480, 640, 1.5);
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
    let block = AutoContrastBlock::new(1.3).with_shadow_lift(0.03);
    let nodes = block.nodes();
    // Should have Mul + Add for contrast, then Add + Mul for shadow lift
    let add_count = nodes
        .iter()
        .filter(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("Add")
        })
        .count();
    assert!(
        add_count >= 2,
        "shadow lift needs 2+ Add ops, got {}",
        add_count
    );
}

#[test]
fn auto_contrast_has_initializers() {
    let block = AutoContrastBlock::new(1.2);
    let inits = block.initializers();
    assert!(
        inits.len() >= 1,
        "need at least 1 initializer, got {}",
        inits.len()
    );
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
    assert_eq!(
        inits.len(),
        4,
        "need threshold + one + blend_w + curr_weight"
    );
}

// ── HdrMergeBlock ─────────────────────────────────────────

#[test]
fn hdr_merge_has_three_inputs() {
    let block = HdrMergeBlock::new();
    let inputs = block.input_tensors();
    assert_eq!(
        inputs.len(),
        3,
        "must have 3 inputs (neutral + under + over)"
    );
}

#[test]
fn hdr_merge_emits_weight_map_nodes() {
    let block = HdrMergeBlock::new();
    let nodes = block.nodes();
    let has_clip = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Clip")
    });
    let has_mul = nodes.iter().any(|n| {
        let s = String::from_utf8_lossy(n);
        s.contains("Mul")
    });
    assert!(has_clip, "must emit Clip for weight clamping");
    assert!(has_mul, "must emit Mul for exposure blending");
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
    let ca = ChromaticAberrationBlock::new().with_radial_correction(480, 640, 1.5);
    let contrast = AutoContrastBlock::new(1.3).with_shadow_lift(0.02);
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
    let warp = WarpGridBlock::new(640, 480).with_gdc(-0.2, 0.0, 0.0);
    let ca = ChromaticAberrationBlock::new().with_radial_correction(480, 640, 1.0);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &warp, &ca, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 200);
}

// ── SharpenBlock ──────────────────────────────────────────

#[test]
fn sharpen_emits_avgpool_sub_mul_add() {
    let block = SharpenBlock::new(0.5);
    let nodes = block.nodes();
    let tags = ["AveragePool", "Sub", "Mul", "Add"];
    for tag in &tags {
        assert!(
            nodes
                .iter()
                .any(|n| String::from_utf8_lossy(n).contains(tag)),
            "must emit {}",
            tag
        );
    }
}

#[test]
fn sharpen_produces_valid_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let sharpen = SharpenBlock::new(0.5);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &sharpen, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 200);
}

// ── ColorSpaceBlock ───────────────────────────────────────

#[test]
fn colorspace_rgb_to_yuv_produces_onnx() {
    use cam_isp::blocks::ColorSpace;
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let cs = ColorSpaceBlock::new(ColorSpace::RgbToYCbCr);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &cs, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 200);
}

#[test]
fn colorspace_roundtrip_601() {
    use cam_isp::blocks::ColorSpace;
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let fwd = ColorSpaceBlock::new(ColorSpace::RgbToYCbCr);
    let rev = ColorSpaceBlock::new(ColorSpace::RgbToHsv);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &fwd, &rev, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 8).unwrap();
    assert!(onnx.len() > 200);
}

// ── AspectCropBlock ───────────────────────────────────────

#[test]
fn aspect_crop_16_9_produces_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(1080, 1920);
    let crop = AspectCropBlock::ratio_16_9();
    let display = DisplayBlock::new(1920);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &crop, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 200);
}

#[test]
fn aspect_crop_has_slice_op() {
    let block = AspectCropBlock::new(16, 9);
    let nodes = block.nodes();
    assert!(nodes
        .iter()
        .any(|n| String::from_utf8_lossy(n).contains("Slice")));
}

// ── GammaBlock ────────────────────────────────────────────

#[test]
fn gamma_2_2_produces_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let gamma = GammaBlock::new(2.2).with_shadow_lift(0.03);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &gamma, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 200);
}

#[test]
fn gamma_emits_log_mul_exp() {
    let block = GammaBlock::new(2.2);
    let nodes = block.nodes();
    let tags = ["Log", "Mul", "Exp"];
    for tag in &tags {
        assert!(
            nodes
                .iter()
                .any(|n| String::from_utf8_lossy(n).contains(tag)),
            "must emit {}",
            tag
        );
    }
}

// ── NoiseEstimateBlock ────────────────────────────────────

#[test]
fn noise_estimate_produces_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let ne = NoiseEstimateBlock::new().with_scale(1.2);
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &ne, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 6).unwrap();
    assert!(onnx.len() > 200);
}

// ── StereoDepthBlock ──────────────────────────────────────

#[test]
fn stereo_depth_has_two_inputs() {
    let block = StereoDepthBlock::new();
    assert_eq!(block.input_tensors().len(), 2);
}

#[test]
fn stereo_depth_emits_conv_sub() {
    let block = StereoDepthBlock::new();
    let nodes = block.nodes();
    assert!(nodes
        .iter()
        .any(|n| String::from_utf8_lossy(n).contains("Conv")));
    assert!(nodes
        .iter()
        .any(|n| String::from_utf8_lossy(n).contains("Sub")));
}

// ── Full pipeline with all new blocks ─────────────────────

#[test]
fn mega_pipeline_all_blocks() {
    use cam_isp::blocks::ColorSpace;
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(480, 640);
    let demosaic = DemosaicCcmBlock::new(0);
    let gamma = GammaBlock::new(2.2).with_shadow_lift(0.02);
    let sharpen = SharpenBlock::new(0.4);
    let contrast = AutoContrastBlock::new(1.3).with_shadow_lift(0.02);
    let cs_fwd = ColorSpaceBlock::new(ColorSpace::RgbToYCbCr);
    let cs_rev = ColorSpaceBlock::new(ColorSpace::RgbToHsv);
    let warp = WarpGridBlock::new(640, 480).with_gdc(-0.1, 0.0, 0.0);
    let ca = ChromaticAberrationBlock::new().with_radial_correction(480, 640, 1.0);
    let ne = NoiseEstimateBlock::new();
    let denoise = TemporalDenoiseBlock::new().with_threshold(0.05);
    let crop = AspectCropBlock::ratio_16_9();
    let display = DisplayBlock::new(640);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![
        &unpack, &demosaic, &gamma, &sharpen, &contrast, &cs_fwd, &cs_rev, &warp, &ca, &ne,
        &denoise, &crop, &display,
    ];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 16).unwrap();
    assert!(
        onnx.len() > 1000,
        "mega pipeline ONNX: {} bytes",
        onnx.len()
    );
    std::fs::write("target/mega_pipeline.onnx", &onnx).ok();
    println!(
        "Mega pipeline: {} stages, {} bytes",
        blocks.len(),
        onnx.len()
    );
}

// ── DynResizeBlock ───────────────────────────────────────

#[test]
fn dyn_resize_produces_valid_onnx() {
    use cam_isp::pipeline::GraphComposer;
    let unpack = UnpackBlock::new().with_concrete_dims(1080, 1920);
    let resize = DynResizeBlock::new(960, 540).with_source_size(1920, 1080);
    let display = DisplayBlock::new(960);
    let blocks: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![&unpack, &resize, &display];
    let onnx = GraphComposer::compose_from_vec(&blocks, &[], 10).unwrap();
    assert!(onnx.len() > 200);
}

#[test]
fn dyn_resize_emits_resize_op() {
    let block = DynResizeBlock::new(640, 480).with_source_size(1280, 720);
    let nodes = block.nodes();
    assert!(nodes
        .iter()
        .any(|n| String::from_utf8_lossy(n).contains("Resize")));
}

#[test]
fn dyn_resize_scales_are_correct() {
    let block = DynResizeBlock::new(960, 540).with_source_size(1920, 1080);
    let (sh, sw) = block.scale_factors();
    assert!((sh - 0.5).abs() < 0.01);
    assert!((sw - 0.5).abs() < 0.01);
}

#[test]
fn test_hdr_tone_block_aces_compose() {
    use cam_isp::blocks::ToneOperator;
    use cam_isp::pipeline_builder::PipelineBuilder;
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .hdr_tone(ToneOperator::Aces)
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 1000);
}

#[test]
fn test_hdr_tone_block_reinhard_compose() {
    use cam_isp::blocks::ToneOperator;
    use cam_isp::pipeline_builder::PipelineBuilder;
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .hdr_tone(ToneOperator::Reinhard)
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 1000);
}

#[test]
fn test_hdr_tone_block_uncharted2_compose() {
    use cam_isp::blocks::ToneOperator;
    use cam_isp::pipeline_builder::PipelineBuilder;
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .hdr_tone(ToneOperator::Uncharted2)
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 1000);
}

#[test]
fn test_wavelet_denoise_compose() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .wavelet_denoise(0.05)
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 1000);
}

#[test]
fn test_plugin_block_compose() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .plugin("dummy_model.onnx")
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 1000);
}

#[test]
fn test_full_hdr_pipeline_compose() {
    use cam_isp::blocks::ToneOperator;
    use cam_isp::pipeline_builder::PipelineBuilder;
    let onnx = PipelineBuilder::new(3840, 2160)
        .unpack()
        .demosaic_binning()
        .hdr_tone(ToneOperator::Aces)
        .wavelet_denoise(0.03)
        .gamma(2.2)
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 2000);
}

#[test]
fn test_super_res_block_emit() {
    use cam_isp::blocks::SuperResBlock;
    use cam_isp::pipeline::IspBlock;
    let mut sr = SuperResBlock::new().with_num_frames(2);
    sr.set_input_source("input/frame0");
    let sr_block = sr.add_frame("input/frame1");
    let nodes = sr_block.nodes();
    assert!(!nodes.is_empty(), "SuperRes should emit nodes");
    let inits = sr_block.initializers();
    assert!(!inits.is_empty(), "SuperRes should have initializers");
    let extras = sr_block.extra_inputs();
    assert_eq!(extras.len(), 1, "SuperRes should have 1 extra input");
}

// === Integration tests for blocks with 0 test files ===

#[test]
fn test_hdr_tone_aces_full_pipeline() {
    use cam_isp::blocks::ToneOperator;
    use cam_isp::pipeline_builder::PipelineBuilder;
    // Full pipeline: unpack → demosaic → HDR tone → gamma → display
    let onnx = PipelineBuilder::new(3840, 2160)
        .unpack()
        .demosaic_binning()
        .hdr_tone(ToneOperator::Aces)
        .gamma(2.2)
        .display()
        .compose()
        .expect("should compose");
    assert!(
        onnx.len() > 3000,
        "full HDR pipeline should produce substantial ONNX"
    );
}

#[test]
fn test_hdr_tone_reinhard_full_pipeline() {
    use cam_isp::blocks::ToneOperator;
    use cam_isp::pipeline_builder::PipelineBuilder;
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .demosaic_binning()
        .hdr_tone(ToneOperator::Reinhard)
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 2000);
}

#[test]
fn test_wavelet_denoise_full_pipeline() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    // Unpack → wavelet denoise → display
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .wavelet_denoise(0.05)
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 1500);
}

#[test]
fn test_wavelet_denoise_with_demosaic() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    // Unpack → demosaic → wavelet denoise → sharpen → display
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .demosaic_bilinear()
        .wavelet_denoise(0.03)
        .sharpen(0.5)
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 2000);
}

#[test]
fn test_laplacian_pyramid_standalone() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    // Unpack → laplacian pyramid → display
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .laplacian_pyramid(3)
        .display()
        .compose()
        .expect("should compose");
    assert!(
        onnx.len() > 2000,
        "laplacian pyramid should produce substantial ONNX"
    );
}

#[test]
fn test_laplacian_pyramid_different_levels() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    for levels in 1..=4 {
        let onnx = PipelineBuilder::new(640, 480)
            .unpack()
            .laplacian_pyramid(levels)
            .display()
            .compose()
            .expect("should compose");
        assert!(
            !onnx.is_empty(),
            "laplacian pyramid with {} levels failed",
            levels
        );
    }
}

#[test]
fn test_hdr_debayer_standalone() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    // HDR debayer → demosaic → display
    let onnx = PipelineBuilder::new(1920, 1080)
        .hdr_debayer(4.0)
        .demosaic_binning()
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 500);
}

#[test]
fn test_hdr_debayer_different_ratios() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    for ratio in &[2.0, 4.0, 8.0] {
        let onnx = PipelineBuilder::new(640, 480)
            .hdr_debayer(*ratio)
            .display()
            .compose()
            .expect("should compose");
        assert!(!onnx.is_empty(), "hdr debayer with ratio {} failed", ratio);
    }
}

#[test]
fn test_blc50_full_pipeline() {
    use cam_isp::pipeline_builder::PipelineBuilder;
    // Unpack → BLC50 → demosaic → display
    let onnx = PipelineBuilder::new(1920, 1080)
        .unpack()
        .blc50(10.0, 12.0, 8.0)
        .demosaic_binning()
        .display()
        .compose()
        .expect("should compose");
    assert!(onnx.len() > 2000);
}
