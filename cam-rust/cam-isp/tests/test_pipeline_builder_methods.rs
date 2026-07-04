//! Integration tests for the new PipelineBuilder methods.
//!
//! These tests verify the high-level builder API works end-to-end:
//! - compose_to_file / compose_to_file_at (ONNX file output)
//! - compose_and_time (timing measurement)
//! - compose_and_validate (validation before compose)
//! - from_config (config roundtrip)
//! - display_rgba_w / display_argb_w / display_agbr_w (format+width)
//! - workgroup on WarpGridBlock through builder

use cam_isp::pipeline_builder::PipelineBuilder;

#[test]
fn test_builder_compose_to_file() {
    let path = "test_builder_compose_to_file.onnx";
    let len = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_binning()
        .display()
        .compose_to_file(path)
        .expect("should compose");
    assert!(len > 100, "compose produced valid ONNX: {} bytes", len);
    let metadata = std::fs::metadata(path).expect("file should exist");
    assert_eq!(metadata.len() as usize, len, "file size matches compose length");
    let _ = std::fs::remove_file(path);
}

#[test]
fn test_builder_compose_to_file_at() {
    let path = "test_builder_compose_to_file_at.onnx";
    let len = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_bilinear()
        .display()
        .compose_to_file_at(3840, 2160, path)
        .expect("should compose");
    assert!(len > 100);
    let metadata = std::fs::metadata(path).expect("file should exist");
    assert_eq!(metadata.len() as usize, len);
    let _ = std::fs::remove_file(path);
}

#[test]
fn test_builder_compose_and_time() {
    let (onnx, elapsed) = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_mhc()
        .display()
        .compose_and_time()
        .expect("should compose");
    assert!(onnx.len() > 100, "ONNX bytes: {}", onnx.len());
    assert!(elapsed.as_secs_f64() > 0.0, "elapsed must be positive: {:?}", elapsed);
    assert!(elapsed.as_secs() < 60, "compose should be fast: {:?}", elapsed);
}

#[test]
fn test_builder_compose_and_validate_valid() {
    let onnx = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_bilinear()
        .display()
        .compose_and_validate()
        .expect("valid pipeline should compose");
    assert!(onnx.len() > 100);
}

#[test]
fn test_builder_compose_and_validate_empty() {
    let result = PipelineBuilder::new(640, 480).compose_and_validate();
    assert!(result.is_err(), "empty pipeline should fail validation");
    match result.unwrap_err() {
        cam_isp::pipeline_builder::PipelineError::EmptyPipeline => (),
        other => panic!("expected EmptyPipeline, got {:?}", other),
    }
}

#[test]
fn test_builder_compose_and_validate_zero_res() {
    let result = PipelineBuilder::new(0, 0)
        .unpack()
        .display()
        .compose_and_validate();
    assert!(result.is_err(), "zero resolution should fail validation");
    match result.unwrap_err() {
        cam_isp::pipeline_builder::PipelineError::InvalidResolution(0, 0) => (),
        other => panic!("expected InvalidResolution, got {:?}", other),
    }
}

#[test]
fn test_builder_summary_format() {
    let s = PipelineBuilder::new(1920, 1080)
        .unpack()
        .demosaic_mhc()
        .gamma(2.2)
        .sharpen(0.5)
        .display()
        .summary();
    assert!(s.contains("1920×1080"), "should contain resolution: {}", s);
    assert!(s.contains("unpack"), "should contain unpack: {}", s);
    assert!(s.contains("demosaic"), "should contain demosaic: {}", s);
    assert!(s.contains("gamma"), "should contain gamma: {}", s);
    assert!(s.contains("sharpen"), "should contain sharpen: {}", s);
    assert!(s.contains("display"), "should contain display: {}", s);
    assert!(s.contains("→"), "should contain arrow separator: {}", s);
}

#[test]
fn test_builder_block_introspection() {
    let b = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_bilinear()
        .gamma(2.2)
        .display();
    assert_eq!(b.block_count(), 4, "should have 4 blocks");
    assert_eq!(
        b.block_ids(),
        vec!["unpack", "demosaic_ccm", "gamma", "display"],
        "block_ids should match construction order"
    );
}

#[test]
fn test_builder_remove_block() {
    let mut b = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_bilinear()
        .gamma(2.2)
        .display();
    let removed = b.remove_block("gamma");
    assert!(removed, "remove_block should succeed");
    assert_eq!(b.block_count(), 3, "3 blocks after removing gamma");
    let missing = b.remove_block("nonexistent");
    assert!(!missing, "missing block returns false");
}

#[test]
fn test_builder_replace_block() {
    use cam_isp::blocks::GammaBlock;

    let mut b = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_bilinear()
        .gamma(2.2)
        .display();
    let replaced = b.replace_block("gamma", Box::new(GammaBlock::new(1.8)));
    assert!(replaced, "replace_block should succeed");
    assert!(b.block_ids().contains(&"gamma".to_string()), "gamma still in pipeline");
}

#[test]
fn test_builder_display_all_format_methods() {
    let rgba = PipelineBuilder::new(640, 480).unpack().display_rgba();
    assert_eq!(rgba.block_count(), 2);

    let argb = PipelineBuilder::new(640, 480).unpack().display_argb();
    assert_eq!(argb.block_count(), 2);

    let agbr = PipelineBuilder::new(640, 480).unpack().display_agbr();
    assert_eq!(agbr.block_count(), 2);

    let rgba_w = PipelineBuilder::new(1920, 1080).unpack().display_rgba_w(960);
    assert_eq!(rgba_w.block_count(), 2);

    let argb_w = PipelineBuilder::new(1920, 1080).unpack().display_argb_w(960);
    assert_eq!(argb_w.block_count(), 2);

    let agbr_w = PipelineBuilder::new(1920, 1080).unpack().display_agbr_w(960);
    assert_eq!(agbr_w.block_count(), 2);
}

#[test]
fn test_builder_from_config_roundtrip() {
    use cam_isp::serializer::PipelineConfig;

    let mut cfg = PipelineConfig::new(1920, 1080);
    cfg.block_ids = vec!["unpack".into(), "demosaic".into(), "gamma".into(), "display".into()];

    let b = PipelineBuilder::from_config(&cfg);
    assert_eq!(b.block_count(), 4, "should reconstruct 4 blocks from config");

    let onnx = b.compose().expect("reconstructed pipeline should compose");
    assert!(onnx.len() > 100);
}

#[test]
fn test_builder_to_config_roundtrip() {
    let original = PipelineBuilder::new(1920, 1080)
        .unpack()
        .demosaic_mhc()
        .gamma(2.2)
        .display();
    let cfg = original.to_config();

    assert_eq!(cfg.width, 1920);
    assert_eq!(cfg.height, 1080);
    assert_eq!(cfg.block_ids.len(), 4);

    // Re-build from config and check same block structure
    let rebuilt = PipelineBuilder::from_config(&cfg);
    assert_eq!(rebuilt.block_count(), 4);
    assert_eq!(rebuilt.block_ids().len(), cfg.block_ids.len());
}

#[test]
fn test_builder_workgroup_via_add() {
    use cam_isp::blocks::*;

    let b = PipelineBuilder::new(640, 480)
        .unpack()
        .push(Box::new(WarpGridBlock::new(32, 32).workgroup(32, 8)))  // Mali preset
        .push(Box::new(DisplayBlock::new(640).rgba().workgroup(64, 4)))  // Adreno preset
        .compose()
        .expect("should compose");
    assert!(b.len() > 100);
}

#[test]
fn test_builder_compose_full_includes_stats() {
    let (onnx, stats, _issues) = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_bilinear()
        .gamma(2.2)
        .display()
        .compose_full()
        .expect("should compose");
    assert!(onnx.len() > 100, "ONNX bytes: {}", onnx.len());
    assert!(stats.block_count >= 4, "stats block_count: {}", stats.block_count);
}

#[test]
fn test_builder_all_stats() {
    let stats = PipelineBuilder::new(640, 480)
        .unpack()
        .demosaic_bilinear()
        .gamma(2.2)
        .display()
        .all_stats(1920, 1080)
        .expect("should compute stats");
    assert!(!stats.is_empty(), "should have block stats");
    let (first, flops, mem) = &stats[0];
    assert!(!first.is_empty(), "block name should be non-empty");
    println!("Block '{}': {} FLOPs, {} memory bytes", first, flops, mem);
}

#[test]
fn test_builder_presets_compose() {
    let photo = PipelineBuilder::photo_preset(1920, 1080).compose().expect("photo");
    let video = PipelineBuilder::video_preset(1920, 1080).compose().expect("video");
    let night = PipelineBuilder::night_preset(1920, 1080).compose().expect("night");
    let minimal = PipelineBuilder::minimal_preset(640, 480).compose().expect("minimal");

    assert!(photo.len() > 1000, "photo preset: {} bytes", photo.len());
    assert!(video.len() > 1000, "video preset: {} bytes", video.len());
    assert!(night.len() > 1000, "night preset: {} bytes", night.len());
    assert!(minimal.len() > 100, "minimal preset: {} bytes", minimal.len());
}

#[test]
fn test_builder_empty_block_count_zero() {
    let b = PipelineBuilder::new(640, 480);
    assert_eq!(b.block_count(), 0);
    assert!(b.block_ids().is_empty());
}

#[test]
fn test_builder_validate_method() {
    let empty = PipelineBuilder::new(640, 480);
    assert!(empty.validate().is_err(), "empty pipeline should fail");

    let zero_res = PipelineBuilder::new(0, 0).unpack().display();
    assert!(zero_res.validate().is_err(), "zero res should fail");

    let valid = PipelineBuilder::new(640, 480).unpack().display();
    assert!(valid.validate().is_ok(), "valid pipeline should pass");
}

#[test]
fn test_builder_to_dot_export() {
    let dot = PipelineBuilder::new(1920, 1080)
        .unpack()
        .demosaic_binning()
        .gamma(2.2)
        .sharpen(0.5)
        .display()
        .to_dot();

    assert!(dot.contains("digraph ISP"));
    assert!(dot.contains("1920x1080"));
    assert!(dot.contains("input -> block_0"));
    assert!(dot.contains("block_4 -> output"));
    assert!(dot.contains("unpack"));
    assert!(dot.contains("gamma"));
}

#[test]
fn test_builder_to_dot_empty_pipeline() {
    let dot = PipelineBuilder::new(640, 480).to_dot();
    assert!(dot.contains("digraph ISP"));
    assert!(dot.contains("input -> output"));
    assert!(!dot.contains("block_0"));
}

#[test]
fn test_builder_cost_estimator() {
    let (flops, mem) = PipelineBuilder::new(1920, 1080)
        .unpack()
        .demosaic_binning()
        .gamma(2.2)
        .display()
        .cost();
    assert!(flops > 0, "FLOPs should be positive");
    assert!(mem > 0, "Memory should be positive");
}

#[test]
fn test_builder_cost_scales_with_resolution() {
    let (flops_small, _) = PipelineBuilder::new(640, 480)
        .unpack().display().cost();
    let (flops_large, _) = PipelineBuilder::new(3840, 2160)
        .unpack().display().cost();
    assert!(flops_large > flops_small, "4K should cost more than HD");
}
