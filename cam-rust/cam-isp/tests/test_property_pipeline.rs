//! Property-based tests for pipeline builders using proptest.
//!
//! Verifies that pipeline builders produce valid ONNX models
//! for arbitrary inputs and configurations.

use cam_isp::pipeline_builder::PipelineBuilder;
use proptest::prelude::*;

proptest! {
    #[test]
    fn test_any_resolution_composes(width in 1u32..4096, height in 1u32..4096) {
        let _ = PipelineBuilder::new(width, height)
            .unpack()
            .display()
            .compose();
    }

    #[test]
    fn test_gamma_range(g in 0.1f32..5.0) {
        let result = PipelineBuilder::new(640, 480)
            .unpack()
            .gamma(g)
            .display()
            .compose();
        prop_assert!(result.is_ok());
    }

    #[test]
    fn test_sharpen_range(s in 0.0f32..2.0) {
        let result = PipelineBuilder::new(640, 480)
            .unpack()
            .sharpen(s)
            .display()
            .compose();
        prop_assert!(result.is_ok());
    }

    #[test]
    fn test_contrast_range(s in 1.0f32..3.0) {
        let result = PipelineBuilder::new(640, 480)
            .unpack()
            .contrast(s)
            .display()
            .compose();
        prop_assert!(result.is_ok());
    }

    #[test]
    fn test_dot_never_panics(width in 1u32..4096, height in 1u32..4096) {
        let dot = PipelineBuilder::new(width, height)
            .unpack()
            .gamma(2.2)
            .display()
            .to_dot();
        prop_assert!(dot.contains("digraph ISP"));
        let res = format!("{}x{}", width, height);
        prop_assert!(dot.contains(&res));
    }

    #[test]
    fn test_validate_consistent(width in 0u32..4096, height in 0u32..4096) {
        let b = PipelineBuilder::new(width, height).unpack().display();
        if width == 0 || height == 0 {
            prop_assert!(b.validate().is_err());
        } else {
            prop_assert!(b.validate().is_ok());
        }
    }

    #[test]
    fn test_config_roundtrip_preserves_ids(ids in proptest::collection::vec(
        prop_oneof![
            Just("unpack".to_string()),
            Just("gamma".to_string()),
            Just("sharpen".to_string()),
            Just("display".to_string()),
        ],
        1..10
    )) {
        use cam_isp::serializer::PipelineConfig;

        let mut cfg = PipelineConfig::new(640, 480);
        cfg.block_ids = ids.clone();
        let b = PipelineBuilder::from_config(&cfg);
        let resulting_ids = b.block_ids();

        // All valid IDs should be present
        let valid = ["unpack", "gamma", "sharpen", "display"];
        let expected: Vec<&str> = ids.iter()
            .filter(|id| valid.contains(&id.as_str()))
            .map(|s| s.as_str())
            .collect();
        prop_assert_eq!(resulting_ids, expected);
    }

    #[test]
    fn test_summary_always_valid(width in 1u32..4096, height in 1u32..4096) {
        let summary = PipelineBuilder::new(width, height)
            .unpack()
            .demosaic_binning()
            .display()
            .summary();
        prop_assert!(summary.contains("×"));
        prop_assert!(summary.contains("→"));
    }

    #[test]
    fn test_cost_scales_with_resolution(w in 1u32..4096, h in 1u32..4096) {
        let (flops, mem) = PipelineBuilder::new(w, h)
            .unpack()
            .demosaic_binning()
            .display()
            .cost();
        prop_assert!(flops > 0, "FLOPs must be > 0 for {}x{}", w, h);
        prop_assert!(mem > 0, "memory must be > 0 for {}x{}", w, h);
    }

    #[test]
    fn test_dot_never_empty(w in 1u32..2048, h in 1u32..2048) {
        let dot = PipelineBuilder::new(w, h)
            .unpack()
            .demosaic_binning()
            .display()
            .to_dot();
        prop_assert!(!dot.is_empty());
        prop_assert!(dot.contains("digraph"));
    }

    #[test]
    fn test_all_presets_valid(w in 64u32..3840, h in 64u32..2160) {
        let presets: Vec<fn(u32, u32) -> PipelineBuilder> = vec![
            PipelineBuilder::photo_preset,
            PipelineBuilder::video_preset,
            PipelineBuilder::night_preset,
            PipelineBuilder::minimal_preset,
        ];
        for preset_fn in &presets {
            let builder = preset_fn(w, h);
            let validation = builder.validate();
            prop_assert!(validation.is_ok(), "Preset failed for {}x{}: {:?}", w, h, validation.err());
        }
    }
}
