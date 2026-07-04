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
}
