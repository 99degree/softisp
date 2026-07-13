//! Integration test: Neural Controller with ONNX model

use cam_isp::controller_api::ControllerApi;
use cam_isp::isp_params::IspParams;
use cam_isp::neural_controller::NeuralController;
use cam_isp::pipeline::IspFrame;
use cam_isp::rectifier_model;
use cam_types::FrameFormat;

fn create_test_frame() -> IspFrame {
    IspFrame {
        width: 1920,
        height: 1080,
        data: vec![128; 1920 * 1080 * 2],
        format: FrameFormat::RawSensor,
        float_data: None,
        aux: None,
        params: IspParams::default(),
        timestamp_ns: 1000,
        prep_duration_ns: 0,
        inference_duration_ns: 0,
        total_duration_ns: 0,
    }
}

#[test]
fn test_mock_model_controller() {
    // Test neural controller with generated mock model
    let mut ctrl = NeuralController::with_mock_model();

    let frame = create_test_frame();
    let params = ctrl.analyze_and_update(&frame);

    // Verify output
    assert!(params.wb.r > 0.0, "WB R should be positive");
    assert!(params.wb.g > 0.0, "WB G should be positive");
    assert!(params.wb.b > 0.0, "WB B should be positive");

    println!("Mock model output:");
    println!(
        "  WB: [{:.3}, {:.3}, {:.3}]",
        params.wb.r, params.wb.g, params.wb.b
    );
    println!("  Contrast: {:.3}", params.tone.contrast);
    println!("  Gamma: {:.3}", params.tone.gamma);
}

#[test]
fn test_params_flow_to_frame() {
    // Test that params flow from controller to frame
    let mut ctrl = NeuralController::with_mock_model();

    let mut frame = create_test_frame();
    let params = ctrl.analyze_and_update(&frame);

    // Store params in frame
    frame.params = params.clone();

    // Verify params are in frame
    assert!((frame.params.wb.r - params.wb.r).abs() < 0.001);
    assert!((frame.params.wb.g - params.wb.g).abs() < 0.001);
    assert!((frame.params.wb.b - params.wb.b).abs() < 0.001);

    println!("Params flow verified:");
    println!(
        "  frame.params.wb = [{:.3}, {:.3}, {:.3}]",
        frame.params.wb.r, frame.params.wb.g, frame.params.wb.b
    );
}

#[test]
fn test_per_frame_variation() {
    // Test that different frames can produce different params
    let mut ctrl = NeuralController::with_mock_model();

    // Frame 1: normal exposure
    let mut frame1 = create_test_frame();
    frame1.timestamp_ns = 1000;
    let params1 = ctrl.analyze_and_update(&frame1);

    // Frame 2: same frame (should be similar due to temporal smoothing)
    let mut frame2 = create_test_frame();
    frame2.timestamp_ns = 2000;
    let params2 = ctrl.analyze_and_update(&frame2);

    // Should be similar (temporal smoothing)
    let diff = (params1.wb.r - params2.wb.r).abs();
    assert!(diff < 0.1, "Temporal smoothing should keep params similar");

    println!("Per-frame variation:");
    println!(
        "  Frame 1 WB: [{:.3}, {:.3}, {:.3}]",
        params1.wb.r, params1.wb.g, params1.wb.b
    );
    println!(
        "  Frame 2 WB: [{:.3}, {:.3}, {:.3}]",
        params2.wb.r, params2.wb.g, params2.wb.b
    );
    println!("  WB R diff: {:.6}", diff);
}

#[test]
fn test_rule_based_vs_neural() {
    // Compare rule-based vs neural controller
    let mut rule_based = cam_isp::controller_api::Controller::rule_based();
    let mut neural = cam_isp::controller_api::Controller::neural();

    let frame = create_test_frame();

    let params_rule = rule_based.analyze_and_update(&frame);
    let params_neural = neural.analyze_and_update(&frame);

    // Both should produce valid params
    assert!(params_rule.wb.r > 0.0);
    assert!(params_neural.wb.r > 0.0);

    println!("Rule-based vs Neural:");
    println!(
        "  Rule-based WB: [{:.3}, {:.3}, {:.3}]",
        params_rule.wb.r, params_rule.wb.g, params_rule.wb.b
    );
    println!(
        "  Neural WB:     [{:.3}, {:.3}, {:.3}]",
        params_neural.wb.r, params_neural.wb.g, params_neural.wb.b
    );
}

#[test]
fn test_isp_params_structure() {
    // Test that IspParams has all expected fields
    let params = IspParams::default();

    // Verify all fields exist and have sensible defaults
    assert!(params.wb.r > 0.0);
    assert!(params.wb.g > 0.0);
    assert!(params.wb.b > 0.0);
    assert!(params.tone.contrast > 0.0);
    assert!(params.tone.gamma > 0.0);

    // CCM should be identity-like
    assert!(params.ccm.matrix[0].abs() > 0.5); // R->R
    assert!(params.ccm.matrix[4].abs() > 0.5); // G->G
    assert!(params.ccm.matrix[8].abs() > 0.5); // B->B

    println!("IspParams structure verified:");
    println!(
        "  WB: [{:.3}, {:.3}, {:.3}]",
        params.wb.r, params.wb.g, params.wb.b
    );
    println!(
        "  CCM diagonal: [{:.3}, {:.3}, {:.3}]",
        params.ccm.matrix[0], params.ccm.matrix[4], params.ccm.matrix[8]
    );
    println!(
        "  Tone: contrast={:.3}, gamma={:.3}",
        params.tone.contrast, params.tone.gamma
    );
}

#[test]
fn test_temporal_smoothing_effect() {
    // Test temporal smoothing over multiple frames
    let mut ctrl = NeuralController::new(); // Use fallback controller

    let mut prev_wb_r = 1.0;

    for i in 0..10 {
        let mut frame = create_test_frame();
        frame.timestamp_ns = i * 33_333_333; // 30fps

        let params = ctrl.analyze_and_update(&frame);

        // Check temporal smoothing
        let diff = (params.wb.r - prev_wb_r).abs();
        assert!(diff < 0.5, "Frame {}: WB change too large: {:.3}", i, diff);

        prev_wb_r = params.wb.r;
    }

    println!("Temporal smoothing verified over 10 frames");
}
