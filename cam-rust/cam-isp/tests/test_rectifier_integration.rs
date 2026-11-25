//! Integration test for rectifier model + neural controller.

use cam_isp::controller_api::{Controller, ControllerApi};
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
        params: cam_isp::isp_params::IspParams::default(),
        timestamp_ns: 1000,
        prep_duration_ns: 0,
        inference_duration_ns: 0,
        total_duration_ns: 0,
        external_data: None,
    }
}

#[test]
fn test_mock_model_generation() {
    // Generate mock ONNX model
    let model = rectifier_model::generate_rectifier_model();

    assert!(!model.is_empty());
    assert!(model.len() > 1000);

    // Save to temp file
    let temp_dir = std::env::temp_dir();
    let model_path = temp_dir.join("test_rectifier_integration.onnx");
    rectifier_model::save_model(model_path.to_str().unwrap()).unwrap();

    // Verify file was created
    let metadata = std::fs::metadata(&model_path).unwrap();
    assert!(metadata.len() > 1000);

    // Cleanup
    let _ = std::fs::remove_file(&model_path);
}

#[test]
fn test_neural_controller_with_mock() {
    // Create neural controller with mock model
    let mut controller = NeuralController::with_mock_model();

    // Process frame
    let frame = create_test_frame();
    let params = controller.analyze_and_update(&frame);

    // Verify output
    assert!(params.wb.r > 0.0);
    assert!(params.wb.g > 0.0);
    assert!(params.wb.b > 0.0);

    // CCM should be close to identity
    assert!(params.ccm.matrix[0].abs() > 0.5); // R→R
    assert!(params.ccm.matrix[4].abs() > 0.5); // G→G
    assert!(params.ccm.matrix[8].abs() > 0.5); // B→B

    // Tone params should be valid
    assert!(params.tone.contrast > 0.0);
    assert!(params.tone.gamma > 0.0);
}

#[test]
fn test_controller_enum_with_mock() {
    // Create controller enum with neural variant
    let mut controller = Controller::neural();

    // Load mock model
    let model_bytes = rectifier_model::generate_rectifier_model();
    let temp_dir = std::env::temp_dir();
    let model_path = temp_dir.join("test_controller_enum.onnx");
    std::fs::write(&model_path, &model_bytes).unwrap();

    // Note: load_model requires rectifier feature
    // This test verifies the fallback path

    // Process frame
    let frame = create_test_frame();
    let params = controller.analyze_and_update(&frame);

    // Should produce valid params (via fallback)
    assert!(params.wb.r > 0.0);

    // Cleanup
    let _ = std::fs::remove_file(&model_path);
}

#[test]
fn test_mock_model_compatibility() {
    // Verify mock model has correct interface
    let model = rectifier_model::generate_rectifier_model();

    // Model should be valid ONNX protobuf
    // (protobuf doesn't have magic bytes, but should be non-empty)
    assert!(!model.is_empty());

    // Check it starts with valid protobuf data
    // First byte encodes field number and wire type
    assert!(model[0] != 0 || model.len() > 100);
}

#[test]
fn test_multiple_frame_processing() {
    let mut controller = NeuralController::with_mock_model();

    // Process 10 frames
    for i in 0..10 {
        let mut frame = create_test_frame();
        frame.timestamp_ns = i * 33_333_333; // 30fps timestamps

        let params = controller.analyze_and_update(&frame);

        // All frames should produce valid params
        assert!(params.wb.r > 0.0);
        assert!(params.wb.g > 0.0);
        assert!(params.wb.b > 0.0);
    }
}

#[test]
fn test_rule_based_vs_neural() {
    let mut rule_based = Controller::rule_based();
    let mut neural = Controller::neural();

    let frame = create_test_frame();

    let params_rule = rule_based.analyze_and_update(&frame);
    let params_neural = neural.analyze_and_update(&frame);

    // Both should produce valid params
    assert!(params_rule.wb.r > 0.0);
    assert!(params_neural.wb.r > 0.0);

    // They may differ but both should be reasonable
    // (rule-based uses heuristics, neural uses mock model)
}

/// Test loading the real pre-trained ONNX model variants.
///
/// Tries each candidate (light_int8 → light → light_fp16) from both
/// `isp-rectifier/models/` and repo root locations. The first one that
/// loads and runs successfully is used; if none exist (e.g. in CI without
/// the model files), the test asserts the fallback path still works.
#[test]
fn test_real_onnx_model_variants() {
    let candidates = [
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../isp-rectifier/models/fusedispcontroller_light_int8.onnx"),
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../isp-rectifier/models/fusedispcontroller_light.onnx"),
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../isp-rectifier/models/fusedispcontroller_light_fp16.onnx"),
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../isp-rectifier/fusedispcontroller_light_int8.onnx"),
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../isp-rectifier/fusedispcontroller_light.onnx"),
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../isp-rectifier/fusedispcontroller_light_fp16.onnx"),
    ];

    let existing: Vec<&str> = candidates.iter().filter(|p| std::path::Path::new(*p).exists()).copied().collect();

    if existing.is_empty() {
        // No pre-trained models available — verify fallback still produces valid params.
        let mut ctrl = NeuralController::new();
        assert!(!ctrl.has_model());
        let frame = create_test_frame();
        let params = ctrl.analyze_and_update(&frame);
        assert!(params.wb.r > 0.0);
        return;
    }

    // Try each candidate; at least one should load and produce valid params.
    let mut loaded_any = false;
    for path in &existing {
        let mut ctrl = NeuralController::with_model(path);
        if ctrl.has_model() {
            loaded_any = true;
            let frame = create_test_frame();
            let params = ctrl.analyze_and_update(&frame);
            assert!(
                params.wb.r > 0.0 && params.wb.g > 0.0 && params.wb.b > 0.0,
                "model {} produced invalid WB gains",
                path
            );
            assert!(
                params.ccm.matrix[0].abs() > 0.1
                    || params.ccm.matrix[4].abs() > 0.1
                    || params.ccm.matrix[8].abs() > 0.1,
                "model {} produced degenerate CCM",
                path
            );
            eprintln!(
                "Real ONNX model {} loaded OK: wb=[{:.3},{:.3},{:.3}]",
                path, params.wb.r, params.wb.g, params.wb.b
            );
        }
    }

    assert!(loaded_any, "none of the existing ONNX models could be loaded");
}

/// Test that `with_default_model()` picks a real model when available,
/// otherwise falls back gracefully.
#[test]
fn test_with_default_model() {
    let mut ctrl = NeuralController::with_default_model();

    let frame = create_test_frame();
    let params = ctrl.analyze_and_update(&frame);

    // Either path must produce valid params (model or fallback).
    assert!(params.wb.r > 0.0);
    assert!(params.wb.g > 0.0);
    assert!(params.wb.b > 0.0);

    if ctrl.has_model() {
        eprintln!("Default model loaded successfully");
    } else {
        eprintln!("No default model found, using rule-based fallback");
    }
}

/// Test both mock and real ONNX model produce params through `ControllerApi`.
#[test]
fn test_mock_and_real_model_through_controller_api() {
    // Mock model path
    let mut mock_ctrl = NeuralController::with_mock_model();
    let mock_has_model = mock_ctrl.has_model();

    let frame = create_test_frame();
    let mock_params = mock_ctrl.analyze_and_update(&frame);
    assert!(mock_params.wb.r > 0.0);

    // Real model path (if available)
    let real_path = "isp-rectifier/models/fusedispcontroller_light_int8.onnx";
    if std::path::Path::new(real_path).exists() {
        let mut real = NeuralController::with_model(real_path);
        assert!(real.has_model());
        let real_params = real.analyze_and_update(&frame);
        assert!(real_params.wb.r > 0.0);
        eprintln!(
            "Mock vs Real WB: mock=[{:.3},{:.3},{:.3}] real=[{:.3},{:.3},{:.3}] has_model={}",
            mock_params.wb.r, mock_params.wb.g, mock_params.wb.b,
            real_params.wb.r, real_params.wb.g, real_params.wb.b,
            mock_has_model
        );
    } else {
        eprintln!("Real ONNX model not found at {real_path}, skipping real-model comparison");
    }
}
