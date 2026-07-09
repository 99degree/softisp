//! Integration test for rectifier model + neural controller.

use cam_isp::neural_controller::NeuralController;
use cam_isp::controller_api::{Controller, ControllerApi};
use cam_isp::rectifier_model;
use cam_isp::pipeline::IspFrame;
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
