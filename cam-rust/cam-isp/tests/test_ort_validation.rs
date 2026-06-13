//! ONNX model validation using ONNX Runtime via the `ort` crate.
//!
//! Creates an ORT session from model bytes — if the model is invalid
//! (malformed protobuf, unsupported ops, shape errors, etc.), ORT
//! returns an error. This is the authoritative validation: ORT IS the
//! ONNX standard reference implementation.
//!
//! Feature-gated behind `ort` feature (requires libonnxruntime.so).

#![cfg(feature = "ort")]

use ort::session::Session;

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use cam_isp::pipeline::IspBlock;

/// Validate an ONNX model by loading it into an ORT session.
fn validate_with_ort(model_bytes: &[u8]) -> Result<String, String> {
    match Session::builder()
        .and_then(|mut b| b.commit_from_memory(model_bytes))
    {
        Ok(session) => {
            let input_count = session.inputs().len();
            let output_count = session.outputs().len();
            
            let mut info = format!("VALID ({} inputs, {} outputs)\n", input_count, output_count);
            
            info.push_str("  Inputs:\n");
            for input in session.inputs() {
                info.push_str(&format!("    {}\n", input.name()));
            }
            
            info.push_str("  Outputs:\n");
            for output in session.outputs() {
                info.push_str(&format!("    {}\n", output.name()));
            }
            
            Ok(info)
        }
        Err(e) => {
            Err(format!("ORT validation failed: {}", e))
        }
    }
}

/// Helper: build wired profile blocks
fn wired_blocks(profile: PipelineProfile) -> Vec<Box<dyn IspBlock>> {
    profile.build_blocks(8, 2)
}

/// Helper: compose wired blocks into ONNX model
fn compose_wired(blocks: &[Box<dyn IspBlock>]) -> Vec<u8> {
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    GraphComposer::compose_from_vec(&refs, &[], 16).unwrap()
}

// ── Tests ────────────────────────────────────────────────────────

#[test]
fn test_lite_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::LITE);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!("LITE: {}", result.as_ref().unwrap_or(&"FAILED".to_string()));
    assert!(result.is_ok(), "LITE model should be valid ORT model: {:?}", result.err());
}

#[test]
fn test_heavy_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::HEAVY);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!("HEAVY: {}", result.as_ref().unwrap_or(&"FAILED".to_string()));
    assert!(result.is_ok(), "HEAVY model should be valid ORT model: {:?}", result.err());
}

#[test]
fn test_med_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::MED);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!("MED: {}", result.as_ref().unwrap_or(&"FAILED".to_string()));
    assert!(result.is_ok(), "MED model should be valid ORT model: {:?}", result.err());
}

#[test]
fn test_pro_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::PRO);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!("PRO: {}", result.as_ref().unwrap_or(&"FAILED".to_string()));
    assert!(result.is_ok(), "PRO model should be valid ORT model: {:?}", result.err());
}

#[test]
fn test_test_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::TEST);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!("TEST: {}", result.as_ref().unwrap_or(&"FAILED".to_string()));
    assert!(result.is_ok(), "TEST model should be valid ORT model: {:?}", result.err());
}
