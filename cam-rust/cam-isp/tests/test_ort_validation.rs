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
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

/// Validate an ONNX model by loading it into an ORT session.
fn validate_with_ort(model_bytes: &[u8]) -> Result<String, String> {
    match Session::builder().and_then(|mut b| b.commit_from_memory(model_bytes)) {
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
        Err(e) => Err(format!("ORT validation failed: {}", e)),
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
    assert!(
        result.is_ok(),
        "LITE model should be valid ORT model: {:?}",
        result.err()
    );
}

#[test]
fn test_heavy_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::HEAVY);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!(
        "HEAVY: {}",
        result.as_ref().unwrap_or(&"FAILED".to_string())
    );
    assert!(
        result.is_ok(),
        "HEAVY model should be valid ORT model: {:?}",
        result.err()
    );
}

#[test]
fn test_med_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::MED);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!("MED: {}", result.as_ref().unwrap_or(&"FAILED".to_string()));
    assert!(
        result.is_ok(),
        "MED model should be valid ORT model: {:?}",
        result.err()
    );
}

#[test]
fn test_pro_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::PRO);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!("PRO: {}", result.as_ref().unwrap_or(&"FAILED".to_string()));
    assert!(
        result.is_ok(),
        "PRO model should be valid ORT model: {:?}",
        result.err()
    );
}

#[test]
fn test_reference_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::REFERENCE);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!(
        "REFERENCE: {}",
        result.as_ref().unwrap_or(&"FAILED".to_string())
    );
    assert!(
        result.is_ok(),
        "REFERENCE model should be valid ORT model: {:?}",
        result.err()
    );
}

#[test]
fn test_infinite_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::INFINITE);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!(
        "INFINITE: {}",
        result.as_ref().unwrap_or(&"FAILED".to_string())
    );
    assert!(
        result.is_ok(),
        "INFINITE model should be valid ORT model: {:?}",
        result.err()
    );
}

#[test]
fn test_test_model_ort_validation() {
    let blocks = wired_blocks(PipelineProfile::TEST);
    let model = compose_wired(&blocks);
    let result = validate_with_ort(&model);
    eprintln!("TEST: {}", result.as_ref().unwrap_or(&"FAILED".to_string()));
    assert!(
        result.is_ok(),
        "TEST model should be valid ORT model: {:?}",
        result.err()
    );
}

/// End-to-end inference test: create a LITE model, feed a dummy INT16 input,
/// run inference, and verify the output has the expected shape.
#[test]
fn test_lite_ort_inference() {
    use ort::value::Tensor;

    let blocks = wired_blocks(PipelineProfile::LITE);
    let model = compose_wired(&blocks);

    let mut session = Session::builder()
        .and_then(|mut b| b.commit_from_memory(&model))
        .expect("LITE model should create a valid ORT session");

    let w: u32 = 8;
    let h: u32 = 8;

    // LITE profile has exactly one input (RawInputPackedBlock/frame) and one output (DisplayBlock/frame)
    let input_name = "RawInputPackedBlock/frame";
    let output_name = "DisplayBlock/frame";

    eprintln!(
        "ORT inference: input='{}' output='{}'",
        input_name, output_name
    );

    // Build input tensor: INT16 [1, 1, h, w]
    let input_data: Vec<i16> = vec![2000i16; (w * h) as usize];
    let tensor = Tensor::from_array((
        vec![1i64, 1, h as i64, w as i64],
        input_data.into_boxed_slice(),
    ))
    .unwrap();

    // Run inference
    let result = session.run(ort::inputs![input_name => tensor]);
    assert!(
        result.is_ok(),
        "ORT inference should succeed: {:?}",
        result.err()
    );
    let outputs_map = result.unwrap();

    let output_val = outputs_map
        .get(output_name)
        .expect("output should be in results");

    let (shape, data) = output_val
        .try_extract_tensor::<u8>()
        .expect("output should be UINT8 tensor");

    let out_vec: Vec<u8> = data.iter().copied().collect();
    eprintln!(
        "ORT inference OK: {}x{} raw -> shape {:?} ({} bytes, first 4: {:?})",
        w,
        h,
        shape,
        out_vec.len(),
        &out_vec[..4.min(out_vec.len())]
    );

    assert_eq!(
        out_vec.len() as u64,
        shape.iter().map(|d| *d as u64).product::<u64>(),
        "output element count matches shape"
    );
    assert!(!out_vec.is_empty(), "output should not be empty");
    assert!(
        out_vec.len() >= (w * h * 4) as usize,
        "output should be at least {} bytes for BGRA output",
        w * h * 4
    );
}
