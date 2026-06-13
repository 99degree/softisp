//! ONNX compliance tests — validates generated models using Python's
//! official `onnx.checker` module. This checks against the actual ONNX
//! protobuf schema and validates structural integrity.
//!
//! The tests generate ONNX models using our pipeline, write them to disk,
//! and invoke Python's onnx.checker.check_model() which is the reference
//! ONNX validation implementation.

use std::process::Command;
use std::io::Write;

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use cam_isp::pipeline::IspBlock;

/// Validate an ONNX model blob using Python's onnx.checker.
fn validate_with_python(model_bytes: &[u8]) -> Result<String, String> {
    // Write model to temp file
    let temp_dir = std::env::temp_dir();
    let model_path = temp_dir.join("cam_isp_validate.onnx");
    let mut f = std::fs::File::create(&model_path).map_err(|e| format!("Create temp file: {}", e))?;
    f.write_all(model_bytes).map_err(|e| format!("Write model: {}", e))?;
    drop(f);

    let model_str = model_path.display().to_string().replace("\\", "/");

    // Build Python script — avoid Rust format! escaping by using String push
    let mut script = String::new();
    script.push_str("import onnx, onnx.checker, sys\n");
    script.push_str("try:\n");
    script.push_str(&format!("    model = onnx.load(\"{}\")\n", model_str));
    script.push_str("    onnx.checker.check_model(model)\n");
    script.push_str("    print(\"VALID\")\n");
    script.push_str("    print(\"ir_version:\", model.ir_version)\n");
    script.push_str("    print(\"nodes:\", len(model.graph.node))\n");
    script.push_str("    for n in model.graph.node:\n");
    script.push_str("        print(n.op_type + \": \" + n.name)\n");
    script.push_str("except onnx.checker.ValidationError as e:\n");
    script.push_str("    print(\"INVALID:\", str(e))\n");
    script.push_str("    sys.exit(1)\n");
    script.push_str("except Exception as e:\n");
    script.push_str("    print(\"ERROR:\", str(e))\n");
    script.push_str("    sys.exit(1)\n");

    let output = Command::new("python3")
        .arg("-c")
        .arg(&script)
        .output()
        .map_err(|e| format!("Execute python3: {}", e))?;

    // Cleanup
    let _ = std::fs::remove_file(&model_path);

    if output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout).to_string();
        Ok(stdout)
    } else {
        let stderr = String::from_utf8_lossy(&output.stderr).to_string();
        Err(format!("Validation failed:\n{}", stderr))
    }
}

/// Helper: build and wire profile blocks (wired by build_blocks internally)
fn wired_blocks(profile: PipelineProfile) -> Vec<Box<dyn IspBlock>> {
    profile.build_blocks(8, 2)
}

/// Helper: compose wired blocks into ONNX model
fn compose_wired(blocks: &[Box<dyn IspBlock>]) -> Vec<u8> {
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    GraphComposer::compose_from_vec(&refs, &[], 16).unwrap()
}

// ── Compliance tests ─────────────────────────────────────────────

#[test]
fn test_lite_profile_onnx_compliance() {
    let blocks = wired_blocks(PipelineProfile::LITE);
    let model = compose_wired(&blocks);
    let result = validate_with_python(&model);
    assert!(result.is_ok(), "LITE model failed ONNX validation: {:?}", result.err());
    let report = result.unwrap();
    assert!(report.contains("VALID"), "LITE model not valid:\n{}", report);
    eprintln!("LITE:\n{}", report);
}

#[test]
fn test_med_profile_onnx_compliance() {
    let blocks = wired_blocks(PipelineProfile::MED);
    let model = compose_wired(&blocks);
    let result = validate_with_python(&model);
    assert!(result.is_ok(), "MED model failed ONNX validation: {:?}", result.err());
    let report = result.unwrap();
    assert!(report.contains("VALID"), "MED model not valid:\n{}", report);
    eprintln!("MED:\n{}", report);
}

#[test]
fn test_heavy_profile_onnx_compliance() {
    let blocks = wired_blocks(PipelineProfile::HEAVY);
    let model = compose_wired(&blocks);
    let result = validate_with_python(&model);
    assert!(result.is_ok(), "HEAVY model failed ONNX validation: {:?}", result.err());
    let report = result.unwrap();
    assert!(report.contains("VALID"), "HEAVY model not valid:\n{}", report);
    eprintln!("HEAVY:\n{}", report);
}

#[test]
fn test_pro_profile_onnx_compliance() {
    let blocks = wired_blocks(PipelineProfile::PRO);
    let model = compose_wired(&blocks);
    let result = validate_with_python(&model);
    assert!(result.is_ok(), "PRO model failed ONNX validation: {:?}", result.err());
    let report = result.unwrap();
    assert!(report.contains("VALID"), "PRO model not valid:\n{}", report);
    eprintln!("PRO:\n{}", report);
}

#[test]
fn test_test_profile_onnx_compliance() {
    let blocks = wired_blocks(PipelineProfile::TEST);
    let model = compose_wired(&blocks);
    let result = validate_with_python(&model);
    assert!(result.is_ok(), "TEST model failed ONNX validation: {:?}", result.err());
    let report = result.unwrap();
    assert!(report.contains("VALID"), "TEST model not valid:\n{}", report);
    eprintln!("TEST:\n{}", report);
}
