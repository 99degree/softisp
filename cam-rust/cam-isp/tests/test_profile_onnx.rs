//! Integration tests for ONNX model generation across all profiles.
//!
//! All tests must call `GraphComposer::wire_blocks` before
//! `compose_from_vec` to ensure proper tensor connections.
//! Without wiring, MNNConvert segfaults on the resulting model.

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use cam_isp::pipeline::IspBlock;

/// Helper: build profile blocks (already wired by build_blocks)
fn wired_blocks(profile: PipelineProfile) -> Vec<Box<dyn IspBlock>> {
    profile.build_blocks(8, 2)
}

/// Helper: compose wired blocks into ONNX model
fn compose_wired(blocks: &[Box<dyn IspBlock>]) -> Vec<u8> {
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    GraphComposer::compose_from_vec(&refs, &[], 16).unwrap()
}

// ── All profiles compose without error ─────────────────────────────

#[test]
fn test_test_profile_onnx() {
    let blocks = wired_blocks(PipelineProfile::TEST);
    let model = compose_wired(&blocks);
    assert!(!model.is_empty(), "TEST profile ONNX should not be empty");
    assert!(model.len() > 200, "Model should be substantial");
}

#[test]
fn test_lite_profile_onnx() {
    let blocks = wired_blocks(PipelineProfile::LITE);
    let model = compose_wired(&blocks);
    assert!(!model.is_empty(), "LITE profile ONNX should not be empty");
    assert_eq!(blocks.len(), 10, "LITE profile should have 10 blocks");
    assert!(model.len() > 2000, "LITE model should be substantial");
}

#[test]
fn test_med_profile_onnx() {
    let blocks = wired_blocks(PipelineProfile::MED);
    let model = compose_wired(&blocks);
    assert!(!model.is_empty(), "MED profile ONNX should not be empty");
    assert_eq!(blocks.len(), 12, "MED profile should have 12 blocks");
    assert!(model.len() > 2000, "MED model should be substantial");
}

#[test]
fn test_heavy_profile_onnx() {
    let blocks = wired_blocks(PipelineProfile::HEAVY);
    let model = compose_wired(&blocks);
    assert!(!model.is_empty(), "HEAVY profile ONNX should not be empty");
    assert_eq!(blocks.len(), 14, "HEAVY profile should have 14 blocks");
    assert!(model.len() > 3000, "HEAVY model should be substantial");
}

#[test]
fn test_pro_profile_onnx() {
    let blocks = wired_blocks(PipelineProfile::PRO);
    let model = compose_wired(&blocks);
    assert!(!model.is_empty(), "PRO profile ONNX should not be empty");
    assert_eq!(blocks.len(), 15, "PRO profile should have 15 blocks");
    assert!(model.len() > 3000, "PRO model should be substantial");
}

// ── Block order validation ────────────────────────────────────────

#[test]
fn test_lite_pipeline_has_correct_block_order() {
    let blocks = PipelineProfile::LITE.build_blocks(8, 2);
    let names: Vec<&str> = blocks.iter().map(|b| b.id()).collect();
    assert_eq!(names[0], "raw_input");
    assert_eq!(names[1], "unpack");
    assert_eq!(names[2], "normalize");
    assert_eq!(names[3], "cfa");
    assert_eq!(names[4], "blc");
    assert_eq!(names[5], "bayer_wb");
    assert_eq!(names[6], "demosaic");
    assert_eq!(names[7], "ccm");
    assert_eq!(names[8], "tone");
    assert_eq!(names[9], "display");
}

#[test]
fn test_heavy_pipeline_block_order() {
    let blocks = PipelineProfile::HEAVY.build_blocks(8, 2);
    let names: Vec<&str> = blocks.iter().map(|b| b.id()).collect();
    // HEAVY: raw, unpack, norm, blc(dpc), cfa, blc, ccm(lsc), wb, demo, ccm, tone, ccm(ldci), ccm(unsharp), display
    assert_eq!(names.len(), 14);
    assert_eq!(names[0], "raw_input");
    assert_eq!(names[1], "unpack");
    assert_eq!(names[2], "normalize");
    assert_eq!(names[3], "blc");  // first BLC acts as DPC
    assert_eq!(names[4], "cfa");
    assert_eq!(names[5], "blc");      // second BLC (for black level)
    assert_eq!(names[6], "ccm");  // LSC uses CCM block
    assert_eq!(names[7], "bayer_wb");
    assert_eq!(names[8], "demosaic");
    assert_eq!(names[9], "ccm");
    assert_eq!(names[10], "tone");
    assert_eq!(names[11], "ccm");  // local contrast
    assert_eq!(names[12], "ccm");  // unsharp
    assert_eq!(names[13], "display");
}

// ── Build blocks now internally wires ───────────────────────────
// build_blocks calls wire_blocks internally for proper MNN compat.

#[test]
fn test_build_blocks_wires_internally() {
    let blocks = PipelineProfile::LITE.build_blocks(8, 2);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    // First block has no input, second block's input should match
    // first block's output
    assert!(refs[0].input_tensors().is_empty(),
        "RawInput should have no inputs");
    let first_out = refs[0].output_tensors()[0].clone();
    let second_in = refs[1].input_tensors()[0].clone();
    assert_eq!(first_out, second_in,
        "Block 0 output '{}' should connect to block 1 input '{}'",
        first_out, second_in);
}

// ── MNN conversion (requires MNN library, ignored by default) ─────

#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires MNNConvert binary and MNN libraries on device
fn test_lite_profile_mnn_conversion() {
    let blocks = wired_blocks(PipelineProfile::LITE);
    let model = compose_wired(&blocks);
    std::fs::write("/data/local/tmp/test_lite_mnn.onnx", &model).unwrap();
    
    use cam_isp::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};
    let opts = MnnConvertOptions::default();
    let result = convert_onnx_to_mnn(
        "/data/local/tmp/test_lite_mnn.onnx",
        "/data/local/tmp/test_lite_mnn.mnn",
        Some(&opts));
    assert!(result.is_ok(), "MNN conversion failed: {:?}", result.err());
}
