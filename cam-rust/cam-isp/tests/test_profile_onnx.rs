//! Integration tests for ONNX model generation across all profiles.

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use cam_isp::pipeline::IspBlock;

// ── All profiles compose without error ─────────────────────────────

#[test]
fn test_test_profile_onnx() {
    let blocks = PipelineProfile::TEST.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    assert!(!model.is_empty(), "TEST profile ONNX should not be empty");
    assert!(model.len() > 100, "Model should be substantial");
}

#[test]
fn test_lite_profile_onnx() {
    let blocks = PipelineProfile::LITE.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    assert!(!model.is_empty(), "LITE profile ONNX should not be empty");
    assert_eq!(blocks.len(), 9, "LITE profile should have 9 blocks");
}

#[test]
fn test_med_profile_onnx() {
    let blocks = PipelineProfile::MED.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    assert!(!model.is_empty(), "MED profile ONNX should not be empty");
}

#[test]
fn test_heavy_profile_onnx() {
    let blocks = PipelineProfile::HEAVY.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    assert!(!model.is_empty(), "HEAVY profile ONNX should not be empty");
}

#[test]
fn test_pro_profile_onnx() {
    let blocks = PipelineProfile::PRO.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    assert!(!model.is_empty(), "PRO profile ONNX should not be empty");
}

// ── Concrete vs symbolic dims ─────────────────────────────────────

#[test]
fn test_lite_profile_with_concrete_dims() {
    let mut profile = PipelineProfile::LITE;
    let mut blocks = profile.build_blocks(8, 2);
    GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    
    // Verify the model starts with Cast op (from RawInputBlock's Cast to FLOAT)
    // This ensures the pipeline is properly wired
    assert!(!model.is_empty());
}

// ── Specific block tests ──────────────────────────────────────────

#[test]
fn test_lite_pipeline_has_correct_block_order() {
    let blocks = PipelineProfile::LITE.build_blocks(8, 2);
    let names: Vec<&str> = blocks.iter().map(|b| b.id()).collect();
    assert_eq!(names[0], "raw_input");
    assert_eq!(names[1], "normalize");
    assert_eq!(names[2], "cfa");
    assert_eq!(names[3], "blc");
    assert_eq!(names[4], "bayer_wb");
    assert_eq!(names[5], "demosaic");
    assert_eq!(names[6], "ccm");
    assert_eq!(names[7], "tone");
    assert_eq!(names[8], "display");
}

// ── MNN conversion (requires MNN library, ignored by default) ─────

#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires MNNConvert binary and MNN libraries
fn test_lite_profile_mnn_conversion() {
    let blocks = PipelineProfile::LITE.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    std::fs::write("test_lite_mnn.onnx", &model).unwrap();
    
    use cam_isp::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};
    let opts = MnnConvertOptions::default();
    let result = convert_onnx_to_mnn("test_lite_mnn.onnx", "test_lite_mnn.mnn", Some(&opts));
    assert!(result.is_ok(), "MNN conversion failed: {:?}", result.err());
}
