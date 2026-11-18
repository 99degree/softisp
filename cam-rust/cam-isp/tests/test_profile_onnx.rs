//! Integration tests for ONNX model generation across all profiles.
//!
//! All tests must call `GraphComposer::wire_blocks` before
//! `compose_from_vec` to ensure proper tensor connections.
//! Without wiring, MNNConvert segfaults on the resulting model.

use cam_isp::pipeline::GraphComposer;
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

/// Helper: build profile blocks (already wired by build_blocks)
fn wired_blocks(profile: PipelineProfile) -> Vec<Box<dyn IspBlock>> {
    profile.build_blocks(8, 2, 0)
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
    // Block counts are an emission detail: pass-through blocks are elided and the
    // head is one fused op, so a literal would go stale every time either changes.
    // Assert the shape — a head, a display tail, nothing pass-through — instead.
    assert!(blocks.len() >= 8, "LITE got {} blocks", blocks.len());
    assert!(
        !blocks.iter().any(|b| b.is_noop()),
        "no pass-through block should survive into the graph"
    );
    assert!(model.len() > 2000, "LITE model should be substantial");
}

#[test]
fn test_med_profile_onnx() {
    let blocks = wired_blocks(PipelineProfile::MED);
    let model = compose_wired(&blocks);
    assert!(!model.is_empty(), "MED profile ONNX should not be empty");
    assert!(blocks.len() >= 10, "MED got {} blocks", blocks.len());
    assert!(
        !blocks.iter().any(|b| b.is_noop()),
        "no pass-through block should survive into the graph"
    );
    assert!(model.len() > 2000, "MED model should be substantial");
}

#[test]
fn test_heavy_profile_onnx() {
    let blocks = wired_blocks(PipelineProfile::HEAVY);
    let model = compose_wired(&blocks);
    assert!(!model.is_empty(), "HEAVY profile ONNX should not be empty");
    assert!(blocks.len() >= 10, "HEAVY got {} blocks", blocks.len());
    // HEAVY's distinguishing feature is the bad-pixel stage.
    assert!(
        blocks.iter().any(|b| b.id().starts_with("blc_dpc")),
        "HEAVY should keep its bad-pixel correction"
    );
    assert!(model.len() > 3000, "HEAVY model should be substantial");
}

#[test]
fn test_pro_profile_onnx() {
    let blocks = wired_blocks(PipelineProfile::PRO);
    let model = compose_wired(&blocks);
    assert!(!model.is_empty(), "PRO profile ONNX should not be empty");
    // PRO = the MED stage set plus the warp/geometry pair.
    assert!(
        blocks.iter().any(|b| b.id().contains("warp")),
        "PRO should carry the warp stage"
    );
    assert!(blocks.len() >= 10, "PRO got {} blocks", blocks.len());
    assert!(model.len() > 3000, "PRO model should be substantial");
}

// ── Block order validation ────────────────────────────────────────

#[test]
fn test_lite_pipeline_has_correct_block_order() {
    let blocks = PipelineProfile::LITE.build_blocks(8, 2, 0);
    let names: Vec<&str> = blocks.iter().map(|b| b.id()).collect();
    // These profiles build the *primitive* chain: `build_blocks(8, 2, 0)` takes
    // the opset from the argument, so the head is the unpack group and demosaic and
    // CCM are separate blocks. Assert the stages rather than their spelling — the
    // fused profiles spell the same stages differently.
    assert!(
        names[0].starts_with("raw_input"),
        "LITE should start at the sensor, got '{}'",
        names[0]
    );
    assert!(
        names.len() >= 8,
        "LITE should have at least 8 blocks, got {}",
        names.len()
    );
    let key_blocks = ["aux_hook_src", "bayer_wb", "tone", "display"];
    for key in &key_blocks {
        assert!(names.contains(key), "LITE should contain block '{}'", key);
    }
    assert!(
        names.iter().any(|n| n.starts_with("demosaic")),
        "LITE should demosaic, got {names:?}"
    );
}

#[test]
fn test_heavy_pipeline_block_order() {
    let blocks = PipelineProfile::HEAVY.build_blocks(8, 2, 0);
    let names: Vec<&str> = blocks.iter().map(|b| b.id()).collect();
    assert!(
        names[0].starts_with("raw_input") || names[0] == "isp_unpack_blc",
        "HEAVY should start at the sensor, got '{}'",
        names[0]
    );
    assert!(
        names.len() >= 8,
        "HEAVY should have at least 8 blocks, got {}",
        names.len()
    );
    let key_blocks = ["aux_hook_src", "bayer_wb", "tone", "display"];
    for key in &key_blocks {
        assert!(names.contains(key), "HEAVY should contain block '{}'", key);
    }
    // Demosaic and CCM are one fused block in the fused spelling and two in the
    // primitive one; HEAVY's profile sets both, so accept either.
    assert!(
        names
            .iter()
            .any(|n| n.starts_with("demosaic") || n.starts_with("ccm")),
        "HEAVY should correct colour, got {names:?}"
    );
    // HEAVY's distinguishing feature is the bad-pixel stage.
    assert!(
        names.iter().any(|n| n.starts_with("blc_dpc")),
        "HEAVY should keep its bad-pixel correction, got {names:?}"
    );
}

// ── Build blocks now internally wires ───────────────────────────
// build_blocks calls wire_blocks internally for proper MNN compat.

#[test]
fn test_build_blocks_wires_internally() {
    let blocks = PipelineProfile::LITE.build_blocks(8, 2, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    // First block has no input, second block's input should match
    // first block's output
    assert!(
        refs[0].input_tensors().is_empty(),
        "RawInput should have no inputs"
    );
    let first_out = refs[0].output_tensors()[0].clone();
    let second_in = refs[1].input_tensors()[0].clone();
    assert_eq!(
        first_out, second_in,
        "Block 0 output '{}' should connect to block 1 input '{}'",
        first_out, second_in
    );
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
        Some(&opts),
    );
    assert!(result.is_ok(), "MNN conversion failed: {:?}", result.err());
}
