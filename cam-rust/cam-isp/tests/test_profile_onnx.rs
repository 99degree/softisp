//! Test TEST profile ONNX generation and MNN conversion

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use cam_isp::pipeline::IspBlock;

#[cfg(feature = "mnn")]
use cam_isp::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};

#[test]
fn test_test_profile_onnx() {
    let blocks = PipelineProfile::TEST.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    std::fs::write("test_profile.onnx", &model).unwrap();
    assert!(!model.is_empty(), "TEST profile ONNX should not be empty");
    println!("TEST profile ONNX: {} bytes, {} blocks", model.len(), blocks.len());
    for b in &blocks {
        println!("  {}", b.id());
    }
}

#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires MNNConvert binary
fn test_test_profile_mnn_conversion() {
    let blocks = PipelineProfile::TEST.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    std::fs::write("test_profile.onnx", &model).unwrap();
    
    let opts = MnnConvertOptions::default();
    let result = convert_onnx_to_mnn("test_profile.onnx", "test_profile.mnn", Some(&opts));
    assert!(result.is_ok(), "MNN conversion failed: {:?}", result.err());
    println!("MNN conversion succeeded: {:?}", result.ok());
}
