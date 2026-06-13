// Test file to generate TEST profile ONNX
#[test]
fn test_generate_test_profile_onnx() {
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::profile::PipelineProfile;
    use cam_isp::pipeline::IspBlock;
    
    let blocks = PipelineProfile::TEST.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    std::fs::write("test_profile.onnx", &model).unwrap();
    println!("TEST profile ONNX: {} bytes, {} blocks", model.len(), blocks.len());
    for b in &blocks {
        println!("  {}", b.name());
    }
}
