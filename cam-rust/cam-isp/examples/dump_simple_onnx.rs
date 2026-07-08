use cam_isp::pipeline::IspBlock;

fn main() {
    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(cam_isp::blocks::UnpackBlock::new().with_concrete_dims(1080, 1920)),
        Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
        Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
    ];
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    println!("Simple ONNX size: {} bytes", onnx.len());
    let path = std::env::temp_dir().join("simple_model.onnx");
    std::fs::write(&path, &onnx).unwrap();
    println!("Saved to {}", path.display());
    
    // Now dump the heavy model
    use cam_isp::profile::PipelineProfile;
    let mut blocks2 = PipelineProfile::HEAVY.build_blocks(1920, 0);
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks2);
    let block_refs2: Vec<&dyn IspBlock> = blocks2.iter().map(|b| b.as_ref()).collect();
    let onnx2 = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs2, &[], 21).unwrap();
    println!("Heavy ONNX size: {} bytes", onnx2.len());
    let path2 = std::env::temp_dir().join("heavy_model.onnx");
    std::fs::write(&path2, &onnx2).unwrap();
    println!("Saved to {}", path2.display());
}
