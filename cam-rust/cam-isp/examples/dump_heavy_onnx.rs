use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

fn main() {
    let profile = PipelineProfile::HEAVY;
    let mut blocks = profile.build_blocks(1920, 0);
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    println!("ONNX model size: {} bytes", onnx.len());
    let path = std::env::temp_dir().join("heavy_model.onnx");
    std::fs::write(&path, &onnx).unwrap();
    println!("Saved to {}", path.display());
}
