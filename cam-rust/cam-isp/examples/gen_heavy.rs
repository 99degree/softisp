fn main() {
    use cam_isp::profile::PipelineProfile;
    use cam_isp::pipeline::{IspBlock, GraphComposer};
    let blocks = PipelineProfile::HEAVY.build_blocks(8, 2);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let m = GraphComposer::compose_from_vec(&refs, &[], 16).unwrap();
    std::fs::write("heavy_profile.onnx", &m).unwrap();
    println!("OK: {} bytes", m.len());
}
