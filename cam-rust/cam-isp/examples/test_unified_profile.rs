use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;
use cam_isp::engine::ProcessParams;

fn run_test(label: &str, profile: PipelineProfile, w: u32, h: u32) {
    let blocks = profile.build_blocks(w, 0);
    println!("{}: {} blocks: {}", label, blocks.len(), blocks.iter().map(|b| b.id()).collect::<Vec<_>>().join(" → "));
    
    let mut blocks = blocks;
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    engine.build(head, aux, None, 21).unwrap();
    let raw: Vec<u8> = vec![128u8; (w as usize * h as usize * 2) as usize];
    let params = ProcessParams::new(w, h, &raw);
    match engine.process(&params) {
        Ok(f) => println!("  OK: {}×{} data={}", f.width, f.height, f.data.len()),
        Err(e) => println!("  ERR: {:?}", e),
    }
}

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    
    // Test all profiles
    for profile in &PipelineProfile::ALL {
        run_test(profile.label, *profile, 1920, 1080);
    }
}
