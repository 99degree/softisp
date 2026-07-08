use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;
use cam_isp::engine::ProcessParams;

fn try_blocks(label: &str, mut blocks: Vec<Box<dyn IspBlock>>) {
    // Wire blocks so input_source is set correctly
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    match engine.build(head, aux, None, 21) {
        Ok(_) => {}
        Err(e) => { println!("  {:40} BUILD FAIL: {:?}", label, e); return; }
    }
    let raw: Vec<u8> = vec![128u8; (1920 * 1080 * 2) as usize];
    let params = ProcessParams::new(1920, 1080, &raw);
    match engine.process(&params) {
        Ok(f) => println!("  {:40} OK: {}×{} data={}", label, f.width, f.height, f.data.len()),
        Err(e) => println!("  {:40} ERR: {:?}", label, e),
    }
}

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    
    // 12 blocks matching HEAVY exactly
    let profile = PipelineProfile::HEAVY;
    let blocks = profile.build_blocks(1920, 0);
    println!("HEAVY blocks ({}):", blocks.len());
    for b in &blocks { println!("  {}", b.id()); }
    
    try_blocks("HEAVY profile", blocks);
}
