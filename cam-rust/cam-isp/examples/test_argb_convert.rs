use cam_isp::pipeline::IspBlock;
use cam_isp::engine::{ProcessParams, OutputFormat};
use cam_isp::profile::PipelineProfile;

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    
    // Test with Argb output format
    let mut profile = PipelineProfile::HEAVY;
    profile.output_format = OutputFormat::Argb;
    let mut blocks = profile.build_blocks(1920, 0);
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    engine.build(head, aux, None, 21).unwrap();
    let raw: Vec<u8> = vec![128u8; (1920 * 1080 * 2) as usize];
    let params = ProcessParams::new(1920, 1080, &raw);
    match engine.process(&params) {
        Ok(f) => println!("HEAVY Argb: OK {}×{} data={}", f.width, f.height, f.data.len()),
        Err(e) => println!("HEAVY Argb: ERR {:?}", e),
    }
}
