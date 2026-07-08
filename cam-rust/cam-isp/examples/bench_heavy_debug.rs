use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;
use cam_isp::engine::ProcessParams;

fn build_and_run(engine_name: &str) {
    let w = 1920u32;
    let h = 1080u32;
    let profile = PipelineProfile::HEAVY;
    let blocks = profile.build_blocks(w, 0);

    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;

    let mut engine = cam_isp::engine::select_engine_by_name(engine_name).unwrap();
    match engine.build(head, aux, None, 21) {
        Ok(_) => println!("  {} build OK", engine_name),
        Err(e) => { println!("  {} build FAILED: {:?}", engine_name, e); return; }
    }

    let raw: Vec<u8> = vec![128u8; (w * h * 2) as usize];
    let params = ProcessParams::new(w, h, &raw);
    match engine.process(&params) {
        Ok(f) => println!("  {} OK: {}×{} data={}B float={}", engine_name, f.width, f.height, f.data.len(),
            f.float_data.as_ref().map_or(0, |v| v.len())),
        Err(e) => println!("  {} ERR: {:?}", engine_name, e),
    }
}

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Cpu);
    build_and_run("mnn_vulkan");
    build_and_run("mnn_cpu");
}
