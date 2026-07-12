use cam_isp::engine::OutputFormat;
use cam_isp::engine::ProcessParams;
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

fn try_blocks(label: &str, blocks: Vec<Box<dyn IspBlock>>) {
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    match engine.build(head, aux, None, 21) {
        Ok(_) => {}
        Err(e) => {
            println!("  {:30} BUILD FAIL: {:?}", label, e);
            return;
        }
    }

    let raw: Vec<u8> = vec![128u8; (1920 * 1080 * 2) as usize];
    let params = ProcessParams::new(1920, 1080, &raw);
    match engine.process(&params) {
        Ok(f) => println!(
            "  {:30} OK: {}×{} data={}",
            label,
            f.width,
            f.height,
            f.data.len()
        ),
        Err(e) => println!("  {:30} ERR: {:?}", label, e),
    }
}

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Cpu);

    // Simple 3-block (works)
    try_blocks(
        "unpack+demosaic+display",
        vec![
            Box::new(cam_isp::blocks::UnpackBlock::new().with_concrete_dims(1080, 1920)),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // Add EE (one at a time)
    try_blocks(
        "+ee",
        vec![
            Box::new(cam_isp::blocks::UnpackBlock::new().with_concrete_dims(1080, 1920)),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // Add FCS
    try_blocks(
        "+fcs",
        vec![
            Box::new(cam_isp::blocks::UnpackBlock::new().with_concrete_dims(1080, 1920)),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // Add LDCI
    try_blocks(
        "+ldci",
        vec![
            Box::new(cam_isp::blocks::UnpackBlock::new().with_concrete_dims(1080, 1920)),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::LdciBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // Full HEAVY profile
    let profile = PipelineProfile::HEAVY;
    let blocks = profile.build_blocks(1920, 0);
    println!("\nHEAVY profile blocks ({}):", blocks.len());
    for b in &blocks {
        println!("  {}", b.id());
    }
    try_blocks("HEAVY full", blocks);
}
