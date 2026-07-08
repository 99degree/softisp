use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;
use cam_isp::engine::ProcessParams;

fn try_blocks(block_ids: &[&str]) {
    let w = 1920u32;
    let h = 1080u32;
    let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();

    for id in block_ids {
        match *id {
            "raw_input" => blocks.push(Box::new(cam_isp::blocks::RawInputBlock::new().with_elem_type(6).with_concrete_width(960))),
            "unpack_cfa" => blocks.push(Box::new(cam_isp::blocks::UnpackCfaBlock::new().with_concrete_width(1920).with_blc(true))),
            "blc" => blocks.push(Box::new(cam_isp::blocks::BlcBlock::new())),
            "cfa" => blocks.push(Box::new(cam_isp::blocks::CfaBlock::new())),
            "demosaic_ccm" => blocks.push(Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0))),
            "normalize" => blocks.push(Box::new(cam_isp::blocks::NormalizeBlock::new())),
            "ee" => blocks.push(Box::new(cam_isp::blocks::EeBlock::new())),
            "fcs" => blocks.push(Box::new(cam_isp::blocks::FcsBlock::new())),
            "ldci" => blocks.push(Box::new(cam_isp::blocks::LdciBlock::new())),
            "display" => blocks.push(Box::new(cam_isp::blocks::DisplayBlock::new(1920))),
            _ => {}
        }
    }

    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;

    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    match engine.build(head, aux, None, 21) {
        Ok(_) => {}
        Err(e) => { println!("  {:30} build FAILED: {:?}", block_ids.join("→"), e); return; }
    }

    let raw: Vec<u8> = vec![128u8; (w * h * 2) as usize];
    let params = ProcessParams::new(w, h, &raw);
    match engine.process(&params) {
        Ok(f) => println!("  {:30} OK: {}×{} data={}", block_ids.join("→"), f.width, f.height, f.data.len()),
        Err(e) => println!("  {:30} ERR: {:?}", block_ids.join("→"), e),
    }
}

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    try_blocks(&["raw_input", "unpack_cfa", "demosaic_ccm", "display"]);
    try_blocks(&["raw_input", "unpack_cfa", "demosaic_ccm", "ee", "display"]);
    try_blocks(&["raw_input", "unpack_cfa", "demosaic_ccm", "ee", "fcs", "display"]);
    try_blocks(&["raw_input", "unpack_cfa", "demosaic_ccm", "ee", "fcs", "ldci", "display"]);
    try_blocks(&["raw_input", "unpack_cfa", "demosaic_ccm", "ee", "fcs", "ldci", "normalize", "display"]);
    try_blocks(&["raw_input", "unpack_cfa", "blc", "cfa", "demosaic_ccm", "display"]);
}
