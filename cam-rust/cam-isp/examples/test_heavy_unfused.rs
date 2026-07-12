use cam_isp::engine::ProcessParams;
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

fn try_blocks(label: &str, blocks: Vec<Box<dyn IspBlock>>) {
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    match engine.build(head, aux, None, 21) {
        Ok(_) => {}
        Err(e) => {
            println!("  {:40} BUILD FAIL: {:?}", label, e);
            return;
        }
    }
    let raw: Vec<u8> = vec![128u8; (1920 * 1080 * 2) as usize];
    let params = ProcessParams::new(1920, 1080, &raw);
    match engine.process(&params) {
        Ok(f) => println!(
            "  {:40} OK: {}×{} data={}",
            label,
            f.width,
            f.height,
            f.data.len()
        ),
        Err(e) => println!("  {:40} ERR: {:?}", label, e),
    }
}

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    // Test 1: HEAVY with unfused unpack (no use_fused_unpack)
    // Build manually: raw_input + unpack + normalize + blc + cfa + identity + ccm + wb + demosaic_ccm + tone + identity + fcs + ldci + ee + display
    try_blocks(
        "unfused heavy",
        vec![
            Box::new(
                cam_isp::blocks::RawInputBlock::new()
                    .with_elem_type(6)
                    .with_concrete_width(960),
            ),
            Box::new(cam_isp::blocks::UnpackBlock::new().with_concrete_dims(1080, 1920)),
            Box::new(cam_isp::blocks::NormalizeBlock::new()),
            Box::new(cam_isp::blocks::BlcBlock::new()),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_src")),
            Box::new(cam_isp::blocks::CcmBlock::new()),
            Box::new(cam_isp::blocks::BayerWbBlock::new()),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::IdentityBlock::new("tone")),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_out")),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::LdciBlock::new()),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // Test 2: HEAVY with fused unpack (UnpackCfaBlock)
    try_blocks(
        "fused heavy",
        vec![
            Box::new(
                cam_isp::blocks::RawInputBlock::new()
                    .with_elem_type(6)
                    .with_concrete_width(960),
            ),
            Box::new(
                cam_isp::blocks::UnpackCfaBlock::new()
                    .with_concrete_width(1920)
                    .with_blc(true),
            ),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_src")),
            Box::new(cam_isp::blocks::CcmBlock::new()),
            Box::new(cam_isp::blocks::BayerWbBlock::new()),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::IdentityBlock::new("tone")),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_out")),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::LdciBlock::new()),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // Test 3: Remove identity blocks
    try_blocks(
        "no identities",
        vec![
            Box::new(
                cam_isp::blocks::RawInputBlock::new()
                    .with_elem_type(6)
                    .with_concrete_width(960),
            ),
            Box::new(
                cam_isp::blocks::UnpackCfaBlock::new()
                    .with_concrete_width(1920)
                    .with_blc(true),
            ),
            Box::new(cam_isp::blocks::CcmBlock::new()),
            Box::new(cam_isp::blocks::BayerWbBlock::new()),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::LdciBlock::new()),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );
}
