use cam_isp::engine::ProcessParams;
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

fn run_test(label: &str, mut blocks: Vec<Box<dyn IspBlock>>) {
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    // Save ONNX for inspection
    let path = std::env::temp_dir().join(format!("{}.onnx", label.replace(' ', "_")));
    let _ = std::fs::write(&path, &onnx);
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    engine.build(head, aux, None, 21).unwrap();
    let raw: Vec<u8> = vec![128u8; (1920 * 1080 * 2) as usize];
    let params = ProcessParams::new(1920, 1080, &raw);
    match engine.process(&params) {
        Ok(f) => println!("{:50} OK: {}×{}", label, f.width, f.height),
        Err(e) => println!("{:50} ERR: {:?}", label, e),
    }
}

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    // 9 blocks: works
    run_test(
        "9blk",
        vec![
            Box::new(
                cam_isp::blocks::RawInputBlock::new()
                    .with_elem_type(6)
                    .with_concrete_width(960)
                    .with_concrete_height(1080),
            ),
            Box::new(
                cam_isp::blocks::UnpackCfaBlock::new()
                    .with_concrete_width(1920)
                    .with_blc(true),
            ),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_src")),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::IdentityBlock::new("tone")),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_out")),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::LdciBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // 10 blocks (+CCM): works
    run_test(
        "10blk_ccm",
        vec![
            Box::new(
                cam_isp::blocks::RawInputBlock::new()
                    .with_elem_type(6)
                    .with_concrete_width(960)
                    .with_concrete_height(1080),
            ),
            Box::new(
                cam_isp::blocks::UnpackCfaBlock::new()
                    .with_concrete_width(1920)
                    .with_blc(true),
            ),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_src")),
            Box::new(cam_isp::blocks::CcmBlock::new()),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::IdentityBlock::new("tone")),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_out")),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::LdciBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // 11 blocks (+BayerWb): fails
    run_test(
        "11blk_wb",
        vec![
            Box::new(
                cam_isp::blocks::RawInputBlock::new()
                    .with_elem_type(6)
                    .with_concrete_width(960)
                    .with_concrete_height(1080),
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
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::LdciBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );

    // 11 blocks without WB but with WB gains fused into UnpackCfaBlock
    run_test(
        "11blk_wb_in_unpacked",
        vec![
            Box::new(
                cam_isp::blocks::RawInputBlock::new()
                    .with_elem_type(6)
                    .with_concrete_width(960)
                    .with_concrete_height(1080),
            ),
            Box::new(
                cam_isp::blocks::UnpackCfaBlock::new()
                    .with_concrete_width(1920)
                    .with_blc(true),
            ),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_src")),
            Box::new(cam_isp::blocks::CcmBlock::new()),
            Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
            Box::new(cam_isp::blocks::IdentityBlock::new("tone")),
            Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_out")),
            Box::new(cam_isp::blocks::EeBlock::new()),
            Box::new(cam_isp::blocks::FcsBlock::new()),
            Box::new(cam_isp::blocks::LdciBlock::new()),
            Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
        ],
    );
}
