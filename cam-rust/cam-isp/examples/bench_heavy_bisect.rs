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

    // Get the actual HEAVY blocks
    let profile = PipelineProfile::HEAVY;
    let blocks = profile.build_blocks(1920, 0);
    println!("HEAVY blocks ({}):", blocks.len());
    for b in &blocks {
        println!("  {}", b.id());
    }

    // Bisect: try first N blocks + display
    let display = Box::new(cam_isp::blocks::DisplayBlock::new(1920));

    for n in 1..=blocks.len() {
        let mut subset: Vec<Box<dyn IspBlock>> = blocks[..n]
            .iter()
            .map(|b| {
                // Rebuild each block by its id
                match b.id() {
                    "raw_input" => Box::new(
                        cam_isp::blocks::RawInputBlock::new()
                            .with_elem_type(6)
                            .with_concrete_width(960),
                    ) as Box<dyn IspBlock>,
                    "unpack_cfa" => Box::new(
                        cam_isp::blocks::UnpackCfaBlock::new()
                            .with_concrete_width(1920)
                            .with_blc(true),
                    ),
                    "blc" => Box::new(cam_isp::blocks::BlcBlock::new()),
                    "normalize" => Box::new(cam_isp::blocks::NormalizeBlock::new()),
                    "cfa" => Box::new(cam_isp::blocks::CfaBlock::new()),
                    "demosaic_ccm" => Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0)),
                    "ee" => Box::new(cam_isp::blocks::EeBlock::new()),
                    "fcs" => Box::new(cam_isp::blocks::FcsBlock::new()),
                    "ldci" => Box::new(cam_isp::blocks::LdciBlock::new()),
                    "display" => Box::new(cam_isp::blocks::DisplayBlock::new(1920)),
                    _ => {
                        println!("  UNKNOWN block: {}", b.id());
                        Box::new(cam_isp::blocks::IdentityBlock::new("unknown"))
                    }
                }
            })
            .collect();

        // Add display at end if not already there
        if subset.last().map_or(true, |b| b.id() != "display") {
            subset.push(Box::new(cam_isp::blocks::DisplayBlock::new(1920)));
        }

        let label = subset.iter().map(|b| b.id()).collect::<Vec<_>>().join("→");
        try_blocks(&label, subset);
    }
}
