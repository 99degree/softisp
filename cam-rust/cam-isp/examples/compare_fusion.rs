use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    let profiles = [
        ("LITE", PipelineProfile::LITE),
        ("HEAVY", PipelineProfile::HEAVY),
    ];

    for (prof_name, mut profile) in profiles {
        profile.output_format = OutputFormat::FloatRgb;
        let mut blocks = profile.build_blocks(1280, 0);
        GraphComposer::wire_blocks(&mut blocks);

        let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let _onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

        let mut block_vec = blocks;
        let head = block_vec.remove(0);
        let aux = block_vec;
        let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();

        eprintln!("=== {} Profile ===", prof_name);
        engine.build(head, aux, None, 21).unwrap();
        eprintln!("  Engine built successfully");
    }
}
