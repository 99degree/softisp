use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;

fn main() {
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

        eprintln!("=== {} Profile ===", prof_name);
        eprintln!("  Blocks: {}", blocks.len());
        for block in &blocks {
            let nodes = block.nodes();
            eprintln!("    {}: {} nodes", block.id(), nodes.len());
        }
    }
}
