use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;

fn main() {
    let profiles = [
        ("LITE", PipelineProfile::LITE),
        ("HEAVY", PipelineProfile::HEAVY),
    ];

    for (name, mut profile) in profiles {
        profile.output_format = OutputFormat::FloatRgb;
        let mut blocks = profile.build_blocks(1920, 0);
        GraphComposer::wire_blocks(&mut blocks);

        let block_refs: Vec<&dyn cam_isp::pipeline::IspBlock> =
            blocks.iter().map(|b| b.as_ref()).collect();
        let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

        eprintln!("=== {} Profile ===", name);
        eprintln!("  ONNX size: {} bytes", onnx.len());

        // Count blocks
        for block in &blocks {
            let nodes = block.nodes();
            let inits = block.extra_input_defaults();
            eprintln!(
                "  Block: {} - nodes: {}, initializers: {}",
                block.id(),
                nodes.len(),
                inits.len()
            );
        }
    }
}
