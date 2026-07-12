use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;
use std::io::Write;

fn main() {
    let mut profile = PipelineProfile::LITE;
    profile.output_format = OutputFormat::Argb;
    let mut blocks = profile.build_blocks(1920, 0);
    GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    // Find all value_info for DisplayBlock/frame
    let onnx_str = String::from_utf8_lossy(&onnx);
    // Look for tensor dimensions
    for block in &blocks {
        if let Some(name) = block.graph_output_name() {
            eprintln!("Block {} output: {}", block.id(), name);
            if let Some(vi) = block.output_value_info() {
                eprintln!("  value_info bytes: {:?}", &vi[..vi.len().min(200)]);
            }
        }
    }
}
