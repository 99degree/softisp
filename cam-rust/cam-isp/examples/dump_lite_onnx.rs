use cam_isp::profile::PipelineProfile;
use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::{GraphComposer, IspBlock};

fn main() {
    let mut profile = PipelineProfile::LITE;
    profile.output_format = OutputFormat::FloatRgb;
    let mut blocks = profile.build_blocks(1280, 0);
    GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    eprintln!("=== LITE ONNX Graph ===");
    eprintln!("ONNX model size: {} bytes", onnx.len());
    eprintln!("Block chain:");
    for (idx, blk) in block_refs.iter().enumerate() {
        let ins = blk.input_tensors().join(",");
        let outs = blk.output_tensors().join(",");
        eprintln!("  [{}] {} in=[{}] out=[{}]", idx, blk.id(), ins, outs);
    }
    
    // Pipeline summary
    eprintln!("\nPipeline summary:");
    eprintln!("  {} blocks, {} nodes, {} bytes ONNX",
        block_refs.len(),
        block_refs.iter().map(|b| b.nodes().len()).sum::<usize>(),
        onnx.len());
}
