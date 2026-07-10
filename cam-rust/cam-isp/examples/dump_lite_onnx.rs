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

    let graph = onnx.graph.as_ref().unwrap();
    eprintln!("=== LITE ONNX Graph ===");
    eprintln!("Nodes: {}", graph.node.len());
    for (idx, node) in graph.node.iter().enumerate() {
        let inputs: Vec<&str> = node.input.iter().map(|s| s.as_str()).collect();
        let outputs: Vec<&str> = node.output.iter().map(|s| s.as_str()).collect();
        eprintln!("  [{}] {} in=[{}] out=[{}]",
            idx, node.op_type, inputs.join(","), outputs.join(","));
    }
    eprintln!("Inputs: {}", graph.input.len());
    for inp in &graph.input {
        let dims: Vec<i64> = inp.r#type.as_ref()
            .and_then(|t| t.tensor_type.as_ref())
            .and_then(|tt| tt.shape.as_ref())
            .map(|s| s.dim.iter().map(|d| d.dim_value).collect())
            .unwrap_or_default();
        eprintln!("  {} {:?}", inp.name, dims);
    }
    eprintln!("Outputs: {}", graph.output.len());
    for out in &graph.output {
        eprintln!("  {}", out.name);
    }
}
