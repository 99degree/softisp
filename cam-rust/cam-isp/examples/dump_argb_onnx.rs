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
    let mut f = std::fs::File::create("debug_argb.onnx").unwrap();
    f.write_all(&onnx).unwrap();
    eprintln!("Wrote debug_argb.onnx ({} bytes)", onnx.len());
}
