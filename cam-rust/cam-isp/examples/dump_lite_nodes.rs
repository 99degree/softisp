use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;
use std::io::Write;

fn main() {
    let mut profile = PipelineProfile::LITE;
    profile.output_format = OutputFormat::FloatRgb;
    let mut blocks = profile.build_blocks(1280, 0);
    GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    // Dump raw ONNX bytes
    let mut f = std::fs::File::create("lite_1280.onnx").unwrap();
    f.write_all(&onnx).unwrap();
    eprintln!("Wrote {} bytes to lite_1280.onnx", onnx.len());

    // Also dump HEAVY for comparison
    let mut profile2 = PipelineProfile::HEAVY;
    profile2.output_format = OutputFormat::FloatRgb;
    let mut blocks2 = profile2.build_blocks(1280, 0);
    GraphComposer::wire_blocks(&mut blocks2);
    let block_refs2: Vec<&dyn IspBlock> = blocks2.iter().map(|b| b.as_ref()).collect();
    let onnx2 = GraphComposer::compose_from_vec(&block_refs2, &[], 21).unwrap();
    let mut f2 = std::fs::File::create("heavy_1280.onnx").unwrap();
    f2.write_all(&onnx2).unwrap();
    eprintln!("Wrote {} bytes to heavy_1280.onnx", onnx2.len());
}
