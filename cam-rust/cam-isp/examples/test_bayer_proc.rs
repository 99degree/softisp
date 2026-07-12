use cam_isp::blocks::{BayerMode, BayerPattern, BayerProcBlock, RawInputBlock};
use cam_isp::pipeline::GraphComposer;
use std::fs;

fn main() {
    let mut blocks: Vec<Box<dyn cam_isp::pipeline::IspBlock>> = Vec::new();

    blocks.push(Box::new(
        RawInputBlock::new()
            .with_elem_type(5) // INT16
            .with_concrete_dims(48, 64),
    ));

    // Test FullProc mode - generates SpaceToDepthEx
    blocks.push(Box::new(
        BayerProcBlock::new()
            .with_mode(BayerMode::FullProc)
            .with_bayer_pattern(BayerPattern::Rggb)
            .with_concrete_dims(48, 64)
            .with_blc(true)
            .with_wb(true),
    ));

    GraphComposer::wire_blocks(&mut blocks);

    let refs: Vec<&dyn cam_isp::pipeline::IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&refs, &[], 5).unwrap();

    println!("Model size: {} bytes", model.len());

    // Save to file for inspection
    fs::write(
        "/data/data/com.termux/files/home/softisp/bayer_model.onnx",
        &model,
    )
    .unwrap();
    println!("Model saved to bayer_model.onnx");
}
