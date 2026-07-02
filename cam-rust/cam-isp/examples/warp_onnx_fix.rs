use cam_isp::{blocks::*, pipeline::{PipelineBuilder, IspBlock}, BayerPattern};

fn main() {
    // Simple pipeline: BayerUnpack → WarpGrid
    let mut pipeline = PipelineBuilder::new();
    let bayer = BayerProcBlock::new().with_bayer_pattern(BayerPattern::RGGB);
    let warp = WarpGridBlock::new(32, 32).with_grid(Some(vec![0.0f32; 32*32*2]));
    let display = DisplayBlock::new(32).rgba();

    pipeline.add_block(Box::new(bayer))
            .add_block(Box::new(warp))
            .add_block(Box::new(display));
    cam_isp::pipeline::wire_blocks(&mut pipeline.blocks);

    // Compose via GraphComposer
    let blocks: Vec<&dyn IspBlock> = pipeline.blocks.iter().map(|b| &**b).collect();
    let model = PipelineBuilder::compose_onnx(blocks.as_slice(), &[], 8, "WarpTest");
    std::fs::write("target/warp_test.onnx", model.onnx_bytes()).unwrap();
    println!("✅ ONNX Warp pipeline emitted: {} bytes", model.onnx_bytes().len());
}