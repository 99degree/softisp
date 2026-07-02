use cam_isp::{blocks::*, pipeline::PipelineBuilder, BayerPattern, IspBlock};

fn main() {
    // Simple pipeline: BayerProc → WarpGrid (with identity grid) — emits GridSample node
    let mut pipeline = Pipeline::builder();
    let bayer = BayerProcBlock::new().with_bayer_pattern(BayerPattern::RBGG);
    let warp = WarpGridBlock::new(64, 64)
        .with_bcs(0.0, 1.0, 1.0)
        .with_grid(Some(vec![0.0f32; 64*64*2])); // identity grid [-1,1] flattened
    let display = DisplayBlock::new(64).rgba();

    // Build & wire
    pipeline.add_block(Box::new(bayer))
            .add_block(Box::new(warp))
            .add_block(Box::new(display))
            .wire_chain();

    // Compose ONNX via static
    let blocks: Vec<&dyn IspBlock> = pipeline.blocks.iter().map(|b| &**b).collect();
    if blocks.is_empty() { panic!("No blocks"); }
    let model = Pipeline::compose(blocks.as_slice(), &[], 8, "WarpONNX");
    std::fs::write("target/warp_test.onnx", model.onnx_bytes()).unwrap();
    println!("✅ ONNX grid warp pipeline: {} bytes", model.onnx_bytes().len());
}