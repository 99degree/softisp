use cam_isp::{blocks::*, pipeline::PipelineBuilder, GraphComposer, Proto};

fn main() {
    let mut pipeline = PipelineBuilder::new();
    let display = DisplayBlock::new(1920)
        .rgba(); // RGBA output
    pipeline.add_block(Box::new(display));

    // Tune workgroup: 32×8 for 4K→FHD
    let blocks: Vec<&dyn cam_isp::IspBlock> = pipeline.blocks.iter().map(|b| &**b).collect();
    let model = GraphComposer::compose(blocks.as_slice(), &[], 8, "Display");
    
    // Inject group_size attribute
    let extra = model.extra_op("isp.display");
    if let Some(extra) = extra {
        let mut new_attr = Proto::attribute_ints("group_size", &[32, 8, 1]);
        *(extra.attr_mut().last_mut().unwrap()) = new_attr;
    }
    
    std::fs::write("target/display_wg32x8.onnx", model.onnx_bytes()).unwrap();
    println!("✅ ONNX display with 32×8 workgroup emitted");
}
