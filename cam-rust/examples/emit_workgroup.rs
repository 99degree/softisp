use cam_isp::{blocks::*, pipeline::PipelineBuilder, GraphComposer};

    let mut pipeline = PipelineBuilder::new();
    pipeline.add_display_rgba(1920)
            .workgroup(wg_x, wg_y);
    let blocks: Vec<&dyn cam_isp::IspBlock> = pipeline.blocks.iter().map(|b| &**b).collect();
    let model = GraphComposer::compose(blocks.as_slice(), &[], 8, name);
    model.onnx_bytes()
}

fn main() {
    std::fs::create_dir_all("target/workgroup").unwrap();
    let variants = vec![
        ("wg16x16", 16, 16),
        ("wg32x8",  32, 8),
        ("wg16x32", 16, 32),
        ("wg32x32", 32, 32),
        ("wg64x4",  64, 4)
    ];

    for (name, wx, wy) in variants {
        std::fs::write(format!("target/workgroup/{}.onnx", name), model).unwrap();
        println!("✅ {} emitted", name);
    }
}
fn main() {
    let variants = vec!("wg32x8", "wg16x16");
    for name in variants {
        let pipeline = PipelineBuilder::new().add_display_rgba(1920).rgba();
        let blocks: Vec<&dyn cam_isp::IspBlock> = vec![&*pipeline.blocks[0]];
        let model = GraphComposer::compose(blocks.as_slice(), &[], 8, "Display");
        
        // Inject workgroup tuning
        if let Some(extra) = model.extra_op_mut("isp.display") {
            extra.emit_attribute("group_size", &[match name {
                "wg32x8" => 32, "wg16x16" => 16, _ => 16},
                match name {
                "wg32x8" => 8, "wg16x16" => 16, _ => 16},
                1
            ]);
        }
        
        std::fs::write(format!("target/display_{}.onnx", name), model.onnx_bytes()).unwrap();
        println!("✅ {} emitted", name);
    }
}
