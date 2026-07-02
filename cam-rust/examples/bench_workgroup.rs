use cam_isp::{blocks::*, pipeline::PipelineBuilder, GraphComposer};

fn create_pipeline(name: &str, wg_x: u32, wg_y: u32) -> Vec<u8> {
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
        let model = create_pipeline(name, wx, wy);
        std::fs::write(format!("target/workgroup/{}.onnx", name), model).unwrap();
        println!("✅ {} emitted", name);
    }
}
