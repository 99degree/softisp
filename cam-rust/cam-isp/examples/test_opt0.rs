//! Test MNN conversion with optimize_level=0 at small resolution.

use cam_isp::blocks::*;
use cam_isp::pipeline::IspBlock;

fn try_convert(name: &str, blocks: Vec<Box<dyn IspBlock>>) {
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = match cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 13) {
        Ok(o) => o,
        Err(e) => {
            println!("  [{}] FAILED: {:?}", name, e);
            return;
        }
    };
    println!("  [{}] ONNX: {} bytes", name, onnx.len());

    let on = format!(".mnn_test_{}.onnx", name);
    let mn = format!(".mnn_test_{}.mnn", name);
    std::fs::write(&on, &onnx).unwrap();

    use cam_isp::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};
    let opts = MnnConvertOptions {
        optimize_level: 0,
        preserve_input_type: true,
        ..Default::default()
    };
    match convert_onnx_to_mnn(&on, &mn, Some(&opts)) {
        Ok(msg) => println!("  [{}] Convert OK: {}", name, msg),
        Err(e) => println!("  [{}] Convert FAILED: {}", name, e),
    }
    let _ = std::fs::remove_file(&on);
}

fn main() {
    cam_isp::init();

    // Small 64x64
    let w: i64 = 64;
    let h: i64 = 64;
    println!("=== 64x64 models ===\n");

    try_convert(
        "unpack_demosaic_display_64",
        vec![
            Box::new(UnpackBlock::new().with_concrete_dims(h, w)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(w as u32)),
        ],
    );

    try_convert(
        "full_64",
        vec![
            Box::new(UnpackBlock::new().with_concrete_dims(h, w)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(WarpGridBlock::new(w as u32, h as u32).with_gdc(-0.1, 0.0, 0.0)),
            Box::new(DisplayBlock::new(w as u32)),
        ],
    );

    // HD
    let w: i64 = 1280;
    let h: i64 = 720;
    println!("\n=== 1280x720 models ===\n");

    try_convert(
        "unpack_demosaic_display_hd",
        vec![
            Box::new(UnpackBlock::new().with_concrete_dims(h, w)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(w as u32)),
        ],
    );
}
