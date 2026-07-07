//! Bisect which block causes MNN converter segfault.
//!
//! Tests each block combination to find the minimal failing set.

use cam_isp::blocks::*;
use cam_isp::pipeline::IspBlock;

fn try_build(name: &str, blocks: Vec<Box<dyn IspBlock>>, engine_name: &str, opset: i64, opt_level: u8) {
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = match cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], opset) {
        Ok(o) => o,
        Err(e) => { println!("  [{}] ONNX compose FAILED: {:?}", name, e); return; }
    };
    println!("  [{}] ONNX: {} bytes, {} blocks, opset={}, opt={}", name, onnx.len(), blocks.len(), opset, opt_level);

    let mut engine = match cam_isp::engine::select_engine_by_name(engine_name) {
        Some(e) => e,
        None => { println!("  [{}] engine '{}' not found", name, engine_name); return; }
    };

    let mut b = blocks;
    let head = b.remove(0);
    let tail = b;
    match engine.build(head, tail, None, 21) {
        Ok(()) => println!("  [{}] Build OK!", name),
        Err(e) => println!("  [{}] Build FAILED: {:?}", name, e),
    }
}

fn main() {
    cam_isp::init();
    cam_isp::cpu::register_cpu_engine();
    #[cfg(feature = "mnn")]
    {
        cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
        cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Cpu);
    }
    let engine_name = std::env::var("ENGINE").unwrap_or_else(|_| "mnn_cpu".to_string());

    let w: u32 = 1280;
    let h: u32 = 720;

    println!("=== Block-by-block MNN conversion test ===\n");

    // Test both opset 13 and opset 21
    for &opset in &[13i64, 21] {
        println!("--- opset {} ---", opset);

        // 1. Unpack only
        try_build("unpack_only", vec![
            Box::new(UnpackBlock::new().with_concrete_dims(h as i64, w as i64)),
        ], &engine_name, opset);

        // 2. Unpack + DemosaicCcm
        try_build("unpack+demosaic", vec![
            Box::new(UnpackBlock::new().with_concrete_dims(h as i64, w as i64)),
            Box::new(DemosaicCcmBlock::new(0)),
        ], &engine_name, opset);

        // 3. Unpack + DemosaicCcm + Display
        try_build("unpack+demosaic+display", vec![
            Box::new(UnpackBlock::new().with_concrete_dims(h as i64, w as i64)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(w)),
        ], &engine_name, opset);

        // 4. Unpack + DemosaicCcm + WarpGrid + Display (full pipeline)
        try_build("full_4blocks", vec![
            Box::new(UnpackBlock::new().with_concrete_dims(h as i64, w as i64)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(WarpGridBlock::new(w, h).with_gdc(-0.1, 0.0, 0.0).with_lens_shading(1.2, 1.0)),
            Box::new(DisplayBlock::new(w)),
        ], &engine_name, opset);
    }

    // 5. Unpack + Display (skip demosaic/warp)
    try_build("unpack+display", vec![
        Box::new(UnpackBlock::new().with_concrete_dims(h as i64, w as i64)),
        Box::new(DisplayBlock::new(w)),
    ], &engine_name, 13);

    // 6. Unpack + WarpGrid + Display (skip demosaic)
    try_build("unpack+warp+display", vec![
        Box::new(UnpackBlock::new().with_concrete_dims(h as i64, w as i64)),
        Box::new(WarpGridBlock::new(w, h).with_gdc(-0.1, 0.0, 0.0)),
        Box::new(DisplayBlock::new(w)),
    ], &engine_name, 13);

    // 7. DemosaicCcm + Display (skip unpack)
    try_build("demosaic+warp+display", vec![
        Box::new(DemosaicCcmBlock::new(0)),
        Box::new(WarpGridBlock::new(w, h).with_gdc(-0.1, 0.0, 0.0)),
        Box::new(DisplayBlock::new(w)),
    ], &engine_name, 13);

    println!("\nDone.");
}
