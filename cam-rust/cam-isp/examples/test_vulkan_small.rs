//! Test Vulkan MNN with small model.

use cam_isp::blocks::*;
use cam_isp::pipeline::IspBlock;

fn main() {
    cam_isp::init();
    
    // Register engines
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Cpu);
    
    // Small 64x64 test - just unpack + display (no demosaic or warp)
    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(UnpackBlock::new().with_concrete_dims(64, 64)),
        Box::new(DisplayBlock::new(64)),
    ];
    
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    println!("ONNX: {} bytes", onnx.len());
    
    // Save
    let _ = std::fs::write("test_vulkan.onnx", &onnx);
    println!("Saved test_vulkan.onnx");
    
    // Try MNN engine
    let engine_name = std::env::var("ENGINE").unwrap_or_else(|_| "mnn_cpu".to_string());
    let mut engine = cam_isp::engine::select_engine_by_name(&engine_name)
        .expect("No engine found");
    println!("Engine: {}", engine.backend_name());
    
    let head = blocks.remove(0);
    let all = blocks;
    match engine.build(head, all, None, 21) {
        Ok(()) => println!("Build OK!"),
        Err(e) => println!("Build FAILED: {:?}", e),
    }
}
