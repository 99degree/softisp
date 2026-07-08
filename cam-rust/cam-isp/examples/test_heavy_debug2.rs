use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;
use cam_isp::engine::ProcessParams;

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    
    let mut blocks = PipelineProfile::HEAVY.build_blocks(1920, 0);
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    println!("ONNX size: {}", onnx.len());
    
    // Save ONNX
    let path = std::env::temp_dir().join("heavy_model_debug.onnx");
    std::fs::write(&path, &onnx).unwrap();
    
    // Use MNN to load and print model info
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;
    
    // Build with verbose logging
    match engine.build(head, aux, None, 21) {
        Ok(_) => println!("Build OK"),
        Err(e) => println!("Build ERR: {:?}", e),
    }
    
    // Check output shapes
    let raw: Vec<u8> = vec![128u8; (1920 * 1080 * 2) as usize];
    let params = ProcessParams::new(1920, 1080, &raw);
    match engine.process(&params) {
        Ok(f) => println!("OK: {}×{} data={}", f.width, f.height, f.data.len()),
        Err(e) => println!("ERR: {:?}", e),
    }
}
