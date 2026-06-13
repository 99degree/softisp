// Test with LITE profile
use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use cam_isp::pipeline::IspBlock;
use cam_isp::engine::IspEngine;
use cam_isp::mnn::MnnEngine;
use cam_isp::mnn::MnnBackend;

fn main() {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let mut blocks = PipelineProfile::LITE.build_blocks(8, 2);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx_model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    std::fs::write("test_lite.onnx", &onnx_model).unwrap();
    println!("LITE ONNX: {} bytes", onnx_model.len());
    
    let head = blocks.remove(0);
    let mut engine = MnnEngine::new(MnnBackend::Cpu);
    engine.build(head, blocks, None, 16).unwrap();
    println!("MNN engine built successfully!");
}
